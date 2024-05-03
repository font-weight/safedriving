
#include <WebServer.h>
#include <WiFi.h>
#include <SPI.h>
#include <SD.h>

#include "Wire.h" 
#include "I2Cdev.h" 
#include "MPU6050.h" 

#define DBG_OUTPUT_PORT Serial

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

bool blinkState = false;


// ____________________Настройки_____________________________________
const byte impulse_num = 8; // сколько импульсов приходится на 1 метр в датчике спидометра
const byte max_speed = 3; // максимальная разрешенная скорость в км/ч
const int NUM_READ = 10;  // количество усреднений для средних арифм. фильтров





volatile uint32_t last_period = 0;
volatile bool is_t = false;

volatile uint32_t debounce;

uint32_t last_time = 0;

uint32_t accel_timer = 0;
uint32_t speed_timer = 0;

uint32_t filtered_period = 0;
uint32_t cur_period = 0;
float cur_speed = 0;
float lateral_accel = 0;

byte static_speed = 0;

byte violations_val = 0;
byte lateral_violations = 0;


// ___________________________________________________________________

const char *ssid = "wifiESP";  // имя точки доступа, которую будет раздавать ESP
const char *password = "12345678";

byte num;

WebServer server(80);  // порт работы


static bool hasSD = false;
File uploadFile;


void returnOK() {
  server.send(200, "text/plain", "");
}

void returnFail(String msg) {
  server.send(500, "text/plain", msg + "\r\n");
}

bool loadFromSdCard(String path){
  String dataType = "text/plain";
  if(path.endsWith("/")) path += "index.html";

  if(path.endsWith(".src")) path = path.substring(0, path.lastIndexOf("."));
  else if(path.endsWith(".html")) dataType = "text/html";
  else if(path.endsWith(".css")) dataType = "text/css";
  else if(path.endsWith(".js")) dataType = "application/javascript";
  else if(path.endsWith(".png")) dataType = "image/png";
  else if(path.endsWith(".gif")) dataType = "image/gif";
  else if(path.endsWith(".jpg")) dataType = "image/jpeg";
  else if(path.endsWith(".ico")) dataType = "image/x-icon";
  else if(path.endsWith(".xml")) dataType = "text/xml";
  else if(path.endsWith(".pdf")) dataType = "application/pdf";
  else if(path.endsWith(".zip")) dataType = "application/zip";

  File dataFile = SD.open(path.c_str());
  if(dataFile.isDirectory()){
    path += "/index.html";
    dataType = "text/html";
    dataFile = SD.open(path.c_str());
  }

  if (!dataFile)
    return false;

  if (server.hasArg("download")) dataType = "application/octet-stream";

  if (server.streamFile(dataFile, dataType) != dataFile.size()) {
    DBG_OUTPUT_PORT.println("Sent less data than expected!");
  }

  dataFile.close();
  return true;
}

void handleFileUpload(){
  if(server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    if(SD.exists((char *)upload.filename.c_str())) SD.remove((char *)upload.filename.c_str());
    uploadFile = SD.open(upload.filename.c_str(), FILE_WRITE);
    DBG_OUTPUT_PORT.print("Upload: START, filename: "); DBG_OUTPUT_PORT.println(upload.filename);
  } else if(upload.status == UPLOAD_FILE_WRITE){
    if(uploadFile) uploadFile.write(upload.buf, upload.currentSize);
    DBG_OUTPUT_PORT.print("Upload: WRITE, Bytes: "); DBG_OUTPUT_PORT.println(upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(uploadFile) uploadFile.close();
    DBG_OUTPUT_PORT.print("Upload: END, Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
  }
}

void deleteRecursive(String path){
  File file = SD.open((char *)path.c_str());
  if(!file.isDirectory()){
    file.close();
    SD.remove((char *)path.c_str());
    return;
  }

  file.rewindDirectory();
  while(true) {
    File entry = file.openNextFile();
    if (!entry) break;
    String entryPath = path + "/" +entry.name();
    if(entry.isDirectory()){
      entry.close();
      deleteRecursive(entryPath);
    } else {
      entry.close();
      SD.remove((char *)entryPath.c_str());
    }
    yield();
  }

  SD.rmdir((char *)path.c_str());
  file.close();
}

void handleDelete(){
  if(server.args() == 0) return returnFail("BAD ARGS");
  String path = server.arg(0);
  if(path == "/" || !SD.exists((char *)path.c_str())) {
    returnFail("BAD PATH");
    return;
  }
  deleteRecursive(path);
  returnOK();
}

void handleCreate(){
  if(server.args() == 0) return returnFail("BAD ARGS");
  String path = server.arg(0);
  if(path == "/" || SD.exists((char *)path.c_str())) {
    returnFail("BAD PATH");
    return;
  }

  if(path.indexOf('.') > 0){
    File file = SD.open((char *)path.c_str(), FILE_WRITE);
    if(file){
      // file.write((const char *)0, 1);  // дичь какая-то
      file.write(0); 
      file.close();
    }
  } else {
    SD.mkdir((char *)path.c_str());
  }
  returnOK();
}

void printDirectory() {
  if(!server.hasArg("dir")) return returnFail("BAD ARGS");
  String path = server.arg("dir");
  if(path != "/" && !SD.exists((char *)path.c_str())) return returnFail("BAD PATH");
  File dir = SD.open((char *)path.c_str());
  path = String();
  if(!dir.isDirectory()){
    dir.close();
    return returnFail("NOT DIR");
  }
  dir.rewindDirectory();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/json", "");
  WiFiClient client = server.client();

  server.sendContent("[");
  for (int cnt = 0; true; ++cnt) {
    File entry = dir.openNextFile();
    if (!entry)
    break;

    String output;
    if (cnt > 0)
      output = ',';

    output += "{\"type\":\"";
    output += (entry.isDirectory()) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += entry.name();
    output += "\"";
    output += "}";
    server.sendContent(output);
    entry.close();
 }
 server.sendContent("]");
 dir.close();
}

void handleNotFound(){
  if(hasSD && loadFromSdCard(server.uri())) return;
  String message = "SDCARD Not Detected\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " NAME:"+server.argName(i) + "\n VALUE:" + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  DBG_OUTPUT_PORT.print(message);
}

void get_cur_speed() {
  server.send(200, "text/plain", String(cur_speed)); // отправляем ответ о выполнении
}


void get_violations() {
  server.send(200, "text/plain", String(violations_val)); // отправляем ответ о выполнении
}

void reset_violations() {
  violations_val = 0;
  lateral_violations = 0;
  server.send(200, "text/plain", String(violations_val + lateral_violations)); // отправляем ответ о выполнении
}

void get_cur_accel() {
  server.send(200, "text/plain", String(lateral_accel)); // отправляем ответ о выполнении
}


void get_lateral_violations() {
  server.send(200, "text/plain", String(lateral_violations)); // отправляем ответ о выполнении
}

// void reset_lateral_violations() {
//   lateral_violations = 0;
//   server.send(200, "text/plain", String(lateral_violations)); // отправляем ответ о выполнении
// }

void spdlst() {       // при прерывании выполнять код:
  if (millis() - debounce >= 30 && !digitalRead(13)) {
    last_period = millis() - debounce;
    debounce = millis();
  is_t = true;
  }
}

void setup(void){
  Wire.begin();
  accelgyro.initialize();

  DBG_OUTPUT_PORT.begin(115200);
  DBG_OUTPUT_PORT.setDebugOutput(true);
  DBG_OUTPUT_PORT.print("\n");
  
  Serial.println("Testing device connections..."); 
  if (accelgyro.testConnection()) { 
    Serial.println("MPU6050 connection successful"); 
  } else { 
    Serial.println("MPU6050 connection failed"); 
    while (1); 
  }

  pinMode(13, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(13), spdlst, FALLING);

  DBG_OUTPUT_PORT.begin(115200);
  DBG_OUTPUT_PORT.setDebugOutput(true);
  DBG_OUTPUT_PORT.print("\n");
  WiFi.softAP(ssid, password);  // создаем точку доступа

  server.begin();                    // инициализируем Web-server

  DBG_OUTPUT_PORT.print("\nMy IP to connect via Web-Browser or FTP: ");
  DBG_OUTPUT_PORT.println(WiFi.softAPIP());  // выводим локальный IP-адресс
  DBG_OUTPUT_PORT.println("\n");

  DBG_OUTPUT_PORT.print("Connected! IP address: ");
  DBG_OUTPUT_PORT.println(WiFi.localIP());


  server.on("/list", HTTP_GET, printDirectory);
  server.on("/edit", HTTP_DELETE, handleDelete);
  server.on("/edit", HTTP_PUT, handleCreate);
  server.on("/edit", HTTP_POST, [](){ returnOK(); }, handleFileUpload);
  server.on("/get_cur_speed", get_cur_speed);     // получить текущую скорость по запросу вида /get_cur_speed
  server.on("/get_violations", get_violations);     // получить количество превышений по запросу вида /get_violations
  server.on("/reset_violations", reset_violations);     // сбросить нарушение скорости по запросу вида /reset_violations
  server.on("/get_lateral_violations", get_lateral_violations);     // получить нарушения по перестроению по запросу
  // server.on("/reset_lateral_violations", reset_lateral_violations);     // сбросить нарушение по перестроению по запросу
  server.on("/get_cur_accel", get_cur_accel);     // получить текущее ускорение по запросу вида /get_cur_accel
  server.onNotFound(handleNotFound);

  server.begin();
  DBG_OUTPUT_PORT.println("HTTP server started");

  
    while (true) {
      if (SD.begin()) {
        DBG_OUTPUT_PORT.println("SD Card initialized.");
        hasSD = true;
        break;
      }
      delay(500);
      // DBG_OUTPUT_PORT.println("SD Card not initialized.");
    }
  
}



void loop(void){
  server.handleClient();
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 
  lateral_accel = (float)ax / 1672;
  if (fabs((lateral_accel) > 5.0) and (millis() - accel_timer > 1000) ) {
    lateral_violations++;
    accel_timer = millis();
  }

  if (is_t) {
    filtered_period = filtered(last_period);
    // Serial.println(filtered_period);
    is_t = false;
    cur_speed = float(3600.0 / (impulse_num * filtered_period));
    
  }

  if (millis() - last_time >= 500) {
    
    if (cur_period == filtered_period) {
      static_speed++;
    } else {
      cur_period = filtered_period;
    }
    if (static_speed >= 5) {
      static_speed = 0;
      cur_speed = 0;
    }
    // Serial.println(cur_speed);
    if ((cur_speed > max_speed) and (millis() - speed_timer > 2000)) {      // если превысил скорость добовляем нарушение
      // Serial.println("Нарушение!");
      violations_val++;
      speed_timer = millis();
    }
    last_time = millis();

  }

}


float filtered(float newVal) {        // функция фильтрации 
  static int t = 0;
  static float vals[NUM_READ];
  static float average = 0;
  if (++t >= NUM_READ) t = 0; // перемотка t
  average -= vals[t];         // вычитаем старое
  average += newVal;          // прибавляем новое
  vals[t] = newVal;           // запоминаем в массив
  return ((float)average / NUM_READ);
}
