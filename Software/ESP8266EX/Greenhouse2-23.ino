#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "AsyncJson.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
//#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//Old Unsorted Defines
#define SSID "Greenhouse"
#define ID_COUNT 3
//
#define SENSORS_MAX 19
#define PLANT_PROFILES_MAX 10
#define FLOAT_SENSOR 0
#define TDS_SENSOR 1
#define PH_SENSOR 2

struct plant_profile {
  char name[25];
  unsigned char system_num;
  unsigned short tds_min;
  unsigned short ph_max; //Conversion: (ph_x/65535)*14
  unsigned short ph_min;
};
struct sensor {
  unsigned char system_num; //0 = overall, 4 = disabled. Used for additional float switches
  unsigned char sensor_type;
  unsigned char sensor_num;
  unsigned char mux_num;
  unsigned char mux_input_num;
  unsigned char driver_up_num;
  unsigned char driver_up_input_num;
  unsigned char driver_down_num;
  unsigned char driver_down_input_num;
  unsigned short current_level;
  unsigned short max_level;
  unsigned short min_level;
};
//Old Unsorted Vars
unsigned char locked = 1;
unsigned char serial_data[33];
unsigned char serial_data_idx = 0;
unsigned char serial_expected = 16;
float pH = -1;
short tds = -1;
//Plant Profiles
struct plant_profile plant_profiles[PLANT_PROFILES_MAX];
String config_profile_id ="-1";
//Sensors
struct sensor sensors[SENSORS_MAX];
unsigned char number_of_systems = 1;


/*
   Order of bits starting from bit 0: phosphorus, nitrogen, potassium
   calcium, magnesium
*/
unsigned char nutrientReservoirLevels = 0b00000000; //(IE phosphorus (ID=2) is
unsigned long times[ID_COUNT];  //times identified by id
unsigned long lastUpdated;

AsyncWebServer server(80);

String convertTime(unsigned long t) {
  float ret_t = (float)(millis() - t) / 1000; //Convert to seconds
  if (ret_t >= 60) ret_t = ret_t / 60; //Convert to minutes
  else return String(ret_t) + "s ago";
  if (ret_t >= 60) ret_t = ret_t / 60; //Convert to hours
  else return String(ret_t) + "m ago";
  if (ret_t >= 24) ret_t = ret_t / 24; //Convert to hours
  return String(ret_t) + "d ago";
}

String processorHTML(const String &var) {
  unsigned char temp;
  if (var == "pH") {
    return String(pH);
  } else if (var == "tds") {
    return String(tds);
  } else if (var == "phosphorus") {
    temp = ((nutrientReservoirLevels >> 0) & 0b00000001);
    return temp ? String("Good") : String("Low");
  } else if (var == "nitrogen") {
    temp = ((nutrientReservoirLevels >> 1) & 0b00000001);
    return temp ? String("Good") : String("Low");
  } else if (var == "potassium") {
    temp = ((nutrientReservoirLevels >> 2) & 0b00000001);
    return temp ? String("Good") : String("Low");
  } else if (var == "calcium") {
    temp = ((nutrientReservoirLevels >> 3) & 0b00000001);
    return temp ? String("Good") : String("Low");
  } else if (var == "magnesium") {
    temp = ((nutrientReservoirLevels >> 4) & 0b00000001);
    return temp ? String("Good") : String("Low");
  } else if (var == "pHTime") {
    return String(millis() - times[0]);
  } else if (var == "tdsTime") {
    return String(millis() - times[1]);
  } else if (var == "nutrientReservoirTime") {
    return String(millis() - times[2]);
  }
  return "Invalid";
}

String configProcessor(const String &var){
  locked=1;
  //Serial.write(config_profile_id);
  locked=0;
  return config_profile_id;
}

String * processorVariables(const String &var) {
  String t[5];
  t[0] = "abc";
  t[1] = "bcd";
  return t;
}

void setup() {
  delay(1000);
  Serial.begin(9600);
  LittleFS.begin();
  WiFi.softAP(SSID);
  //Setup HTTP server and begin
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/index.html", String(), false, processorHTML);
  });
  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/index.html", String(), false, processorHTML);
  });
  server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/styles.css", "text/css");
  });
  server.on("/vars.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    getSensorData();
    getPlantProfiles();
    delay(100);
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument doc(8000);
    JsonArray json_array = doc.to<JsonArray>();
    for (unsigned char i = 0; i < SENSORS_MAX; i++) {
      JsonObject json_sensor = json_array.createNestedObject();
      json_sensor["system_num"] = sensors[i].system_num;
      json_sensor["sensor_num"] = sensors[i].sensor_num;
      json_sensor["sensor_type"] = sensors[i].sensor_type;
      json_sensor["driver_up_num"] = sensors[i].driver_up_num;
      json_sensor["driver_up_input_num"] = sensors[i].driver_up_input_num;
      json_sensor["driver_down_num"] = sensors[i].driver_down_num;
      json_sensor["driver_down_input_num"] = sensors[i].driver_down_input_num;
      json_sensor["current_level"] = sensors[i].current_level;
      json_sensor["max_level"] = sensors[i].max_level;
      json_sensor["min_level"] = sensors[i].min_level;
    }
    response->print("sensors=");
    serializeJson(doc, *response);
    response->print(";\n");
    response->print("plant_profiles=");
    doc.clear();
    JsonArray json_array1 = doc.to<JsonArray>();
      for (unsigned char i = 0; i < PLANT_PROFILES_MAX; i++) {
      JsonObject json_plant_profile = json_array1.createNestedObject();
      json_plant_profile["name"] = plant_profiles[i].name;
      json_plant_profile["system_num"] = plant_profiles[i].system_num;
      json_plant_profile["tds_min"] = plant_profiles[i].tds_min;
      json_plant_profile["ph_max"] = plant_profiles[i].ph_max;
      json_plant_profile["ph_min"] = plant_profiles[i].ph_min;
    }
    serializeJson(doc, *response);
    doc.clear();
    response->print(";\n");
    response->print("lastUpdated=");
    JsonVariant json_last_updated = doc.to<JsonVariant>();
    json_last_updated.set(millis() - lastUpdated);
    serializeJson(doc, *response);
    request->send(response);
  });
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/favicon.ico", "image/x-icon");
  });
  server.on("/favicon.png", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/favicon.png", "image/x-icon");
  });
  server.on("/config.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/config.html", String());
  });
  server.on("/getSensorData", HTTP_GET, [](AsyncWebServerRequest * request) {
    getSensorData();
    getPlantProfiles();
    delay(100);
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument doc(8000);
    JsonArray json_array = doc.to<JsonArray>();
    for (unsigned char i = 0; i < SENSORS_MAX; i++) {
      JsonObject json_sensor = json_array.createNestedObject();
      json_sensor["system_num"] = sensors[i].system_num;
      json_sensor["sensor_num"] = sensors[i].sensor_num;
      json_sensor["sensor_type"] = sensors[i].sensor_type;
      json_sensor["driver_up_num"] = sensors[i].driver_up_num;
      json_sensor["driver_up_input_num"] = sensors[i].driver_up_input_num;
      json_sensor["driver_down_num"] = sensors[i].driver_down_num;
      json_sensor["driver_down_input_num"] = sensors[i].driver_down_input_num;
      json_sensor["current_level"] = sensors[i].current_level;
      json_sensor["max_level"] = sensors[i].max_level;
      json_sensor["min_level"] = sensors[i].min_level;
    }
    response->print("[");
    serializeJson(doc, *response);
    response->print(",");
    //response->print("plant_profiles=");
    doc.clear();
    JsonArray json_array1 = doc.to<JsonArray>();
      for (unsigned char i = 0; i < PLANT_PROFILES_MAX; i++) {
      JsonObject json_plant_profile = json_array1.createNestedObject();
      json_plant_profile["name"] = plant_profiles[i].name;
      json_plant_profile["system_num"] = plant_profiles[i].system_num;
      json_plant_profile["tds_min"] = plant_profiles[i].tds_min;
      json_plant_profile["ph_max"] = plant_profiles[i].ph_max;
      json_plant_profile["ph_min"] = plant_profiles[i].ph_min;
    }
    serializeJson(doc, *response);
    doc.clear();
    response->print(",");
    //response->print("lastUpdated=");
    JsonVariant json_last_updated = doc.to<JsonVariant>();
    json_last_updated.set(millis() - lastUpdated);
    serializeJson(doc, *response);
    response->print("]");
    request->send(response);
  });
  server.on("/updateSensorData", HTTP_GET, [](AsyncWebServerRequest * request) {
    updateSensorData();
    //delay(100);
    //getSensorData();
    getPlantProfiles();
    delay(100);
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument doc(8000);
    JsonArray json_array = doc.to<JsonArray>();
    for (unsigned char i = 0; i < SENSORS_MAX; i++) {
      JsonObject json_sensor = json_array.createNestedObject();
      json_sensor["system_num"] = sensors[i].system_num;
      json_sensor["sensor_num"] = sensors[i].sensor_num;
      json_sensor["sensor_type"] = sensors[i].sensor_type;
      json_sensor["driver_up_num"] = sensors[i].driver_up_num;
      json_sensor["driver_up_input_num"] = sensors[i].driver_up_input_num;
      json_sensor["driver_down_num"] = sensors[i].driver_down_num;
      json_sensor["driver_down_input_num"] = sensors[i].driver_down_input_num;
      json_sensor["current_level"] = sensors[i].current_level;
      json_sensor["max_level"] = sensors[i].max_level;
      json_sensor["min_level"] = sensors[i].min_level;
    }
    response->print("[");
    serializeJson(doc, *response);
    response->print(",");
    //response->print("plant_profiles=");
    doc.clear();
    JsonArray json_array1 = doc.to<JsonArray>();
      for (unsigned char i = 0; i < PLANT_PROFILES_MAX; i++) {
      JsonObject json_plant_profile = json_array1.createNestedObject();
      json_plant_profile["name"] = plant_profiles[i].name;
      json_plant_profile["system_num"] = plant_profiles[i].system_num;
      json_plant_profile["tds_min"] = plant_profiles[i].tds_min;
      json_plant_profile["ph_max"] = plant_profiles[i].ph_max;
      json_plant_profile["ph_min"] = plant_profiles[i].ph_min;
    }
    serializeJson(doc, *response);
    doc.clear();
    response->print(",");
    //response->print("lastUpdated=");
    JsonVariant json_last_updated = doc.to<JsonVariant>();
    json_last_updated.set(millis() - lastUpdated);
    serializeJson(doc, *response);
    response->print("]");
    request->send(response);
  });
  server.on("/editProfile", HTTP_POST, [](AsyncWebServerRequest * request){},NULL, [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total){
    StaticJsonDocument<200> doc;
    deserializeJson(doc, data);
    config_profile_id=String(doc["id"]);
    
    request->send(LittleFS, "/editProfile.html", String(),false, configProcessor);
  });
  server.on("/updateProfile", HTTP_POST, [](AsyncWebServerRequest * request){},NULL, [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total){
    StaticJsonDocument<200> doc;
    deserializeJson(doc, data);
    const char *name= doc["name"];
    unsigned char id = (unsigned char)doc["id"];
    unsigned short tds_min = (unsigned short)doc["tds_min"];
    unsigned short ph_min = (unsigned short)doc["ph_min"];
    unsigned short ph_max = (unsigned short)doc["ph_max"];
    unsigned char system_num = (unsigned char)doc["system_num"];
    //Serial.println(id);
    strcpy(plant_profiles[id].name, name);
    plant_profiles[id].tds_min=tds_min;
    plant_profiles[id].system_num=system_num;
    plant_profiles[id].ph_min=ph_min;
    plant_profiles[id].ph_max=ph_max;
    //Serial.println(plant_profiles[id].name);
    updatePlantProfiles();
    request->send(200);
  });
  server.on("/removeProfile", HTTP_POST, [](AsyncWebServerRequest * request){},NULL, [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total){
    StaticJsonDocument<200> doc;
    deserializeJson(doc, data);
    unsigned char id = (unsigned char)doc["id"];
    unsigned char i;
    for(i=id;i<9;i++){
      plant_profiles[i]=plant_profiles[i+1];
    }
    memset(&plant_profiles[9],0,sizeof(struct plant_profile));
    updatePlantProfiles();
    delay(100);
    request->send(LittleFS, "/config.html", String());
  });
  locked=0;
  updateSensorData();
  //getPlantProfiles();
  //updatePlantProfiles();
  server.begin();
}

void loop() {
  if(Serial.available() && locked!=1){
    unsigned char cmd = Serial.read();
    //Serial.print(cmd);
    switch(cmd){
      case 0:
        //getSensorData();
        lastUpdated = millis();
        break;
    }
  }
}

void getSensorData() {
  while(locked !=0);
  locked=1;
  while(Serial.available()) char t=Serial.read();
  Serial.write(0);
  serial_data[0] = 0;
  serial_data_idx = 1;
  unsigned char buf[60];
  unsigned char j = 0;
  unsigned char i = 0;
  //for (i; i < number_of_systems * 6 + 1; i++) {
  for(i;i<SENSORS_MAX; i++){  
    while (Serial.available() < 16);
    for (j = 0; j < 16; j++) {
      buf[j] = Serial.read();
    }
    memcpy(&sensors[i], buf, sizeof(struct sensor));
  }
  /*for (i = number_of_systems * 6 + 1; i < SENSORS_MAX; i++) {
    sensors[i].system_num = 4;
  }*/
  while(Serial.available()) char t=Serial.read();
  locked=0;
}

void updateSensorData(){
  while(locked !=0);
  locked=1;
  while(Serial.available()) char t=Serial.read();
  Serial.write(2);
  while(!Serial.available());
  if(Serial.read()==0) lastUpdated = millis();
  getSensorData();
  locked=0;
}

void updatePlantProfiles(){
  //strcpy(plant_profiles[0].name,"fern");
  while(Serial.available()) char t=Serial.read();
  Serial.write(3);
  unsigned char buf[sizeof(struct plant_profile)];
  for(unsigned int i=0;i<10;i++){
    memcpy(buf,&plant_profiles[i],sizeof(struct plant_profile));
    for(unsigned int j=0;j<sizeof(struct plant_profile);j++){
      Serial.write(buf[j]);
    }
  }
}

void getPlantProfiles() {
  while(locked !=0);
  locked=1;
  while(Serial.available()) char t=Serial.read();
  Serial.write(1);
  unsigned char buf[60];
  unsigned char i = 0;
  unsigned char j = 0;
  for (i; i < PLANT_PROFILES_MAX; i++) {
    while (Serial.available() < 32);
    for (j = 0; j < 32; j++) {
      buf[j] = Serial.read();
    }
    memcpy(&plant_profiles[i], buf, sizeof(struct plant_profile));
    //for(j=0;plant_profiles[i].name[j]!='\0';j++) Serial.print(plant_profiles[i].name[j]);
    //Serial.print(plant_profiles[i].system_num);
  }
  while(Serial.available()) char t=Serial.read();
  locked=0;
}

/*
   On power up we will initalize the time checked to 0
*/
void initSensorData() {
  unsigned char dataBuf[6];
  size_t readSize;
  File data = LittleFS.open("/sensorData.bin", "r");
  File tempData = LittleFS.open("/sensorData.bin.tmp", "w+");
  while (readSize = data.readBytes((char *)dataBuf, 6)) {
    Serial.write(dataBuf[0]); //Request sensor data
    while (!Serial.available()); //Wait for response
    dataBuf[1] = (unsigned char)Serial.read();
    memset(&dataBuf[2], 0, sizeof(unsigned long));
    tempData.write(dataBuf, 6);
  }
  data.close();
  tempData.close();
  LittleFS.remove("/sensorData.bin");
  LittleFS.rename("/sensorData.bin.tmp", "/sensorData.bin");
  readSensorData();
  locked = 0;
}
/*
   Sensor data will be stored in a binary file.
   Each set of data will be 6 (0 to 5) bytes long broken down as follows:
   Byte 0 = Sensor ID,
   Byte 1 = Sensor value
   Byte 2-5 = Milliseconds based on execution time of ESP8266
*/
//void updateSensorData() {
  //FOR TESTING PURPOSES ONLY
  /*serial_data[0]=(unsigned char)random(0,2);
    Serial.println(serial_data[0]);
    serial_data[1]=(unsigned char)random(0,256);*/
  //END
  /*unsigned char dataBuf[6];
  size_t readSize;
  File data = LittleFS.open("/sensorData.bin", "r");
  File tempData = LittleFS.open("/sensorData.bin.tmp", "w+");
  while (readSize = data.readBytes((char *)dataBuf, 6)) {
    if (dataBuf[0] == serial_data[0]) {
      dataBuf[1] = serial_data[1];
      unsigned long m = millis();
      memcpy(&dataBuf[2], &m, sizeof(unsigned long));
    }
    tempData.write(dataBuf, 6);
  }
  data.close();
  tempData.close();
  LittleFS.remove("/sensorData.bin");
  LittleFS.rename("/sensorData.bin.tmp", "/sensorData.bin");
  readSensorData();
}*/

void readSensorData() {
  unsigned char id;
  unsigned char val;
  unsigned long time;
  unsigned char dataBuf[6];
  size_t readSize;
  File data = LittleFS.open("/sensorData.bin", "r");
  while (readSize = data.readBytes((char *)dataBuf, 6)) {
    id = dataBuf[0];
    val = dataBuf[1];
    time = (unsigned long)dataBuf[5] << 24;
    time |= (unsigned long)dataBuf[4] << 16;
    time |= (unsigned long)dataBuf[3] << 8;
    time |= (unsigned long)dataBuf[2];
    setSensorData(id, val, time);
  }
  data.close();
}
/*
   Sensor IDS:
   0 = pH,
   1 = TDS,
   2 = Nutrient Reservoirs
*/
void setSensorData(unsigned char id, unsigned char val, unsigned long time) {
  switch (id) {
    case 0:
      pH = ((float)val / 255) * 14;
      times[id] = time;
      break;
    case 1:
      tds = ((float)val / 255) * 10000;
      times[id] = time;
      break;
    case 2:
      nutrientReservoirLevels = val;
      times[id] = time;
      break;
    default:
      //Serial.println("Error: Data id not found!");
      break;
  }




}
