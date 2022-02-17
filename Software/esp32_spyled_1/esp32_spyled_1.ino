/*
 *    - Get Tempal Temperature and Battery Voltage from MCU -> Send to update web service.
 *    - Get Panel, Examination, Focus, Level States from list web service -> Send to MCU.
 * 
 */

#include "WiFi.h"
#include "HTTPClient.h"
#include <Arduino_JSON.h>
 
const char* ssid = "ErtuncOzcanKat2";
const char* password =  "ertarge2017";

const char* serverName = "https://espprojecteo.herokuapp.com/api/esp/update/1";
const char* serverNameGet = "https://espprojecteo.herokuapp.com/api/esp/list";

int httpResponseCode ;
String start, focus, exam, level;
String start_old="0", focus_old="0", exam_old="0", level_old="0";
byte tempData;
byte tempalArray[4];
int counter = 0;
String temperature = "";
String battery = "";
uint8_t vbat=0;
uint16_t tempal_temp = 0;
uint16_t tempal_vbat = 0;
bool isConnected = 0;

void setup() {
 
  Serial.begin(115200);
  Serial2.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial2.write("c");
}
 
void loop() {
 if(WiFi.status()== WL_CONNECTED){
   HTTPClient http;   
   http.begin(serverNameGet);
   httpResponseCode = http.GET();
   String payload = "{}";
   if (httpResponseCode>0)
   {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();

    JSONVar myObject = JSON.parse(payload);
    if (JSON.typeof(myObject) == "undefined")
    {
      Serial.println("Parsing input failed!");
      return;
    }
    Serial.print("JSON object = ");
    Serial.println(myObject[0]);

    if(myObject[0].hasOwnProperty("start"))
    {
      Serial.print("myObject[\"start\"] = ");
      Serial.println(myObject[0]["start"]);
      start = myObject[0]["start"];
      if(start != start_old)
      {
        Serial2.write("S");
      }
      start_old = start;
    }
    if(myObject[0].hasOwnProperty("focus"))
    {
      Serial.print("myObject[\"focus\"] = ");
      Serial.println(myObject[0]["focus"]);
      focus = myObject[0]["focus"];
      if(focus != focus_old)
      {
        Serial2.write("F");
      }
      focus_old = focus;
    }
    if(myObject[0].hasOwnProperty("exam"))
    {
      Serial.print("myObject[\"exam\"] = ");
      Serial.println(myObject[0]["exam"]);
      exam = myObject[0]["exam"];
      if(exam != exam_old)
      {
        Serial2.write("E");
      }
      exam_old = exam;
    }
    if(myObject[0].hasOwnProperty("level"))
    {
      Serial.print("myObject[\"level\"] = ");
      Serial.println(myObject[0]["level"]);
      level = myObject[0]["level"];
      if(level != level_old)
      {
        Serial2.print(level);
      }
      level_old = level;
    }
    /*if (myObject[0].hasOwnProperty("slug"))
    {
      Serial.print("myObject[\"slug\"] = ");
      Serial.println(myObject[0]["slug"]);
      slug = myObject[0]["slug"];
    }*/
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }

  Serial2.print("t");
  temperature = "";
  counter = 0;
  while(Serial2.available() > 0)
  {
    tempalArray[counter] = Serial2.read();
    counter++;
  }
  
  tempal_temp = tempalArray[2];
  tempal_temp = (tempal_temp<<8)+tempalArray[3];

  tempal_vbat = tempalArray[0];
  tempal_vbat = (tempal_vbat<<8)+tempalArray[1];
  
  Serial.print(" Temperature :");
  Serial.println(tempal_temp);
  Serial.print(" Battery :");
  Serial.println(tempal_vbat);

  /*if(tempal_vbat>3985){vbat = 100;}
  else if(3985>=tempal_vbat && tempal_vbat>3841){vbat = 80;}
  else if(3841>=tempal_vbat && tempal_vbat>3765){vbat = 60;}
  else if(3765>=tempal_vbat && tempal_vbat>3685){vbat = 40;}
  else{vbat = 20;}*/

  if(tempal_vbat>4000){vbat = 100;}          // if battery voltage of tempal greater than 4000 mv, level %100
  else if(4000>=tempal_vbat && tempal_vbat>3685){vbat = ((90*tempal_vbat)/314)-1046;} // 4000mv = %100, 3686mv = %10
  else if(3685>=tempal_vbat && tempal_vbat>=3675){ vbat = tempal_vbat - 3675;} // 3685mv = %9 - 3675mv = %0
  else{vbat = 0;} // %0
    
  http.begin(serverName);
  
  String jsonTemp = "{\"temperature\":\"";
  jsonTemp += String(tempal_temp);
  jsonTemp += "\"}";
  Serial.println(jsonTemp);
  http.addHeader("Content-Type", "application/json");          
  httpResponseCode = http.PUT(jsonTemp);
   
  if(httpResponseCode>0)
  {
    String response = http.getString();   
    Serial.println(httpResponseCode);          
  }
  else
  {
    Serial.print("Error on sending PUT Request: ");
    Serial.println(httpResponseCode);
  }

  delay(20);

  String jsonVbat = "{\"battery\":\"";
  jsonVbat += String(vbat);
  jsonVbat += "\"}";
  Serial.println(jsonVbat);
  http.addHeader("Content-Type", "application/json");          
  httpResponseCode = http.PUT(jsonVbat);
   
  if(httpResponseCode>0)
  {
    String response = http.getString();   
    Serial.println(httpResponseCode);          
  }
  delay(20);

  isConnected = !isConnected;
  String jsonIsConnected = "{\"isConnected\":\"";
  jsonIsConnected += String(isConnected);
  jsonIsConnected += "\"}";
  Serial.println(jsonIsConnected);
  http.addHeader("Content-Type", "application/json");          
  httpResponseCode = http.PUT(jsonIsConnected);
   
  if(httpResponseCode>0)
  {
    String response = http.getString();   
    Serial.println(httpResponseCode);          
  }
  else
  {
    Serial.print("Error on sending PUT Request: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
 }
 else
 {
    WiFi.disconnect();
    Serial2.write("d");
    Serial.println("Error in WiFi connection");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Reconnecting ...");
      Serial2.print("e");
    }
    Serial.println("Reconnected");
 }
 delay(200);
}
