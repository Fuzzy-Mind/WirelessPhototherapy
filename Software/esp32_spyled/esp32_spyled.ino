/*
 *    - Get skin sensor value from MCU -> Send to web update service.
 *    - Get Panel, Examination, Focus states from web list service -> Send to MCU.
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
char tempData;
String temperature = "";
char battery;

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
  while(Serial2.available() > 0)
  {
    tempData = Serial2.read();
    temperature += tempData;
  }
  Serial.print(" Temperature :");
  Serial.println(temperature);

  Serial2.print("b");
  battery = "";
  while(Serial2.available() > 0)
  {
    tempData = Serial2.read();
    temperature += tempData;
  }
  Serial.print(" Temperature :");
  Serial.println(temperature);
  
  http.begin(serverName);
  
  String jsonTemp = "{\"temperature\":\"";
  jsonTemp += temperature;
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
  http.end();
 }
 else
 {
    Serial.println("Error in WiFi connection");
    while(1)
    {
      delay(100);
      WiFi.disconnect();
      delay(100);
      WiFi.reconnect();
      Serial.println("Reconnecting ...");
      delay(500);
      Serial2.print("e");
      if(WiFi.status() == WL_CONNECTED)
      {
        Serial.println("Reconnected");
        break;
      }
    }
 }
 delay(400);
}
