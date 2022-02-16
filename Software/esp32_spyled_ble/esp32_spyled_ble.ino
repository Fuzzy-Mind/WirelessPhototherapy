#include "WiFi.h"
#include "HTTPClient.h"
#include <Arduino_JSON.h>
#include "BLEDevice.h"

static BLEUUID serviceUUID("c050");
static BLEUUID    charUUID("c05a");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
 
const char* ssid = "ErtuncOzcanKat2";
const char* password =  "ertarge2017";

const char* serverName = "https://espprojecteo.herokuapp.com/api/esp/update/1";
const char* serverNameGet = "https://espprojecteo.herokuapp.com/api/esp/list";

int httpResponseCode ;
String start, focus, exam, level;
String start_old="0", focus_old="0", exam_old="0", level_old="0";
char tempData;
String temperature = "";

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    uint16_t vBatt;
    uint16_t temperature;
  
    temperature = pData[2];
    temperature = (temperature<<8)+pData[3];
    
    vBatt = pData[0];
    vBatt = (vBatt<<8)+pData[1];
    
    Serial.println(temperature);
    Serial.println(vBatt);
}

/*class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};*/

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    //pClient->setClientCallbacks(new MyClientCallback());
    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");
    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    } // Found our server
  } // onResult
};

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

  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(10, false);
  connectToServer();
  Serial.println("tarama bitti");
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
 /*if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }
  if(doScan)
  {
    Serial.println("doscan");
    BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }
 delay(400);*/
}
