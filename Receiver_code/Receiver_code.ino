#include <HTTPClient.h>
#include <WiFi.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "WebServer.h"

#include <ThingSpeak.h>
#include <PubSubClient.h>

#include <LoRa.h>
#include <SPI.h>

#define ss 5
#define rst 22
#define dio0 2

#define MAIN_SSID "Galaxy A224D5E"
#define MAIN_PASS "123456789"
#define CSE_IP "esw-onem2m.iiit.ac.in"
#define CSE_PORT 443
#define OM2M_ORGIN "fIsOtR:&LTnM1"
#define OM2M_MN "/~/in-cse/in-name/"
#define OM2M_AE "Team-38"
#define OM2M_DATA_CONT "Node-1/Data"

Adafruit_MPU6050 mpu; 

const char * ntpServer = "pool.ntp.org";

const char* server = "mqtt3.thingspeak.com";

const char* mqttUserName = "NjYzOAY7DyYQNywgFDcRIRA";

const char* mqttPass = "eZ4fBcf5JJ1QsIg0+jDZILyI";

const char* clientID="NjYzOAY7DyYQNywgFDcRIRA";

long writeChannelID = 1957365;

const char* writeAPIKey = "06MRSX6FV4DWKFIG";

WiFiClient client;

//PubSubClient mqttClient(client);

int port = 1883;

String fall_value, Lat, Lng, LoRaData;

HTTPClient http;

void createCI()
{
    String data;
    String server = "https://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;

    http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_CONT + "/");

    http.addHeader("X-M2M-Origin", OM2M_ORGIN);
    http.addHeader("Content-Type", "application/json;ty=4");
    http.addHeader("Content-Length", "100");

    data = "[" + String(fall_value) + ", " + String(Lat) + ", " + String(Lng) +   + "]"; 
    String req_data = String() + "{\"m2m:cin\": {"

      +
      "\"con\": \"" + data + "\","

      +
      "\"lbl\": \"" + "V1.0.0" + "\","

      //+ "\"rn\": \"" + "cin_"+String(i++) + "\","

      +
      "\"cnf\": \"text\""

      +
      "}}";
    int code = http.POST(req_data);
    Serial.println(code);
    if (code == -1) 
    {
      Serial.println("UNABLE TO CONNECT TO THE SERVER");
    }
    http.end();
}


void setup() {
  Serial.begin(115200);
  //delay(1000);
  while (!Serial);
  Serial.println("LoRa Receiver");
 
  LoRa.setPins(ss, rst, dio0);    //setup LoRa transceiver module
 
  while (!LoRa.begin(433E6))     //433E6 - Asia, 866E6 - Europe, 915E6 - North America
  {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");

  WiFi.mode(WIFI_STA);
  WiFi.begin(MAIN_SSID, MAIN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("Connecting...");
  }
  Serial.print("Connected to ");
  Serial.println(MAIN_SSID);
  Serial.println("IP address is : ");
  Serial.println(WiFi.localIP());
  ThingSpeak.begin(client);
}

void loop() {
  Lat = "", Lng = "";
  
  int packetSize = LoRa.parsePacket();    // try to parse packet
  
  if (packetSize) 
  { 
    
    Serial.print("Received packet '");
    
    while (LoRa.available())              // read packet
    {
      LoRaData = LoRa.readString();
      Serial.print(LoRaData); 
      
    }
    Serial.print("' with RSSI ");         // print RSSI of packet
    Serial.println(LoRa.packetRssi());
    
    int i;
    if(LoRaData[0]=='1')
    {
      fall_value = "1";
      ThingSpeak.setField(1, fall_value);
      Serial.println("Fall Detected");      
      for(i=2; ; i++)
      {
        if(LoRaData[i]=='/')
        {
          break;
        }
        Lat += LoRaData[i];
      }
      
      ThingSpeak.setField(2, Lat);
      
      i++;

      int len = sizeof(LoRaData)/sizeof(LoRaData[0]);
      Serial.print("len= ");
      Serial.println(len);
      int pos;
      for(;LoRaData[i]!=0; i++) {
        Lng += LoRaData[i];
      }
      ThingSpeak.setField(3, Lng);

      Serial.println(Lat);
      Serial.println(Lng);
      
      int x = ThingSpeak.writeFields(writeChannelID, writeAPIKey);
      if (x == 200) {
       Serial.println("Channel update successful");
      }
      else {
      Serial.println("Problem updating channel. HTTP error code " + String(x) + "\n");
      }

      createCI();

      Serial.println("done");
    }
  }
  else
  {
    fall_value = "0";
    ThingSpeak.setField(1, fall_value);
    int x = ThingSpeak.writeFields(writeChannelID, writeAPIKey);
      if (x == 200) {
       Serial.println("Channel update successful");
      }
      else {
      Serial.println("Problem updating channel. HTTP error code " + String(x) + "\n");
      }
  }
  //delay(500);
}
