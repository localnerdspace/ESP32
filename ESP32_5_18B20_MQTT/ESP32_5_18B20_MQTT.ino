 /*
 * Original by randomnerdtutorials.com 
 * Adjusted to add more sensors and more mqtt publishers (localnerd.de)
 */
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// If you don´t want to use your own configuration file, comment this one and use the variables below
// make sure you include the MyConfug file into your /Arduino/libraries/MyConfig folder
#include <MyConfig.h>

// Replace the next variables with your SSID/Password combination if you don´t use your own "MyConfig.h" library
//#define WIFI_SSID "SSID"
//#define WIFI_PASSWORD "PASSWORD"
// Add your MQTT Broker IP address
//#define MQTT_HOST IPAddress(xxx, xxx, xxx, xxx)
//#define MQTT_PORT 1883

// Temperature MQTT Topic
#define MQTT_PUB_TOPIC1 "esp32/topics/topic1" 
#define MQTT_PUB_TOPIC2 "esp32/topics/topic2" 
#define MQTT_PUB_TOPIC3 "esp32/topics/topic3" 
#define MQTT_PUB_TOPIC4 "esp32/topics/topic4" 
#define MQTT_PUB_TOPIC5 "esp32/topics/topic5" 
#define MQTT_PUB_TOPIC6 "esp32/topics/topic6"

// GPIO where the DS18B20 are connected to
const int oneWireBus = 4;
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
// to find the address of your DS18B20 please refer to
// https://github.com/localnerdspace/ESP32/blob/main/ESP32_multi_18B20_select_Address/ESP32_multi_18B20_select_Address.ino

DeviceAddress Sensor1 = { 0x28,  0xBE,  0xE5,  0x56,  0xB5,  0x1,  0x3C,  0x83 }; 
DeviceAddress Sensor2 = { 0x28,  0x5D,  0x30,  0x56,  0xB5,  0x1,  0x3C,  0x7F };
DeviceAddress Sensor3 = { 0x28,  0xB8,  0x1,  0x56,  0xB5,  0x1,  0x3C,  0xF4 };
DeviceAddress Sensor4 = { 0x28,  0x28,  0x10,  0x56,  0xB5,  0x1,  0x3C,  0xF4 };
DeviceAddress Sensor5 = { 0x28,  0xF8,  0xDC,  0x56,  0xB5,  0x1,  0x3C,  0x33 };

float SensorValue1;
float SensorValue2;
float SensorValue3;
float SensorValue4;
float SensorValue5;
int DevOn; //signal integer for visualization of device status

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 60000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}


void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}


void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup(){
  Serial.begin(115200);
  sensors.begin();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPLACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  connectToWifi();
  
}

void loop(){ 
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
  //check again if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    connectToWifi();
  }
    // New temperature readings
    sensors.requestTemperatures(); 
    //error handling... Sensor presents sometimes 85.00°C or 25.00°C after power on
    if (sensors.getTempCByIndex(0) == 85.00) { 
      // in case of 85.00 read sensors again
      sensors.requestTemperatures();
    }
    if (sensors.getTempCByIndex(0) == 25.00) { 
      // in case of 25.00 read sensors again
      sensors.requestTemperatures();
    }     
    // Temperature in Celsius degrees
    // array of sensors begin with index=0
    SensorValue1 = sensors.getTempCByIndex(0); //index of sensor 1
    SensorValue2 = sensors.getTempCByIndex(1); //index of sensor 2
    SensorValue3 = sensors.getTempCByIndex(2); //index of sensor 3
    SensorValue4 = sensors.getTempCByIndex(3); //index of sensor 4
    SensorValue5 = sensors.getTempCByIndex(4); //index of sensor 5
        

    // Publish a MQTT message to topic MQTT_PUB_TOPICx
    //conversion from float to string to publish correctly to MQTT
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TOPIC1, 1, true, String(SensorValue1).c_str());
    //for better debugging print everything to the console
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TOPIC1);
    Serial.println(packetIdPub1);
    Serial.printf("Message: %.2f /n", sensors.getTempCByIndex(0));

    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_TOPIC2, 1, true, String(SensorValue2).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TOPIC2);
    Serial.println(packetIdPub2);
    Serial.printf("Message: %.2f /n", sensors.getTempCByIndex(1));

    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_TOPIC3, 1, true, String(SensorValue3).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TOPIC3);
    Serial.println(packetIdPub3);
    Serial.printf("Message: %.2f /n", sensors.getTempCByIndex(2));

    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_TOPIC4, 1, true, String(SensorValue4).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TOPIC4);
    Serial.println(packetIdPub4);
    Serial.printf("Message: %.2f /n", sensors.getTempCByIndex(3));

    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_TOPIC5, 1, true, String(SensorValue5).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TOPIC5);
    Serial.println(packetIdPub5);
    Serial.printf("Message: %.2f /n", sensors.getTempCByIndex(4));

    // check if SensoValue4 is > 48°C to determine a specific device status
    if(SensorValue4 > 48) {
      DevOn = 2;
      uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_TOPIC6, 1, true, String(DevOn).c_str());                            
      Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TOPIC6);
      Serial.println(packetIdPub6);
    } else {
      // check if SensorValue4 is > 35°C but < 48°C
      if((SensorValue4 > 35) && (SensorValue4 < 48)) {
        DevOn = 1;
        uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_TOPIC6, 1, true, String(DevOn).c_str());                            
        Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TOPIC6);
        Serial.println(packetIdPub6);
      }
    } 
    // if SensorValue4 is < 35°C --> Device must be inactive
    if(SensorValue4 <= 35) {
      DevOn = 0;
      uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_TOPIC6, 1, true, String(DevOn).c_str());                            
      Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TOPIC6);
      Serial.println(packetIdPub6);
    } 
    
  }
  
}
