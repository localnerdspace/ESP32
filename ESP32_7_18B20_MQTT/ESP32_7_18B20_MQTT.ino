 /*
 * Original by randomnerdtutorials.com 
 * Adjusted to add more sensors and more mqtt publishers
 * The heating device is a ROTEX/Daikin hpsu compact 516 with solar panels and an 
 * outside converter device
 * This code works fine with DS18B20 temperature sensors attached to GPIO 4
 * For more details see https://localnerd.de/?p=53
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
// make sure you include the MyConfig file into your /Arduino/libraries/MyConfig folder
#include <MyConfig.h>

// Temperature MQTT Topic
#define MQTT_PUB_SOLAR "esp32/heating/solar" 
#define MQTT_PUB_DEVOUT "esp32/heating/tempdevout" 
#define MQTT_PUB_WARMWATER "esp32/heating/warmwater" 
#define MQTT_PUB_WWFROMHEAT "esp32/heating/wwfromheat" 
#define MQTT_PUB_WWFROMREFLOW "esp32/heating/wwfromreflow" 
#define MQTT_PUB_FH_FLOW "esp32/heating/fhflow" 
#define MQTT_PUB_FH_RETURN "esp32/heating/fhreturn" 
#define MQTT_PUB_DEVON "esp32/heating/devon" //symbolize the outside device is running
#define MQTT_PUB_VOLTS "esp32/temperature/voltage"

#define VOLTAGE_PIN 13 //Analog pin to read the voltage
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */


// GPIO where the DS18B20 are connected to
const int oneWireBus = 4;
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
DeviceAddress sensor1 = { 0x28,  0xBE,  0xE5,  0x56,  0xB5,  0x1,  0x3C,  0x83 }; //WW FLOW
DeviceAddress sensor2 = { 0x28,  0x5D,  0x30,  0x56,  0xB5,  0x1,  0x3C,  0x7F }; //SOLAR
DeviceAddress sensor3 = { 0x28,  0xB8,  0x1,  0x56,  0xB5,  0x1,  0x3C,  0xF4 }; //HEATING REFLOW
DeviceAddress sensor4 = { 0x28,  0x28,  0x10,  0x56,  0xB5,  0x1,  0x3C,  0xF4 }; //DEVOUT
DeviceAddress sensor5 = { 0x28,  0xF8,  0xDC,  0x56,  0xB5,  0x1,  0x3C,  0x33 }; //HEATING FLOW
DeviceAddress sensor6 = { 0x28,  0xCE,  0x14,  0x5F,  0x5,  0x0,  0x0,  0x86 }; //WW REFLOW
DeviceAddress sensor7 = { 0x28,  0x8A,  0x74,  0x5F,  0x5,  0x0,  0x0,  0x3E }; //WW FROM HEATING

float TempSolar; //sensor on solar return pipe
float TempDevOut; //sensor on heating device outside pipe
float TempWarmWater; //sensor on warm water pipe
float TempWWFromHeating; //sensor on warm water pipe from heating
float TempWWFromReflow; //sensor on warm water pipe from circulation reflow
float TempFloorHeatingFlow; //sensor on Floor Heating flow pipe
float TempFloorHeatingReturn; //sensor on Floor Heating return pipe
int DevOn; //signal integer for visualization of device status
int MQTT_Volts; //Voltage from the battery to pin 13 for analogRead

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 5000;        // Interval at which to publish sensor readings

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

void getTemperature(DeviceAddress deviceAddress) {
float tempC = sensors.getTempC(deviceAddress);
/*if (tempC == -127.00)
   {
   Serial.print(tempC);
   }
   else
   {
   Serial.print(tempC);
   //Serial.print(" °C");
   }*/
  // return(tempC);
  
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
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
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
    /*
    //error handling... Sensor presents sometimes 85.00°C and 25.00 after power on
    /*if (sensors.getTempCByIndex(0) == 85.00) { 
      // in case of 85.00 read sensors again
      sensors.requestTemperatures();
    }
    if (sensors.getTempCByIndex(0) == 25.00) { 
      // in case of 25.00 read sensors again
      sensors.requestTemperatures();
    }  */   
    // Temperature in Celsius degrees
    // array of sensors begin with index=0
    TempDevOut = sensors.getTempC(sensor4);
    TempFloorHeatingReturn = sensors.getTempC(sensor3);
    TempFloorHeatingFlow = sensors.getTempC(sensor5);
    TempWWFromHeating = sensors.getTempC(sensor7);
    TempWWFromReflow = sensors.getTempC(sensor6);
    TempWarmWater = sensors.getTempC(sensor1);
    TempSolar = sensors.getTempC(sensor2);

    // Publish a MQTT message to topic MQTT_PUB_SOLAR
    //conversion from float to string to publish correctly to MQTT
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_SOLAR, 1, true, String(TempSolar).c_str());
    //for better debugging print everything to the console
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_SOLAR);
    Serial.println(packetIdPub1);
    Serial.printf("Message: %.2f /n", TempSolar);

    // Publish a MQTT message to topic MQTT_PUB_DEVOUT
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_DEVOUT, 1, true, String(TempDevOut).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_DEVOUT);
    Serial.println(packetIdPub2);
    Serial.printf("Message: %.2f /n", TempDevOut);

    // Publish a MQTT message to topic MQTT_PUB_WARMWATER
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_WARMWATER, 1, true, String(TempWarmWater).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_WARMWATER);
    Serial.println(packetIdPub3);
    Serial.printf("Message: %.2f /n", TempWarmWater);

    // Publish a MQTT message to topic MQTT_PUB_WWFROMHEAT
    uint16_t packetIdPub8 = mqttClient.publish(MQTT_PUB_WWFROMHEAT, 1, true, String(TempWWFromHeating).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_WWFROMHEAT);
    Serial.println(packetIdPub8);
    Serial.printf("Message: %.2f /n", TempWWFromHeating);

    // Publish a MQTT message to topic MQTT_PUB_WWFROMREFLOW
    uint16_t packetIdPub9 = mqttClient.publish(MQTT_PUB_WWFROMREFLOW, 1, true, String(TempWWFromReflow).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_WWFROMREFLOW);
    Serial.println(packetIdPub9);
    Serial.printf("Message: %.2f /n", TempWWFromReflow);

    // Publish a MQTT message to topic MQTT_PUB_FH_FLOW
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_FH_FLOW, 1, true, String(TempFloorHeatingFlow).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_FH_FLOW);
    Serial.println(packetIdPub4);
    Serial.printf("Message: %.2f /n", TempFloorHeatingFlow);

    // Publish a MQTT message to topic MQTT_PUB_FH_RETURN
    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_FH_RETURN, 1, true, String(TempFloorHeatingReturn).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_FH_RETURN);
    Serial.println(packetIdPub5);
    Serial.printf("Message: %.2f /n", TempFloorHeatingReturn);

    // check if TempDevOut is > 48°C --> Device must be in compression mode --> present 2
    if(TempDevOut > 48) {
      DevOn = 2;
      uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_DEVON, 1, true, String(DevOn).c_str());                            
      Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_DEVON);
      Serial.println(packetIdPub6);
    } else {
      // check if TepDevOut is > 30°C but < 48°C --> Device must be in normal active mode --> present 1
      if((TempDevOut > 35) && (TempDevOut < 48)) {
        DevOn = 1;
        uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_DEVON, 1, true, String(DevOn).c_str());                            
        Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_DEVON);
        Serial.println(packetIdPub6);
      }
    } 
    if(TempDevOut <= 35) {
      DevOn = 0;
      uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_DEVON, 1, true, String(DevOn).c_str());                            
      Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_DEVON);
      Serial.println(packetIdPub6);
    } // if TempDevOut is < 30°C --> Device must be inactive

  // Publish a MQTT message to topic MQTT_PUB_VOLTS
    MQTT_Volts = analogRead(VOLTAGE_PIN);
    uint16_t packetIdPub7 = mqttClient.publish(MQTT_PUB_VOLTS, 1, true, String(MQTT_Volts).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_VOLTS);
    Serial.println(packetIdPub7);
    Serial.printf("Message: %.2f /n", MQTT_Volts);
    delay(500);
    Serial.println("Going down for DeepSleep now.");
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }
  
}
