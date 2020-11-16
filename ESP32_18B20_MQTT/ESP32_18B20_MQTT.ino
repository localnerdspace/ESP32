/*
 * Original code from randomnerdtutorials.com
 * Adopted to my needs. (localnerd.de)
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>

// make sure you include the MyConfig file into your /Arduino/libraries/MyConfig folder
// If you don´t want to use your own configuration file, comment this one and use the variables below
#include <MyConfig.h>

// Replace the next variables with your SSID/Password combination
//#define WIFI_SSID "<WLANSSID>"
//#define WIFI_PASSWORD "<PASSWORD>"

// Add your MQTT Broker IP address; may also included in your MyConfig.h
//#define MQTT_HOST IPAddress(xxx, xxx, xxx, xxx)
//#define MQTT_PORT 1883

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */
#define MQTT_PUBLISH "esp32/heating/tempdevout"

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;     

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  // Start the Serial Monitor
  Serial.begin(115200);
  // Start the DS18B20 sensor
  sensors.begin();

  setup_wifi();
  client.setServer(MQTT_HOST, MQTT_PORT);
  //client.setCallback(callback);
}


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      //client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void loop() {
if (!client.connected()) {
    reconnect();
  }
  client.loop();
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  char tempString[8];
  dtostrf(temperatureC, 1, 2, tempString);
  Serial.print("Temperature: ");
  Serial.println(tempString);
  client.publish(MQTT_PUBLISH, tempString);
  Serial.println("Going to sleep for 60 sec now");
  delay(1000);
//  Serial.flush(); 
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}
