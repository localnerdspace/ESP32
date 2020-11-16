# ESP32
Codes for the ESP32

Please keep in mind:
All my .ino files contain a MyConfig.h. This file is e.g. my local WiFi and MQTT config.
If you don´t want to use it, please comment the #include "MyConfig.h" and uncomment the respective variable in the code file.

## Installation

* browse to your local **/Arduino/libraries** folder
* create a new directory **/MyConfig**
* create a new textfile and save it there as **MyConfig.h**

### MyConfig.h
```
// This configuration file contains the bare minimum connection information
//Replace the next variables with your SSID/Password combination
#define WIFI_SSID "YourSSID"
#define WIFI_PASSWORD "YourWifiPassword"

// Add your MQTT Broker IP address
#define MQTT_HOST IPAddress(XXX, XXX, XXX, XXX) // example: (192, 168, 0, 1)
#define MQTT_PORT 1883
```
