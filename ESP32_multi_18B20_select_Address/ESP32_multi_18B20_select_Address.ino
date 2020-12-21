#include <OneWire.h>
OneWire  sensor(4);  
byte i;
byte type_s;
byte data[12];
byte Address[8];
  
void getAddress(){
  Serial.println();
  if ( !sensor.search(Address)) {
    
    Serial.println("No or no more sensor(s) attached to D4");
    Serial.println();
    sensor.reset_search();
    return;
  }
  
  Serial.print("Address = ");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print("0x");
    Serial.print(Address[i], HEX);
    if ( i != 7 ) {
      Serial.print(", ");
    }
  }

}

void setup() {
   Serial.begin(115200);
}

void loop() {
//call the getAddress function and wait 2.5 seconds to repeat
  getAddress();
  delay(2500);
}
