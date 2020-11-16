#include <OneWire.h>
// OneWire bus connected on Pin 4
// a 4.7K resistor is necessary between +5V and Data wire
OneWire  ds(4);  

byte i;
byte type_s;
byte data[12];
byte addr[8];
  
void OneWireScanner(){
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    return;
  }
  
  Serial.print("ROM = ");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print("0x");
    Serial.print(addr[i], HEX);
    if ( i != 7 ) {
      Serial.print(", ");
    }
  }
  
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 
}

void setup() {
   Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  OneWireScanner();
  delay(5000);
}
