#include "mh_z14a.h"

#ifdef ESP32
HardwareSerial port(2); // RX2 16, TX2 17 on ESP32
#else
#include <"SoftwareSerial.h>
SoftwareSerial port(2, 3); //RX & TX
#endif



MH_Z14A sensor;

void setup() {
  Serial.begin(115200);
  sensor.begin(&port);
}

void loop() {
  uint32_t ppm;
  uint8_t err = sensor.read(&ppm);

  if(err) {
    Serial.print("Error code: ");
    Serial.println(err);
  }
  
    Serial.print("CO2 ppm: ");
    Serial.println(ppm);
  
  delay(5000);
}
