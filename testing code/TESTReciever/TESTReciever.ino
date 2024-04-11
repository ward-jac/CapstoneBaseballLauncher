// proximity sensor
#include "Adafruit_VCNL4010.h"
Adafruit_VCNL4010 vcnl;

void setup() {
  // start serial monitor communication at 9600 baud rate
  Serial.begin(9600);

  // start the proximity sensor
  vcnl.begin();

}

void loop() {
  int proximity = vcnl.readProximity();
  Serial.println(proximity);
  delay(100);
}