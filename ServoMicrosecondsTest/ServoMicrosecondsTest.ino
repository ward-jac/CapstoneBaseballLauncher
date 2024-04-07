// required for servo operation
#include <Servo.h>

// for the servo motor
Servo myServo;
int servoPin = 5;

void setup() {
  // establish serial communication
  Serial.begin(9600);

  // write the initial position to the servo and attach it to the Arduino
  myServo.writeMicroseconds(1500);
  myServo.attach(servoPin);

  // 1 second delay
  delay(1000);
}

void loop() {
  // // minimum position reached at 553 microseconds
  // for (int i = 600; i > 0; i--) {
  //   myServo.writeMicroseconds(i);
  //   delay(1000);
  //   Serial.println(myServo.read());
  //   Serial.println(i);
  //   Serial.println("");
  // }

  // // maximum position reached at 2400 microseconds
  // for (int i = 2350; i < 2800; i++) {
  //   myServo.writeMicroseconds(i);
  //   delay(500);
  //   Serial.println(myServo.read());
  //   Serial.println(i);
  //   Serial.println("");
  // }
}