// required for servo operation
#include <Servo.h>

// the servo motor
Servo myServo;

int servoPin = 5;

void setup() {
  // put your setup code here, to run once:
  //myServo.write(90);
  myServo.attach(servoPin);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(myServo.read());
  myServo.write(0);
  delay(6000);
  Serial.println(myServo.read());
  myServo.write(180);
  delay(6000);
}