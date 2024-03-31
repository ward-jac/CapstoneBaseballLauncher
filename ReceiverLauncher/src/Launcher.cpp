#include <Arduino.h>
#include <Servo.h>
#include "../include/Launcher.h"


#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0));
ServoTimer2 myServo;

// the maximum values of theta and phi
const int maxAngle = 30;
const int servoPin = 5;
int servoPos = 0;

const int speed3MinAngle = 30;
const int speed2MinAngle = 20;
const int speed1MinAngle = 10;
const int speed3 = 1;
const int speed2 = 100;
const int speed1 = 300;
int speed = 0;

// Arduino pins for linear actuator (can't use 10 with AltSoftSerial)
const int act_pin = A0;             // linear actuator potentiometer pin 54
const int act_RPWM = 11;            // linear actutator RPWM connection
const int act_LPWM = 12;            // linear actuator LWPM connection
const int act_RPWM = 11;            // linear actutator RPWM connection
const int act_LPWM = 12;            // linear actuator LWPM connection

// servo
Servo myServo;
int servoPos = 0;
const int speed3MinAngle = 30;
const int speed2MinAngle = 20;
const int speed1MinAngle = 10;
const int speed3 = 1;
const int speed2 = 100;
const int speed1 = 300;
int speed = 0;

// actuator
const int actReading = 0;
const int strokeLength = 8.0;

// for the relays
const int POWER = 2;
const int SPEED_UP = 3;
const int SPEED_DOWN = 4;
const int ENTER = 5;

Launcher::Launcher() {
  myServo.write(servoPos);
  myServo.attach(servoPin);
}

// maps input to output positions
float Launcher::mapFloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

// increments speed by incSpeed
float Launcher::addSpeed(int incSpeed) {
  if (incSpeed < 0) {
    addSpeed(incSpeed + 1);
    digitalWrite(SPEED_DOWN, LOW);
    delay(10);
    digitalWrite(SPEED_DOWN, HIGH);
  }
  else if (incSpeed > 0) {
    addSpeed(incSpeed - 1);
    digitalWrite(SPEED_UP, LOW);
    delay(10);
    digitalWrite(SPEED_UP, HIGH);
  }
}

// moves the linear actuator in a given direction at a given speed
void Launcher::driveActuator(int Direction, int Speed) {
  switch(Direction){
    case 1:  // extension
      analogWrite(act_RPWM, Speed);
      analogWrite(act_LPWM, 0);
      break;
   
    case 0:  // no movement
      analogWrite(act_RPWM, 0);
      analogWrite(act_LPWM, 0);
      break;

    case -1:  // retraction
      analogWrite(act_RPWM, 0);
      analogWrite(act_LPWM, Speed);
      break;
  }
}

// moves the linear actuator accordingly if the vertical (up and down) IMU angle surpasses the minimum angle
void Launcher::moveAct(float phi) {
    // obtain the analog potentiometer reading of the linear actuator
    actReading = analogRead(act_pin);

    // constrain phi within the max angles
    if (abs(phi) > maxAngle)
    {
        phi = phi * maxAngle;
    }

    // the difference between phi and the activation angle for a given sensitivity
    float diff = abs(phi) - sensitivity[sensitivityMode];

    // determine the actuator speed based on phi
    float actSpeed = mapFloat(abs(phi), 0, maxAngle, 0, 255);

    // extends the linear actuator if phi surpasses the minimum angle
    if (diff > 0)
    {
        phi = sgn(phi);
        driveActuator(phi, actSpeed);
    }
    // does not move the linear actuator if the minimum angle isn't surpassed
    else
    {
        actPos = actReading;
        driveActuator(0, actSpeed);
    }
}

void Launcher::updateServo(float theta) {
  if(abs(theta)>speed3MinAngle) {
    speed = speed3;
  }
  else if(abs(theta)>speed2MinAngle) {
    speed = speed2;
  }
  else if(abs(theta)>speed1MinAngle) {
    speed = speed1;
  }
  else {
    speed = 0;
  }
  
  if(speed!=0 && millis()%speed==0) {
      if(theta>0) {
        servoPos+=1;
      } else {
        servoPos-=1;
      }
      myServo.write(servoPos);
  }
}

void Launcher::setSensitivity(char c) {
  if(c == 'H')
  {
    sensitivityMode = 1;
  }
  else if(c == 'L')
  {
    sensitivityMode = 0;
  }
}