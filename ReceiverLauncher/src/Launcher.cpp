#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_VCNL4010.h/>
#include "../include/Launcher.h"

//pitch actuator pins
const int act_pin = A0;  // potentiometer pin 54
const int act_RPWM = 11;
const int act_LPWM = 12;
//servo pin
const int servoPin = 9;
//autoloader pins
int autoload_RPWM = 3;
int autoload_LPWM = 4;
//relay pins
const int POWER = 5;
const int SPEED_UP = 6;
const int SPEED_DOWN = 7;
const int ENTER = 8;

// servo
Servo myServo;
const int speed3MinAngle = 30;
const int speed2MinAngle = 20;
const int speed1MinAngle = 10;
const int speed3 = 1;
const int speed2 = 100;
const int speed1 = 300;
int speed = 0;
int servoPos = 0;

// actuator
const int actReading = 0;
const int strokeLength = 8.0;

// autoloader
Adafruit_VCNL4010 vcnl;
int autoload_forwardPWM = 0;  // initial values for forward autoloader direction
int autoload_reversePWM = 0;  // initial values for reverse autoloader direction
int proximity;

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

void Launcher::driveAutoLoad(){
  /*When this function is called, a ball will be loaded into the system. The DC motor in the autoloader
  will turn until the proximity sensor senses a ball. When the proximity sensor sesnses a ball this means that
  the ball has dropped into the pitching machine*/
  bool ball_sensed = false;
  proximity = vcnl.readProximity();   //the value of the proximity sensor is read
  while (ball_sensed == false){       //the DC motor will rotate until a ball has been sensed
    autoload_forwardPWM = 1023;
    proximity = vcnl.readProximity();
    if (proximity > 4000){            //4000 is a good value to determine when a ball has dropped in front of the sensor
      ball_sensed = true;
      autoload_forwardPWM = 0;        //the autoloader will stop spinning to stop another ball from dropping
    }
    analogWrite(autoload_LPWM, 0);
    analogWrite(autoload_RPWM, autoload_forwardPWM); //this will cause the DC motor to move forward
  }
  delay(15000); //once a ball is dropped in, all systems will be stopped by this delay. This allows time for the ball to be launched without aiming or loading another ball
}