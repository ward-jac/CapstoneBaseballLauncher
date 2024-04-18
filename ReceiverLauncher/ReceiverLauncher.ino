// for BT communication
#define BTSerial Serial1

// required for servo operation
#include <Servo.h>

// proximity sensor
#include "Adafruit_VCNL4010.h"
Adafruit_VCNL4010 vcnl;

// for sign operations
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

// start of text char
char stx = 2;

// end of text char
char etx = 3;

// to store the received data
String data = "";

// char to store read data
char c = ' ';

// IMU angles
float theta_angle = 0.0;
float phi_angle = 0.0;

// the maximum values of theta and phi
int maxAngle = 30;

// for the servo motor
Servo myServo;
int servoPin = 31;
int servoZeroDeg = 90;                                                         // zeroed position of the servo, in degrees
int servoLowBoundDeg = servoZeroDeg - 30;                                      // minimum position of the servo, in degrees
int servoMinMicro = 553;                                                       // minimum position of the servo, in microseconds
int servoHighBoundDeg = servoZeroDeg + 30;                                     // maximum position of the servo, in degrees
int servoMaxMicro = 2400;                                                      // maximum position of the servo, in microseconds
int prevServoMicro = map(servoZeroDeg, 0, 180, servoMinMicro, servoMaxMicro);  // the previous servo position, in microseconds
int servoSpeed;                                                                // the scaled rotation speed of the servo
int servoPos;                                                                  // the set position that the servo is being moved to

// for the proximity sensor
int proximity = 0;
int proxThreshold = 4000;

// pins for autoloader
int autoload_RPWM = 10;
int autoload_LPWM = 9;

// pins for linear actuator
int act_pin = A0;  // linear actuator potentiometer pin
int act_RPWM = 3;  // linear actutator RPWM connection
int act_LPWM = 4;  // linear actuator LWPM connection

// state variables for linear actuator
int actReading = 0;          // the value read by the linear actuator potentiometer
float actSpeed;              // speed of the linear actuator (value from 0-255)
int maxAnalogReading = 940;  // max value that linear actuator is allowed to move to
int minAnalogReading = 45;   // min value that linear actuator is allowed to move to

// the minimum angle needed to activate
int sensitivity = 7;

// to keep track of the switch info to send to the launcher
int speedInfo;
int fireInfo;
int powerInfo;
boolean powerOn = false;

// to change the launcher speed with a relay
const int power = 8;
const int speedUp = 7;
const int speedDown = 6;
const int enter = 5;

// the minimum time between each fire
long fireCooldown = 5000;

// the time when the launcher was fired last
long lastFireTime = 0;

// returns whether or not a given linear actuator movement is valid
bool validActMove(int dir, int pot) {
  return (dir == 0) || ((dir == 1) && (pot < maxAnalogReading)) || ((dir == -1) && (pot > minAnalogReading));
}

// returns whether or not a given servo movement is valid
bool validServoMove(int dir, int servoPosMicro) {
  return (dir == 0) || ((dir == 1) && (servoPosMicro < map(servoHighBoundDeg, 0, 180, servoMinMicro, servoMaxMicro)))
         || ((dir == -1) && (servoPosMicro > map(servoLowBoundDeg, 0, 180, servoMinMicro, servoMaxMicro)));
}

// maps input to output positions
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// moves the linear actuator in a given direction at a given speed
void driveActuator(int dir, float Speed) {
  // update the current potentiometer reading
  actReading = analogRead(act_pin);

  // confirm that the actuator can move in the specified direction
  if (!validActMove(dir, actReading)) {
    return;
  }

  switch (dir) {
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
void moveAct(float phi) {
  // the absolute value of phi
  float a = abs(phi);

  // constrain phi within the max angles
  if (a > maxAngle) {
    phi = sgn(phi) * maxAngle;
  }

  // the difference between phi and the activation angle for the given sensitivity
  float diff = abs(phi) - sensitivity;

  // determine the actuator speed based on phi
  float actSpeed = mapFloat(abs(phi), 0.0, maxAngle, 0.0, 175.0);

  // extends the linear actuator if phi surpasses the minimum angle
  if (diff > 0) {
    driveActuator(sgn(phi), actSpeed);
  }
  // does not move the linear actuator if the minimum angle isn't surpassed
  else {
    driveActuator(0, actSpeed);
  }
}

// moves the servo accordingly if the horizontal (left and right) IMU angle surpasses the minimum angle
void moveServo(float theta) {
  // absolute value of theta
  float a = abs(theta);

  // constrain theta within the max angles
  if (a > maxAngle) {
    theta = sgn(theta) * maxAngle;
  }

  // the difference between theta and the activation angle for a given sensitivity
  float diff = abs(theta) - sensitivity;

  // determine the servo speed based on theta
  servoSpeed = map(diff, 0.0, (maxAngle - sensitivity), 0.0, 10.0);

  // before moving, obtain the last written position of the servo
  prevServoMicro = map(myServo.read(), 0, 180, servoMinMicro, servoMaxMicro);

  // rotates the servo if theta surpasses the minimum angle
  if (diff > 0) {
    if (theta > 0) {
      // confirm that the servo can rotate right
      if (!validServoMove(1, prevServoMicro)) {
        return;
      }
      myServo.writeMicroseconds(prevServoMicro + servoSpeed);
    } else if (theta < 0) {
      // confirm that the servo can rotate left
      if (!validServoMove(-1, prevServoMicro)) {
        return;
      }
      myServo.writeMicroseconds(prevServoMicro - servoSpeed);
    }
  } else {
    myServo.writeMicroseconds(prevServoMicro);
  }
}

// obtain the first number in the string (theta)
float dataToTheta(String str) {
  return str.substring(0, str.indexOf(" ")).toFloat();
}

// obtain the second number in the string (phi)
float dataToPhi(String str) {
  int space1 = str.indexOf(" ");
  int space2 = (str.substring(space1 + 1, str.indexOf(etx))).indexOf(" ") + space1 + 1;
  return str.substring(space1 + 1, space2).toFloat();
}

// convert a string to an integer for speed info (positive or negative)
int stringToSpeedInfo(String s) {
  // the only character is the speed (positive)
  if (s.length() == 1) {
    return s.toInt();
  }
  // otherwise, the number is negative
  else {
    return -1 * s.toInt();
  }
}

// update the state variables for speed change, fire, and power
void updateStateVars(String str) {
  int space1 = str.indexOf(" ");
  int space2 = (str.substring(space1 + 1, str.indexOf(etx))).indexOf(" ") + space1 + 1;
  int space3 = (str.substring(space2 + 1, str.indexOf(etx))).indexOf(" ") + space2 + 1;
  int space4 = (str.substring(space3 + 1, str.indexOf(etx))).indexOf(" ") + space3 + 1;

  speedInfo = str.substring(space2 + 1, space3).toInt();
  fireInfo = str.substring(space3 + 1, space4).toInt();
  powerInfo = str.substring(space4 + 1, str.indexOf(etx)).toInt();
}

// safely read and process a character from BT
char readSafe() {
  // the character to read
  char ch = -1;

  // read until we have a valid read
  while (!isValidChar(ch)) {
    // read while there is information to read
    while (BTSerial.available() > 0) {
      ch = char(BTSerial.read());
    }
  }

  // return the valid char
  return ch;
}

// returns whether or not the given char is valid
bool isValidChar(char ch) {
  // a valid char is one of [stx (2), etx (3), space (32), minus (45), period (46), number (48-57)]
  char validChars[15] = { 2, 3, 32, 45, 46, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57 };

  // assume the character is invalid
  bool isValid = false;

  // check the char against the list of valid chars
  for (int i = 0; i < 15; i++) {
    isValid = isValid || (ch == validChars[i]);
  }

  return isValid;
}


// holds relay to increment speed
void changeSpeed(int speedInc) {
  int s;
  if (speedInc > 0) {
    s = speedUp;
  } else {
    s = speedDown;
  }
  for (int i = 0; i < abs(speedInc); i++) {
    digitalWrite(s, LOW);
    // delay 1 second for a speed increment of 5
    delay(1000);
    digitalWrite(s, HIGH);
    delay(50);
  }
}

// loads a ball into the system, the DC motor will turn until the proximity sensor senses a ball
void driveAutoloader() {
  /*
  // no ball is present initially
  bool sensed = false;

  // start time of autoloader
  long start = millis();

  // run the DC motor until a ball has been sensed for a max of 5 seconds
  while ((!sensed) && (millis() - start <= 5000)) {
    // read the proximity sensor
    proximity = vcnl.readProximity();
    Serial.println(proximity);

    // check if a ball has been sensed
    if (proximity > 4000) {
      sensed = true;
    }

    // drive the autoloader
    analogWrite(autoload_LPWM, 0);
    analogWrite(autoload_RPWM, 1023);
    delay(100);
  }
  analogWrite(autoload_LPWM, 0);
  analogWrite(autoload_RPWM, 0);
  */

  analogWrite(autoload_LPWM, 0);
  analogWrite(autoload_RPWM, 1023);
  delay(2000);
  analogWrite(autoload_LPWM, 0);
  analogWrite(autoload_RPWM, 0);
}

void togglePower() {
  delay(1000);
  digitalWrite(power, LOW);
  delay(2000);
  digitalWrite(power, HIGH);
  delay(200);

  powerOn = !powerOn;
}

// start up procedure to unlock launcher with access code 1919
void startUpProcedure() {
  togglePower();

  for (int i = 0; i <= 1; i++) {
    digitalWrite(speedUp, LOW);
    delay(200);
    digitalWrite(speedUp, HIGH);
    delay(200);

    digitalWrite(speedDown, LOW);
    delay(200);
    digitalWrite(speedDown, HIGH);
    delay(200);

    digitalWrite(enter, LOW);
    delay(200);
    digitalWrite(enter, HIGH);
    delay(200);
  }
}

void setup() {
  // start serial monitor communication at 9600 baud rate
  Serial.begin(9600);
  Serial.print("Sketch:   ");
  Serial.println(__FILE__);
  Serial.print("Uploaded: ");
  Serial.println(__DATE__);
  Serial.println(" ");

  // start the proximity sensor
  vcnl.begin();

  // start receiver BT at 9600 baud rate
  BTSerial.begin(9600);
  Serial.println("Receiver started at 9600");

  // establish the linear actuator
  pinMode(act_pin, INPUT);
  pinMode(act_RPWM, OUTPUT);
  pinMode(act_LPWM, OUTPUT);

  // establish the autoloader
  pinMode(autoload_RPWM, OUTPUT);
  pinMode(autoload_LPWM, OUTPUT);

  // establish the relay for speed changes and make sure both are initially off
  pinMode(power, OUTPUT);
  pinMode(speedUp, OUTPUT);
  pinMode(speedDown, OUTPUT);
  pinMode(enter, OUTPUT);
  digitalWrite(power, HIGH);
  digitalWrite(speedUp, HIGH);
  digitalWrite(speedDown, HIGH);
  digitalWrite(enter, HIGH);

  // zero the servo and attach it to the Arduino
  myServo.writeMicroseconds(map(servoZeroDeg, 0, 180, servoMinMicro, servoMaxMicro));
  myServo.attach(servoPin);

  // for powering on
  delay(1000);
  startUpProcedure();
  delay(1000);
}

void loop() {
  // read until we reach the beginning of the data string
  while (c != stx) {
    c = readSafe();
  }

  // read the next char (the first char of interest in the data string)
  c = readSafe();

  // continue reading and constructing the data string until etx or no more chars to read
  while (c != etx) {
    // append the most recent char to the data string
    data += String(c);

    // read the next char
    c = readSafe();
  }

  // check to see if the entire string was received (last char is etx)
  if (c == etx) {
    // update theta and phi and reset the data string
    theta_angle = dataToTheta(data);
    phi_angle = dataToPhi(data);

    // update the state variables for speed change, fire, and power
    updateStateVars(data);

    // move the servo and linear actuator
    moveServo(theta_angle);
    moveAct(phi_angle);

    // fire if we can
    if ((millis() - lastFireTime) > fireCooldown) {
      if (fireInfo) {
        driveAutoloader();
        lastFireTime = millis();
      }
    }

    // change speed if necessary
    if (speedInfo != 0) {
      changeSpeed(speedInfo);
    }

    // toggle power if necessary
    if (powerInfo) {
      if (powerOn) {
        togglePower();
      } else {
        startUpProcedure();
      }
    }

    data = "";
  }
}