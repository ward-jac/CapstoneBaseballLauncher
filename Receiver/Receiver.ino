// required for servo operation
#include <ServoTimer2.h>

// for sign operations
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

//  Pins
//  BT VCC to Arduino 5V out.
//  BT GND to GND
//  Arduino 48 (Mega) RX -> BT TX no need voltage divider
//  Arduino 46 (Mega) TX -> BT RX through a voltage divider (5v to 3.3v)

// https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
#include <AltSoftSerial.h>
AltSoftSerial receiver;

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

// for the servo motor
ServoTimer2 myServo;
int servoPin = 5;
int servoZero = 90;
int prevServoPos = servoZero;

// the maximum values of theta and phi
int maxAngle = 30;

// to deal with gimbal lock
int dangerZone = 60;

// Arduino pins for linear actuator
int act_pin = A0;   // linear actuator potentiometer pin
int act_RPWM = 10;  // linear actutator RPWM connection
int act_LPWM = 11;  // linear actuator LWPM connection

// state variables for linear actuator
int actReading = 0;        // the value read by the linear actuator potentiometer
float actSpeed;            // speed of the linear actuator (value from 0-255)
float strokeLength = 8.0;  // length of linear actuator stroke
int maxAnalogReading = 1023;      // max value that linear actuator is allowed to move to
int minAnalogReading = 0;      // min value that linear actuator is allowed to move to
int actPos;                // the set position that the linear actuator is being moved to

// state variables for servo
float servoSpeed;  // the scaled rotation of the servo (value from 0-1)
float servoPos;    // the set position that the servo is being moved to
int microPos;      // the servo position in microseconds that it is moving to

// to keep track of the current firing mode
// 0 = fine
// 1 = coarse
int sensitivityMode = 1;

// the minimum angle needed to activate
int sensitivity[] = { 7, 15 };

// returns whether or not a given linear actuator movement is valid
bool validMovement(int dir, int pot) {
  return (dir == 0) || ((dir == 1) && (pot < maxAnalogReading)) || ((dir == -1) && (pot > minAnalogReading));
}

// maps input to output positions
float mapFloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

// moves the linear actuator in a given direction at a given speed
void driveActuator(int dir, int speed) {
  // update the current potentiometer reading
  actReading = analogRead(act_pin);

  // confirm that the actuator can move in the specified direction
  if (!validMovement(dir, actReading)) {
    return;
  }

  switch (dir) {
    case 1:  // extension
      analogWrite(act_RPWM, speed);
      analogWrite(act_LPWM, 0);
      break;

    case 0:  // no movement
      analogWrite(act_RPWM, 0);
      analogWrite(act_LPWM, 0);
      break;

    case -1:  // retraction
      analogWrite(act_RPWM, 0);
      analogWrite(act_LPWM, speed);
      break;
  }
}

// moves the linear actuator accordingly if the vertical (up and down) IMU angle surpasses the minimum angle
void moveAct(float phi) {
  // the absolute value of phi
  float a = abs(phi);

  // don't move the machine if the values are incorrect
  if (a > dangerZone) {
    phi = 0.0;
  }
  // otherwise constrain phi within the max angles
  else if (a > maxAngle) {
    phi = sgn(phi) * maxAngle;
  }

  // the difference between phi and the activation angle for a given sensitivity
  float diff = abs(phi) - sensitivity[sensitivityMode];

  // determine the actuator speed based on phi
  float actSpeed = mapFloat(abs(phi), 0, maxAngle, 0, 255);

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

  // don't move the machine if the values are incorrect
  if (a > dangerZone) {
    theta = 0.0;
  }
  // otherwise constrain theta within the max angles
  else if (a > maxAngle) {
    theta = sgn(theta) * maxAngle;
  }

  // the difference between theta and the activation angle for a given sensitivity
  float diff = abs(theta) - sensitivity[sensitivityMode];

  // before moving, obtain the last written position of the servo
  prevServoPos = myServo.read();

  // rotates the servo if theta surpasses the minimum angle
  if (diff > 0) {
    if (theta > 0) {
      myServo.write(prevServoPos - 12);
    } else if (theta < 0) {
      myServo.write(prevServoPos + 12);
    }
  }
}

// obtain the first number in the string (theta)
float stringToTheta(String str) {
  return str.substring(0, str.indexOf(" ")).toFloat();
}

// obtain the second number in the string (phi)
float stringToPhi(String str) {
  return str.substring(str.indexOf(" ") + 1, str.indexOf(etx)).toFloat();
}

// safely read and process a character from BT
char readSafe() {
  // the character to read
  char ch = -1;

  // read until we have a valid read
  while (!isValidChar(ch)) {
    // read while there is information to read
    while (receiver.available() > 0) {
      ch = char(receiver.read());
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

void setup() {
  // start serial monitor communication at 9600 baud rate
  Serial.begin(9600);
  Serial.print("Sketch:   ");
  Serial.println(__FILE__);
  Serial.print("Uploaded: ");
  Serial.println(__DATE__);
  Serial.println(" ");

  // start receiver BT at 9600 baud rate
  receiver.begin(9600);
  Serial.println("Receiver started at 9600");

  // zero the servo and attach it to the Arduino
  myServo.write(servoZero);
  myServo.attach(servoPin);

  // 1 second delay
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
    theta_angle = stringToTheta(data);
    phi_angle = stringToPhi(data);

    // move the servo and linear actuator if necessary
    moveServo(theta_angle);
    moveAct(phi_angle);

    // reset the data string
    data = "";

    for (int i = 0; i < data.length(); i++) {
      Serial.print(data.charAt(i));
    }

    Serial.println("");

    Serial.println("Theta: " + String(theta_angle));
    Serial.println("Phi: " + String(phi_angle));
  }
}