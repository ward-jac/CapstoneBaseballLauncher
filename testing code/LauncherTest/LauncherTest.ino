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

// for the servo motor
ServoTimer2 myServo;
int servoPin = 5;
int servoZero = 90;
int prevServoPos = servoZero;

// the maximum values of theta and phi
int maxAngle = 30;

// to deal with gimbal lock
int dangerZone = 60;

// IMU angles
float theta;
float phi;

// Arduino pins for linear actuator
int act_pin = A0;             // linear actuator potentiometer pin
int act_RPWM = 10;            // linear actutator RPWM connection
int act_LPWM = 11;            // linear actuator LWPM connection

// state variables for linear actuator
int actReading = 0;           // the value read by the linear actuator potentiometer
float actSpeed;               // speed of the linear actuator (value from 0-255)
float strokeLength = 8.0;     // length of linear actuator stroke
int maxAnalogReading;         // max value that linear actuator is allowed to move to
int minAnalogReading;         // min value that linear actuator is allowed to move to
int actPos;                   // the set position that the linear actuator is being moved to

// state variables for servo
float servoSpeed;             // the scaled rotation of the servo (value from 0-1)
float servoPos;               // the set position that the servo is being moved to
int microPos;                 // the servo position in microseconds that it is moving to

// to keep track of the current firing mode
// 0 = fine
// 1 = coarse
int sensitivityMode = 1;

char c = ' ';
boolean NL = true; 

// the minimum angle needed to activate
int sensitivity[] = {7, 15};

// maps input to output positions
float mapFloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

// moves the linear actuator in a given direction at a given speed
void driveActuator(int Direction, int Speed) {
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
void moveAct(float phi) {

  // obtain the analog potentiometer reading of the linear actuator
  actReading = analogRead(act_pin);

  // don't move the machine if the values are incorrect
  if (abs(phi) > dangerZone) {
    phi = 0;
  }

  // constrain phi within the max angles
  if (abs(phi) > maxAngle) {
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
    actPos = actReading;
    driveActuator(0, actSpeed);
  }
}

// moves the servo accordingly if the horizontal (left and right) IMU angle surpasses the minimum angle
void moveServo(float theta) {

  // don't move the machine if the values are incorrect
  if (abs(theta) > dangerZone) {
    theta = 0;
  }

  // constrain theta within the max angles
  if (abs(theta) > maxAngle) {
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
    }
    else if (theta < 0) {
      myServo.write(prevServoPos + 12);
    }
  }
}

// obtain the first number in the string (theta)
float stringToTheta(String str) {
  return str.substring(0, str.indexOf(" ")).toFloat();
}

// obtain the first number in the string (phi)
float stringToPhi(String str) {
  return str.substring(str.indexOf(" "), str.length()).toFloat();
}

void setup(void) 
{
  // start serial monitor communication at 9600 baud rate
  Serial.begin(9600);
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);
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

void loop(void) 
{  
  // read from receiver BT and print to serial monitor
  if (receiver.available() > 0)
  {
    String str = receiver.readString();
    theta = stringToTheta(str);
    phi = stringToPhi(str);
    Serial.print(theta);
    Serial.print(" ");
    Serial.println(phi);
  }
 
  // read from serial monitor and send to receiver BT
  if (Serial.available() > 0)
  {
    c = Serial.read();

    // do not send line end characters to the HM-10
    if (c != 10 & c != 13) 
    {  
      receiver.write(c);
    }
 
    // Echo the user input to the main window. 
    // If there is a new line print the ">" character.
    if (NL) 
    { 
      Serial.print("\r\n>");  
      NL = false; 
    }

    Serial.write(c);
    if (c == 10) 
    { 
      NL = true;
    }
  }
  
  // move the servo if necessary
  moveServo(theta);

  // move the linear actuator if necessary
  moveAct(phi);
}