// required for IMU calculations
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// required for servo operation
#include <Servo.h>

// for sign operations
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

// the sample rate delay and IMU
#define BNO055_SAMPLERATE_DELAY_MS (500)
Adafruit_BNO055 myIMU = Adafruit_BNO055(55);

// for the servo motor
Servo myServo;
int servoPin = 5;
int servoZero = 90;
int prevServoPos = servoZero;

// the curent angles at the time of calibration
float shift_theta = 0.0;
float shift_phi = 0.0;
int updateCount = 0;

// the maximum values of theta and phi
int maxAngle = 30;

// to deal with gimbal lock
int dangerZone = 60;

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

// returns the shifted value of theta after calibration
float getTheta(imu::Vector<3> euler) {

  float shifted = euler.x() + (-1.0 * shift_theta);

  // to bound the angle from -180 to 180 degrees
  if (shifted > 180.0) {
    return shifted - 360.0;
  }
  else if (shifted < -180.0) {
    return shifted + 360.0;
  }
  else {
    return shifted;
  }
}

// returns the shifted value of phi after calibration
float getPhi(imu::Vector<3> euler) {
  return euler.z() + (-1.0 * shift_phi);
}

// updates the euler shift angles (theta and phi)
void updateShifts() {
  
  // calibrate and obtain the euler angles from the IMU
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);

  // euler x = theta
  shift_theta = euler.x();

  // euler z = phi
  shift_phi = euler.z();
}

void setup(void) 
{
  // begin serial communication at a 9600 baud rate
  Serial.begin(9600);
    
  myIMU.begin();
  delay(1000);
  int8_t temp = myIMU.getTemp();
  myIMU.setExtCrystalUse(true);

  // zero the servo and attach it to the Arduino
  myServo.write(servoZero);
  myServo.attach(servoPin);
  
  // 1 second delay
  delay(1000);
}

void loop(void) 
{  
  // zero the IMU at the very beginning of the program
  if (updateCount < 1) {
    updateShifts();
    Serial.println("Calibrated\n");
    updateCount++;
  }

  // obtain the current euler angles of the imu
  imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("Theta: ");
  Serial.println(getTheta(euler));
  Serial.print("Phi: ");
  Serial.println(getPhi(euler));
  Serial.println("");

  // move the linear actuator if necessary
  moveAct(getPhi(euler));

  // move the servo if necessary
  moveServo(getTheta(euler));

  // delay for the designated sample rate delay
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
