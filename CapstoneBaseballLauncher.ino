#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include "Adafruit_VCNL4010.h"
#include <Servo.h>
#include <elapsedMillis.h>
elapsedMillis timeElapsed;

int actuator_RPWM = 10;       // linear actutator RPWM connection  
int actuator_LPWM = 11;       // linear actuator LWPM connection
int sensorPin = A0;           // linear actuator potentiometer pin
int sensorVal;                // value being read by the linear actuator potentiometer
int ActuatorSpeed = 120;      // speed of the linear actuator (value from 0-255)
float strokeLength = 8.0;     // length of linear actuator stroke
int maxAnalogReading;         // max value that linear actuator is allowed to move to
int minAnalogReading;         // min value that linear actuator is allowed to move to

int Proximity;
int mode_pin = 40;            // microlight switch connection that switches modes
int autoload_pin = 32;        // microlight switch connection that loads ball
int autoload_RPWM = 3;        // autoloader RPWM connection
int autoload_LPWM = 4;        // autoloader LWPM connection
int autoload_forwardPWM = 0;  // initial values for forward autoloader direction
int autoload_reversePWM = 0;  // initial values for reverse autoloader direction

int mode = 1;                          // which aiming mode the code is in
bool mode_switch_pressed = false;      // if the mode button has been pressed (originally unpressed)
bool autoload_switch_pressed = false;  // if the autoload button has been pressed (originally unpressed)

/* values for the IMU, each corresponds to the angle the IMU is pointed and 
   equations are used to turn this value into an understandable angle */
float thetaM;
float phiM;
float thetaFold = 0;
float thetaFnew;
float phiFold = 0;
float phiFnew;
float phi_sum = 0;
float theta_sum = 0;
float phi_calibrated_zero = 0;    // gets the zeroed head position of the user
float theta_calibrated_zero = 0;  // gets the zeroed head position of the user
int calibration_cycles = 0; 

float posChange;     // the change in the servo position
int actuatorChange;  // the change in the actuator position
float ServoPos;      // the set position that the servo is being moved to
int ActuatorPos;     // the set position that the linear actuator is being moved to
int microPos;        // the servo position in microseconds that it is moving to

unsigned long start_mode_pressed = 0;
unsigned long end_mode_pressed = 0;
unsigned long time_mode_pressed = 0;
unsigned long calibration_time = 3000;     // the time that the IMU is being calibrated for
unsigned long start_calibrate = 0;
unsigned long start_autoload_pressed = 0;
unsigned long end_autoload_pressed = 0;
unsigned long time_autoload_pressed = 0;
unsigned long remove_ball_time = 3000;     // the time the autoload button must be pressed to remove the ball

Servo myServo;

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 myIMU = Adafruit_BNO055();
Adafruit_VCNL4010 vcnl;

/* Takes inputted positions and maps them to outputted poistions (allows float conversion) */
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

/* Moves the linear actuator based on a given direction:
    -1 : retract
     0 : no movement
     1 : extend
   Moves in the given direction at the given speed */
void driveActuator(int Direction, int Speed) {
  switch(Direction){
    case 1:  // extension
      analogWrite(actuator_RPWM, Speed);
      analogWrite(actuator_LPWM, 0);
      break;
   
    case 0:  // no movement
      analogWrite(actuator_RPWM, 0);
      analogWrite(actuator_LPWM, 0);
      break;

    case -1:  // retraction
      analogWrite(actuator_RPWM, 0);
      analogWrite(actuator_LPWM, Speed);
      break;

    default:  // invalid input
      Serial.println("Error: " + Direction + " is not a valid direction for Drive Actuation");
      break;
  }
}

/* Moves the linear actuator accordingly if the vertical (up and down) IMU angle surpasses the minimum angle */
void MoveActuator(float imu_angle, int moveangle_min, int moveangle_max, int mapmin, int mapmax, int posmin, int posmax) {

  // obtain the actual value of the linear actuator
  sensorVal = analogRead(sensorPin);

  // extends the linear actuator if the IMU angle surpasses the positive value of the minimum angle
  if (imu_angle >= phi_calibrated_zero + moveangle_min) { 
    actuatorChange = -map(imu_angle, phi_calibrated_zero + moveangle_min, phi_calibrated_zero + moveangle_max, mapmin, mapmax);
  }
  // retracts the linear actuator if the IMU angle surpasses the negative value of the minimum angle
  else if (imu_angle <= -moveangle_min) { 
    actuatorChange = -map(imu_angle, phi_calibrated_zero - moveangle_max, phi_calibrated_zero - moveangle_min, -mapmax, -mapmin);
  }
  // does not move the linear actuator if the minimum angle isn't surpassed
  else { 
    actuatorChange = 0;
    ActuatorPos = sensorVal;
  }

  // extends/retracts the linear actuator if it will not reach the max/min values
  if (ActuatorPos + actuatorChange <= posmax and ActuatorPos + actuatorChange >= posmin) { 
    ActuatorPos += actuatorChange;
  }
  // moves the linear actuator to the maximum position
  else if(ActuatorPos + actuatorChange >= posmax){
    ActuatorPos = posmax;
  }
  // moves the linear actuator to the minimum position
  else if(ActuatorPos + actuatorChange <= posmin){ 
    ActuatorPos = posmin;
  }
  
  // addition gives buffer to prevent actuator from rapidly vibrating due to noisy data inputs
  if (ActuatorPos > (sensorVal + 5)) { 
    driveActuator(1, ActuatorSpeed);
  }
  else if (ActuatorPos < (sensorVal - 5)) {             
    driveActuator(-1, ActuatorSpeed);
  }
  else {
    driveActuator(0, ActuatorSpeed);
  }
}

/* Moves the servo accordingly if the horizontal (left and right) IMU angle surpasses the minimum angle */
void MoveServo(float imu_angle, int moveangle_min, int moveangle_max, int mapmin, int mapmax, int maxangle) {

  int maxmicrosec round(1472 + (maxangle * 10/3));  // conversion from deg to microsec with 0 deg = 1472 microsec
  int minmicrosec round(1472 - (maxangle * 10/3));  // conversion from deg to microsec with 0 deg = 1472 microsec

  // rotates the servo if the IMU angle surpasses the positive value of the minimum angle
  if (imu_angle >= theta_calibrated_zero + moveangle_min) { 
    posChange = mapfloat(imu_angle, theta_calibrated_zero + moveangle_min, theta_calibrated_zero + moveangle_max, mapmin, mapmax);
  }
  // rotates the servo if the IMU angle surpasses the negative value of the minimum angle
  else if (imu_angle <= theta_calibrated_zero - moveangle_min) { 
    posChange = mapfloat(imu_angle, theta_calibrated_zero - moveangle_max, theta_calibrated_zero - moveangle_min, -mapmax, -mapmin);
  }
  // does not rotate the servo if the IMU angle does not surpass the minimum angle
  else { 
    posChange = 0;
  }

  // rotates the servo if it will not reach the max/min values
  if (ServoPos + posChange <= maxangle and ServoPos + posChange >= -maxangle) { 
    ServoPos += posChange;
  }
  // rotates the servo to the maximum position
  else if (ServoPos + posChange >= maxangle) {
    ServoPos = maxangle;
  }
  // rotates the servo to the maximum position
  else if (ServoPos + posChange <= -maxangle) {
    ServoPos = -maxangle;
  }

  // the servo position (in degrees) is mapped to microseconds to gain more resolution
  microPos = map(ServoPos, -maxangle, maxangle, minmicrosec, maxmicrosec); 
  myServo.writeMicroseconds(microPos);
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void DriveAutoload(){
  /*When this function is called, a ball will be loaded into the system. The DC motor in the autoloader
  will turn until the proximity sensor senses a ball. When the proximity sesnor sesnses a ball this means that
  the ball has dropped into the pitching machine*/
  bool ball_sensed = false;
  Proximity = vcnl.readProximity(); //the value of the proximity sensor is read
  while (ball_sensed == false){//the DC motor will rotate until a ball has been sensed
    autoload_forwardPWM = 1023;
    Proximity = vcnl.readProximity();
    if (Proximity > 4000){ //4000 is a good value to determine when a ball has dropped in front of the sesnro
      ball_sensed = true;
      autoload_forwardPWM = 0; //the autoloader will stop spinning to stop another ball from dropping
    }
    analogWrite(autoload_LPWM, 0);
    analogWrite(autoload_RPWM, autoload_forwardPWM); //this will cause the DC motor to move forward
  }
  delay(15000); //once a ball is dropped in, all systems will be stopped by this delay. This allows time for the ball to be launched without aiming or loading another ball
}

float get_theta(){
  /* gets theta which is the left and right tilt of the IMU 
  Equation was found online as a good way to convert the IMU values to an angle*/
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  thetaM = -atan2(acc.x()/9.8, acc.z()/9.8)*180/3.14159;
  thetaFnew = 0.85*thetaFold + 0.15*thetaM;
  return thetaFnew;
}

float get_phi(){
  /* gets phi which is the up and down tilt of the IMU 
  Equation was found online as a good way to convert the IMU values to an angle*/
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  phiM = atan2(acc.y()/9.8, acc.z()/9.8)*180/3.14159;
  phiFnew = 0.85*phiFold + 0.15*phiM;
  return phiFnew;
}

int mode_button_func(){
  /* This function checks if the mode button had been pressed. If it is quickly pressed, the mode will switch. 
   *  If its been held for at least 3 seconds then calibration mode is entered
   */
  if (((millis() - start_mode_pressed) > calibration_time) and mode_switch_pressed == true){ //checks if the button has been pressed for at least 3 seconds
      Serial.println("Calibrating");
      start_calibrate = millis(); //takes the time the calibration started
      phi_sum = 0;
      theta_sum = 0;
      calibration_cycles = 0;
      while (millis() - start_calibrate < 3000){//continues calibrating for 3 seconds
        phi_sum += get_phi(); //continues to add values for phi
        theta_sum += get_theta(); //continues to add values for theta
        calibration_cycles += 1; //adds to the number of times the calibration cycle has gones through
      }
      phi_calibrated_zero = phi_sum/calibration_cycles; //gets the newly calibrated phi value
      theta_calibrated_zero = theta_sum/calibration_cycles; //gets the newly calibrated theta value
    }
    
  if (digitalRead(mode_pin) == HIGH){ //checks if the switch is not being pressed
    if (mode_switch_pressed == true){ //checks if the switch had previously been pressed
      end_mode_pressed = millis(); //takes the time the switch was unpressed
      time_mode_pressed = end_mode_pressed - start_mode_pressed; //calclulates the time that the switch was pressed for
      if (time_mode_pressed < calibration_time){ //changes the mode if the time the button was pressed was less than 3 seconds
        if (mode == 4){ //goes back to mode 1 when the button is pressed while in mode 4
          mode = 1;
        }
        else{
          mode += 1;
        }
      }
    }
    mode_switch_pressed = false; //the mode is no longer being pressed
  }
  if (digitalRead(mode_pin) == LOW and mode_switch_pressed == false){ //checks if the button has began to be pressed
    start_mode_pressed = millis(); //takes the current time as when the button is first pressed
    mode_switch_pressed = true; //indicates that the button is currently being pressed
  }
  return mode;
}

void autoload_func(){
  /* This function loads the ball into the pitching machine when the load button has been pressed. There is also an option
   *  to hold the load button in to unload a ball that has been stuck in the pitching machine
   */
  if (digitalRead(autoload_pin) == HIGH){ //checks if the button is unpressed
    if (autoload_switch_pressed == true){ //checks if the button had presviously been pressed
      end_autoload_pressed = millis(); //takes the time that the button had been unpressed
      time_autoload_pressed = end_autoload_pressed - start_autoload_pressed; //takes the total time that the button has been pressed
      if (time_autoload_pressed < remove_ball_time){ //if the button was pressed for less than 3 seconds, a ball will be loaded
        Serial.println("Starting to Load");
        DriveAutoload(); //autoload function is called to load a ball
        Serial.println("System now ready");
      }
      else{ //if the button had been pressed for over 3 seconds, the pitching machine is tiled down to unload a stuck ball
        while (sensorVal > 150){ //moves pitching machine down to this position, this will point it downwards
          sensorVal = analogRead(sensorPin);
          driveActuator(-1,ActuatorSpeed);
        }
        while (sensorVal < 480){ //moves pitchine machine back up to the standard postion
          sensorVal = analogRead(sensorPin);
          driveActuator(1,ActuatorSpeed);
        }
        Serial.println("Done Releasing");
        ActuatorPos = sensorVal;
      }
    }
    autoload_switch_pressed = false; //indicates that the button is no longer being pressed
  }
  
  if (digitalRead(autoload_pin) == LOW and autoload_switch_pressed == false){ //checks if the button is being pressed, and previous had not been pressed
    start_autoload_pressed = millis(); //takes the time that the butoon began to be pressed
    autoload_switch_pressed = true; //indicates that the button is currently being pressed
  } 
}

void setup() {
  Serial.begin(115200);
  myIMU.begin();
  vcnl.begin();
  delay(1000);
  int8_t temp = myIMU.getTemp();
  myIMU.setExtCrystalUse(true);
  
  pinMode(mode_pin, INPUT_PULLUP); //sets buttons as pullups
  pinMode(autoload_pin, INPUT_PULLUP);
  
  myServo.attach(2, 1322, 1622); //sets the min and max positions on the servo, these equate to -45 to 45 degrees
  ServoPos = 0;
  microPos = map(ServoPos,-45,45,1322,1622);
  myServo.writeMicroseconds(microPos); //moves the servo to its centered position
  
  pinMode(autoload_RPWM, OUTPUT);
  pinMode(autoload_LPWM, OUTPUT);
  pinMode(actuator_RPWM, OUTPUT);
  pinMode(actuator_LPWM, OUTPUT);
  analogWrite(actuator_RPWM, 0);
  analogWrite(actuator_LPWM, 0);
  pinMode(sensorPin, INPUT);
  
  minAnalogReading = 50; //sets minimum possible value of the linear actuator
  maxAnalogReading = 800; //sets minimum possible value of the linear actuator
  ActuatorPos = 300; //sets inital position of the linear actuator
  delay(2000);
  Serial.println("Machine Ready");
}

void loop() {

  //continues to get the IMU position 
  phiFold = get_phi();
  thetaFold = get_theta(); 

  //continues to call the functions that check whether either the mode or the autoloader button is being pressed
  mode = mode_button_func();
  autoload_func();
  
  if (mode == 1){// fielding mode, stopped
    MoveServo(theta_calibrated_zero, 5, 50, 0.5, 10, 43); //won't move the servo
    MoveActuator(phi_calibrated_zero, 5, 60, 20, 40, 480, 750); //won't move the linear actuator
  }
  else if (mode == 2){// fine/pitching mode
    MoveServo(thetaFold, 5, 50, 0.5, 5, 20); //allows movement of +/- 20 degress from zeroed position, moves finely
    MoveActuator(phiFold, 5, 50, 5, 40, 480, 600); //allows minimal movement up and down to only allow roughly pitching angles
  }
  else if (mode == 3){
    ; //pitching mode, stopped
    MoveServo(theta_calibrated_zero, 5, 50, 1, 5, 20); //won't move the servo
    MoveActuator(phi_calibrated_zero, 5, 50, 5, 40, 480, 600); //won't move the linear actuator
  }
  else if (mode == 4){ // coarse/fielding mode
    MoveServo(thetaFold, 5, 50, 2, 10, 43); //allows movement of +/- 43 degrees from zeroed position, moves more coarsely
    MoveActuator(phiFold, 5, 60, 20, 40, 480, 750); //allows higher angles for pop flys
  }
  delay(10);
}
