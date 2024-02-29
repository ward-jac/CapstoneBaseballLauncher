// required for IMU calculations
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// for sign operations
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

// the sample rate delay and IMU
#define BNO055_SAMPLERATE_DELAY_MS (500)
Adafruit_BNO055 myIMU = Adafruit_BNO055(55);

//  Pins for BT
//  BT VCC to Arduino 5V out. 
//  BT GND to GND
//  Arduino 48 (Mega) RX -> BT TX no need voltage divider 
//  Arduino 46 (Mega) TX -> BT RX through a voltage divider (5v to 3.3v)

// https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
#include <AltSoftSerial.h>
AltSoftSerial sender;

char c = ' ';
boolean NL = true; 

// the curent angles at the time of calibration
float shift_theta = 0.0;
float shift_phi = 0.0;
int updateCount = 0;

// the maximum values of theta and phi
int maxAngle = 30;

// to deal with gimbal lock
int dangerZone = 60;

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
  // start serial monitor communication at 9600 baud rate
  Serial.begin(9600);
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);
  Serial.println(" ");
 
  // start sender BT at 9600 baud rate
  sender.begin(9600);  
  Serial.println("Sender started at 9600");
    
  myIMU.begin();
  delay(1000);
  int8_t temp = myIMU.getTemp();
  myIMU.setExtCrystalUse(true);
  
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

  // send theta and phi to receiver separated by a space
  sender.print(getTheta(euler));
  sender.print(" ");
  sender.println(getPhi(euler));

  // delay for the designated sample rate delay
  delay(BNO055_SAMPLERATE_DELAY_MS);
}