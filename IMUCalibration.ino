// required for IMU calculations
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// the sample rate delay and IMU
#define BNO055_SAMPLERATE_DELAY_MS (1000)
Adafruit_BNO055 myIMU = Adafruit_BNO055(55);

// the curent euler angles at the time of calibration
float euler_shift_theta = 0.0;
float euler_shift_phi = 0.0;

int updateCount = 0;

// returns the shifted value of theta after calibration
float getTheta() {
  
  imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  int shifted = euler.x() + (-1 * euler_shift_theta);

  // to bound the angle from -180 to 180 degrees
  if (shifted > 180.0) {
    return shifted - 360.0;
  }
  else {
    return shifted;
  }
}

// returns the shifted value of phi after calibration
float getPhi() {
  
  imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.z() + (-1 * euler_shift_phi);
}

// updates the euler shift angles (theta and phi)
void updateEulerShifts() {
  
  // calibrate and obtain the euler angles from the IMU
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);

  // euler x = theta
  euler_shift_theta = euler.x();

  // euler z = phi
  euler_shift_phi = euler.z();
}

void setup(void) 
{
  // begin serial communication at a 9600 baud rate
  Serial.begin(9600);
    
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
    updateEulerShifts();
    updateCount++;
  }

  Serial.print("Theta: ");
  Serial.println(getTheta());
  Serial.print("Phi: ");
  Serial.println(getPhi());
  Serial.println("");

  // delay for the designated sample rate delay
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
