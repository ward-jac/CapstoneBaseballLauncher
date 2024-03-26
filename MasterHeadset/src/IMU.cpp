// TODO: edit for BNO085

#include <Wire.h>
#include <utility/imumaths.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include "../include/IMU.h"

#define BNO055_SAMPLERATE_DELAY_MS (500)

IMU::IMU(int imuAddress) {
  _imuAddress = imuAddress;
  euler_shift_theta = 0.0;
  euler_shift_phi = 0.0;
  myIMU = Adafruit_BNO055(_imuAddress);
}

void IMU::begin() {
    myIMU.begin();
    delay(1000);
    int8_t temp = myIMU.getTemp();
    myIMU.setExtCrystalUse(true);
    delay(1000);
}

// calibrates by setting the euler shift offset values (theta and phi)
void IMU::calibrate() {
  // obtain the euler angles from the IMU
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);

  // euler x = theta
  euler_shift_theta = euler.x();

  // euler z = phi
  euler_shift_phi = euler.z();
}

// returns the shifted value of theta after calibration
float IMU::getTheta() {
  imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.x() - euler_shift_theta;
}

// returns the shifted value of phi after calibration
float IMU::getPhi() {
  imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.z() - euler_shift_phi;
}