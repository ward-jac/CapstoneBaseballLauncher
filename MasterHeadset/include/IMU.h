// TODO: edit for BNO085

#ifndef IMU_h
#define IMU_h
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_Sensor.h>

class IMU {
public:
    IMU(int imuAddress);
    void begin();
	void calibrate();
    float getTheta();
    float getPhi();

private:
	int _imuAddress;
    float euler_shift_theta;
    float euler_shift_phi;
    Adafruit_BNO055 myIMU;
};
#endif