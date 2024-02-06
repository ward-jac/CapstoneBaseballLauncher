#ifndef IMU_h
#define IMU_h
#include <Wire.h>
#include <utility/imumaths.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

class IMU {
public:
    IMU(int imuAddress);
    void begin();
	void calibrate();
    float getTheta();
    float getPhi();
    int getTickRate();

private:
	int _imuAddress;
    float euler_shift_theta;
    float euler_shift_phi;
    int updateCount;
    Adafruit_BNO055 myIMU;
};
#endif