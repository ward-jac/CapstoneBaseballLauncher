#ifndef IMU_h
#define IMU_h
#include <Wire.h>
#include <utility/imumaths.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define BNO055_SAMPLERATE_DELAY_MS (1000)

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
    int updateCount;
    Adafruit_BNO055 myIMU;
};
#endif