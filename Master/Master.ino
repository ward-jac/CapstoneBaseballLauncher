#include "include/IMU.h"
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>

const int RX_PIN;
const int TX_PIN;
SoftwareSerial bluetoothSerial(RX_PIN, TX_PIN);

const int IMU_ADDRESS = 55;
IMU _imu(IMU_ADDRESS);

boolean startUp;

void setup() {
    // Open serial communications and wait for port to open:
    Serial.begin(19200);
    // set the data rate for the SoftwareSerial port
    bluetoothSerial.begin(9600);

    _imu.begin();

    startUp = true;
}

void loop() {
    if (startUp) {
        _imu.calibrate();
    }



    bluetoothSerial.write('P');
    int floatLength = 5;
    for(int i=0; i<floatLength; i++) {
        bluetoothSerial.write(_imu.getPhi());
    }
    
    bluetoothSerial.write('T');
    bluetoothSerial.write(_imu.getTheta());

    Serial.println("phi: " + String(_imu.getPhi()) + " / theta: " + String(_imu.getTheta()));
    
    startUp = false;
    delay(BNO055_SAMPLERATE_DELAY_MS);
}