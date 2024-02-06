#include <SoftwareSerial.h>
//#include <Arduino.h>
#include "include/IMU.h"

// IMU pins
const int IMU_ADDRESS = 55;
const int RX_PIN = 0;
const int TX_PIN = 1;
// Button pins, rename
const int BUTTON_1 = 2; 
const int BUTTON_2 = 3;
const int BUTTON_3 = 4;

IMU _imu(IMU_ADDRESS);
SoftwareSerial bluetoothSerial(RX_PIN, TX_PIN);

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

    // writing phi over bluetooth
    float phi = _imu.getPhi();
    char bufferPhi[7] // (+/-) x x x . x x
    dtostrf(phi, 3, 2, bufferPhi);
    bluetoothSerial.write('P');
    bluetoothSerial.write(bufferPhi);
    
    // writing theta over bluetooth
    float theta = _imu.getTheta();
    char bufferTheta[7] // (+/-) x x x . x x
    dtostrf(theta, 3, 2, bufferTheta);
    bluetoothSerial.write('T');
    bluetoothSerial.write(bufferTheta);
    bluetoothSerial.write('\n');

    Serial.println("phi: " + String(phi) + " / theta: " + String(theta));
    
    startUp = false;
    delay(_imu.getTickRate());
}