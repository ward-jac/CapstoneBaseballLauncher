#include <AltSoftSerial.h>
#include <avr/io.h>
#include <stdlib.h>
// #include <Arduino.h>
#include "include/IMU.h"
#include "include/Audio.h"

// IMU
const int IMU_ADDRESS = 55;
IMU _imu(IMU_ADDRESS);
AltSoftSerial bluetoothSerial;

// Audio
const int SP_RX_PIN = 123;
const int SP_TX_PIN = 123;
Audio speaker(SP_RX_PIN, SP_TX_PIN);

// how long buttons need to be held to change response
const long holdTime = 2000;

// Speed Control
const int SPEED_INPUT_PIN = 4;
const int launcherProcessingSpeed = 200; // fastest time that launcher can process speed change inputs
unsigned int currentSpeed = 40;
unsigned long speedStartTime;
boolean speedInputActive = false;
boolean isSensitivityLow = true;

// Locking
const int LOCK_INPUT_PIN = 2;
unsigned long lockStartTime;
boolean lockInputActive = false;
boolean isLocked = false;
boolean inCalibration = false;

// AutoLoad
const int AUTOLOAD_INPUT_PIN = 3;

boolean startUp = true;

void setup()
{
    // Open serial communications and wait for port to open:
    Serial.begin(19200);
    Serial.println("Hello");

    bluetoothSerial.begin(9600);
    bluetoothSerial.println("Hello, world?");

    _imu.begin();
}

void speechRecognition()
{
}

// press toggles lock on/off
// holding down for holdTime enters calibration mode
char checkLockInput()
{
    // check if input is initially activated
    if (digitalRead(LOCK_INPUT_PIN) == LOW && !lockInputActive)
    {
        lockInputActive = true;
        lockStartTime = millis();
    }

    else
    {
        // check if input is held for holdTime, lock launcher for calibration
        if (millis() - lockStartTime == holdTime)
        {
            isLocked = true;
            speaker.play("Face To Calibrate");
            inCalibration = true;
        }

        // check if input is released
        if (digitalRead(LOCK_INPUT_PIN) == HIGH && lockInputActive)
        {
            lockInputActive = false;
            if (inCalibration)
            {
                _imu.calibrate();
                speaker.play("Calibrated");
                isLocked = false;
                inCalibration = false;
            }
            else
            {
                isLocked = !isLocked;
            }
        }
    }
}

// press adds 10 to speed
// holding down for holdTime toggles sensitivity
char checkSpeedInput()
{
    // check if input is initially activated
    if (digitalRead(SPEED_INPUT_PIN) == LOW && !speedInputActive)
    {
        speedInputActive = true;
        speedStartTime = millis();
    }

    // check if input is released
    else if (digitalRead(SPEED_INPUT_PIN) == HIGH && speedInputActive)
    {
        if (millis() - speedStartTime == holdTime)
        {
            speaker.play("Changing Sensitivity");
            isSensitivityLow = !isSensitivityLow;
            if(isSensitivityLow)
            {
                bluetoothSerial.write('L');
            }
            else
            {
                bluetoothSerial.write('H');
            }
        }
        else
        {
            float speed = 10;
            speaker.play("Add 10 Speed");

            bluetoothSerial.write('S');
            sendFloat(10);
        }
    }
}

char checkAutoLoadInput()
{
    // check if input is active
    if (digitalRead(AUTOLOAD_INPUT_PIN) == LOW)
    {
    }
}

void sendFloat(float f)
{
    // Convert the float to a byte array
    byte byteArray[sizeof(float)];
    memcpy(byteArray, &f, sizeof(float));

    // Send the byte array over Bluetooth
    bluetoothSerial.write(byteArray, sizeof(byteArray));
}

void loop()
{
    if (startUp)
    {
        _imu.calibrate();
    }

    speechRecognition();
    checkLockInput();
    checkSpeedInput();
    checkAutoLoadInput();

    // Do not send IMU data if lock is enabled
    if(!isLocked)
    {   
        /*
        float phi = _imu.getPhi();
        float theta = _imu.getTheta();
        bluetoothSerial.print(phi);
        bluetoothSerial.print(" ");
        bluetoothSerial.println(theta);
        Serial.println("phi: " + String(phi) + " / theta: " + String(theta));
        */
       
        float phi = _imu.getPhi();
        bluetoothSerial.write('P');
        sendFloat(phi);

        float theta = _imu.getTheta();
        sendFloat(theta);

        Serial.println("phi: " + String(phi) + " / theta: " + String(theta));

        /*
        // writing phi over bluetooth
        float phi = _imu.getPhi();
        char bufferPhi[7]; // (+/-) x x x . x x
        dtostrf(phi, 3, 2, bufferPhi);
        bluetoothSerial.write('P');
        bluetoothSerial.write(bufferPhi);

        // writing theta over bluetooth
        float theta = _imu.getTheta();
        char bufferTheta[7]; // (+/-) x x x . x x
        dtostrf(theta, 3, 2, bufferTheta);
        bluetoothSerial.write('T');
        bluetoothSerial.write(bufferTheta);
        bluetoothSerial.write('\n');
        */
    }

    startUp = false;
    delay(10);
}