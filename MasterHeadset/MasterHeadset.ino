#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
#include <avr/io.h>
#include <stdlib.h>
// #include <Arduino.h>
#include "include/IMU.h"

// IMU
const int IMU_ADDRESS = 55;
IMU _imu(IMU_ADDRESS);
AltSoftSerial sender;

// Audio
const int AU_RX_PIN = 15;
const int AU_TX_PIN = 14;
SoftwareSerial audioSerial(AU_RX_PIN, AU_TX_PIN);

// Speech
const int SP_RX_PIN = 17;
const int SP_TX_PIN = 16;
SoftwareSerial speechSerial(SP_RX_PIN, SP_TX_PIN);

// Bluetooth
//  Arduino 48 (Mega) RX -> BT TX no need voltage divider
//  Arduino 46 (Mega) TX -> BT RX through a voltage divider (5v to 3.3v)
AltSoftSerial bluetoothSerial;

// how long buttons need to be held to change response
const long holdTime = 2000;

// Speed Control
const int SPEED_INPUT_PIN = 7;
const int launcherProcessingSpeed = 200; // fastest time that launcher can process speed change inputs
unsigned int currSpeed = 40;
unsigned long speedStartTime;
boolean speedInputActive = false;

// Locking
const int LOCK_INPUT_PIN = 6;
unsigned long lockStartTime;
boolean lockInputActive = false;
boolean isLocked = false;
boolean inCalibration = false;


const int AUTOLOAD_INPUT_PIN = 5;
boolean startUp = true;

void setup()
{
    speechSerial.begin(9600);
    audioSerial.begin(9600);
    bluetoothSerial.begin(9600);
    _imu.begin();
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

    // check if input is held for holdTime, lock launcher for calibration
    if (lockInputActive && millis() - lockStartTime > holdTime)
    {
        isLocked = true;
        inCalibration = true;
        audioSerial.print("14\n");
    }

    // check if input is released
    if (digitalRead(LOCK_INPUT_PIN) == HIGH && lockInputActive)
    {
        lockInputActive = false;
        if (inCalibration)
        {
            _imu.calibrate();
            isLocked = false;
            inCalibration = false;
            audioSerial.print("13\n");
        }
        else
        {
            isLocked = !isLocked;
            if(!isLocked){
                audioSerial.print("12\n");
            }
            else {
                audioSerial.print("11\n");
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
        }
        else
        {
            if(currSpeed==100){
                currSpeed = 10;
            }
            else {
                currSpeed+=10;
            }
            audioSerial.print(String(currSpeed/10) + '\n');
            bluetoothSerial.print("S10\n");
        }
    }
}

char checkFireInput()
{
    // check if input is active
    if (digitalRead(AUTOLOAD_INPUT_PIN) == LOW)
    {
    }
}

void sendMessage(float theta, float phi)
{
    // start of text char
    char stx = 2;

    // end of text char
    char etx = 3;

    // create the data string to send over BT
    String data = stx + String(theta, 1) + " " + String(phi, 1) + etx;

    // send the string over BT char by char
    for (int i = 0; i < data.length(); i++) {
    sender.write(data.charAt(i));
    // Serial.print("Data sent: ");
    // Serial.println(data.charAt(i));
    }
}

void loop()
{
    if (startUp)
    {
        delay(500);
        _imu.calibrate();
        delay(500);
    }

    checkLockInput();
    checkSpeedInput();
    checkFireInput();

    String msg = "";
    while(speechSerial.available() > 0) {
        char c = speechSerial.read();
        msg += c;
        if(c=='\n') {
            msg = msg.substring(0,msg.length()-1);
            int val = msg.toInt();
            msg = "";
            // speed control
            if(val<=10) {
                int diff = val*10 - currSpeed;
                currSpeed = val*10;
                audioSerial.print(String(currSpeed/10) + '\n');
                bluetoothSerial.print("S" + String(diff) + '\n');
            }
            else {
                switch(val) {
                    // lock
                    case 11:

                    // unlock
                    case 12:
                    
                    // Calibrate
                    case 13:

                    // Fire
                    case 14:
                }
            }
        }
    }



    // Do not send IMU data if lock is enabled
    if(!isLocked)
    {   
        float theta = _imu.getTheta();
        float phi = _imu.getPhi();
        //sendMessage function

        Serial.println("phi: " + String(phi) + " / theta: " + String(theta));
    }

    startUp = false;
    delay(10);
}