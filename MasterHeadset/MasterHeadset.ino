#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <avr/io.h>
#include <stdlib.h>
#include <Arduino.h>
#include "include/IMU.h"

// IMU
const int IMU_ADDRESS = 55;
IMU _imu(IMU_ADDRESS);
AltSoftSerial sender;

#define BTSerial Serial3
#define DFSerial Serial2
#define SpeechSerial Serial1

String speechMsg = "";
DFRobotDFPlayerMini myDFPlayer;
bool DFPlayerActive;

// Bluetooth
//  Arduino 48 (Mega) RX -> BT TX no need voltage divider
//  Arduino 46 (Mega) TX -> BT RX through a voltage divider (5v to 3.3v)
// AltSoftSerial bluetoothSerial;

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
    Serial.begin(19200);
    SpeechSerial.begin(9600);
    BTSerial.begin(9600);
    _imu.begin();
    
    /*
     (val): Control name
    (1-10): Speed control - sets speed to val*10 speed
        11: Lock Launcher
        12: Unlock Launcher
        13: Sensor Calibrated
        14: Face Forward to Calibrate
        15: Fire
    */
    DFSerial.begin(9600);
    if (!myDFPlayer.begin(DFSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
      Serial.println("No Connection to DFPlayer");
      DFPlayerActive = false;
    } else {
      DFPlayerActive = true;
      Serial.println(F("Connection Succesfull"));
      myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms
      myDFPlayer.volume(70);
      myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
      myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
    }
}

void loop()
{
    if (startUp)
    {
        delay(500);
        //_imu.calibrate();
        delay(500);
    }
    //checkLockInput();
    //checkSpeedInput();
    //checkFireInput();
    while(SpeechSerial.available()>0) {
        char c = SpeechSerial.read();
        speechMsg += c;
        if(c=='\n') {
            speechMsg = speechMsg.substring(0,speechMsg.length()-1);
            Serial.println(speechMsg);
            int val = speechMsg.toInt();
            speechMsg = "";

            // speed control
            if(val<=10) {
                int diff = val*10 - currSpeed;
                currSpeed = val*10;
                if(DFPlayerActive) {
                  myDFPlayer.playMp3Folder(val);
                  BTSerial.print("S" + String(diff) + '\n');
                }
            }
            else {
              /*
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
                */
              }
          }
    }



    // Do not send IMU data if lock is enabled
    if(!isLocked)
    {   
        //float theta = _imu.getTheta();
        //float phi = _imu.getPhi();
        //sendMessage function

        //Serial.println("phi: " + String(phi) + " / theta: " + String(theta));
    }

    startUp = false;
    delay(10);
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
    else if (lockInputActive && millis() - lockStartTime > holdTime)
    {
        isLocked = true;
        inCalibration = true;
        myDFPlayer.playMp3Folder(14);
    }

    // check if input is released
    else if (digitalRead(LOCK_INPUT_PIN) == HIGH && lockInputActive)
    {
        lockInputActive = false;
        if (inCalibration)
        {
            _imu.calibrate();
            isLocked = false;
            inCalibration = false;
            myDFPlayer.playMp3Folder(13);
        }
        else
        {
            isLocked = !isLocked;
            if(!isLocked){
                myDFPlayer.playMp3Folder(12);
            }
            else {
                myDFPlayer.playMp3Folder(11);
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
            myDFPlayer.playMp3Folder(currSpeed/10);
            //bluetoothSerial.print("S10\n");
        }
    }
}

void checkFireInput() {
  // check if input is active
  if (digitalRead(AUTOLOAD_INPUT_PIN) == LOW) {
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
    //sender.write(data.charAt(i));
    // Serial.print("Data sent: ");
    // Serial.println(data.charAt(i));
    }
}