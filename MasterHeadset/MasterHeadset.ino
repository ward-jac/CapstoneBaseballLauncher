#include <DFRobotDFPlayerMini.h>
#include <avr/io.h>
#include <stdlib.h>
#include <Arduino.h>
#include "include/IMU.h"

// Bluetooth
//  Arduino 48 (Mega) RX -> BT TX no need voltage divider
//  Arduino 46 (Mega) TX -> BT RX through a voltage divider (5v to 3.3v)
// AltSoftSerial bluetoothSerial;

const int SPEED_INPUT_PIN = 7;
const int LOCK_INPUT_PIN = 6;
const int IMU_ADDRESS = 55;
#define BTSerial Serial3
#define DFSerial Serial2
#define SpeechSerial Serial1

IMU _imu(IMU_ADDRESS);
DFRobotDFPlayerMini myDFPlayer;
bool DFPlayerActive;

// Speed Control
const int launcherProcessingSpeed = 200; // fastest time that launcher can process speed change inputs
unsigned int currSpeed = 40;
unsigned long speedStartTime;
boolean speedInputActive = false;
boolean inFire = false;

// Locking
unsigned long lockStartTime;
boolean lockInputActive = false;
boolean isLocked = false;
boolean inCalibration = false;

const long holdTime = 2000; // button hold down time
String speechMsg = "";      // speech recognition
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
  if (!myDFPlayer.begin(DFSerial, /*isACK = */ true, /*doReset = */ true))
  { // Use serial to communicate with mp3.
    Serial.println("No Connection to DFPlayer");
    DFPlayerActive = false;
  }
  else
  {
    DFPlayerActive = true;
    Serial.println(F("Connection Succesfull"));
    myDFPlayer.setTimeOut(500); // Set serial communictaion time out 500ms
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
    _imu.calibrate();
    delay(500);
  }

  checkInput_LockCalibrate();
  checkInput_SpeedFire();

  while (SpeechSerial.available() > 0)
  {
    char c = SpeechSerial.read();
    speechMsg += c;
    if (c == '\n')
    {
      speechMsg = speechMsg.substring(0, speechMsg.length() - 1);
      Serial.println(speechMsg);
      int val = speechMsg.toInt();
      speechMsg = "";

      // speed control
      if (val <= 10)
      {
        int diff = val * 10 - currSpeed;
        currSpeed = val * 10;
        if (DFPlayerActive)
        {
          myDFPlayer.playMp3Folder(val);
          // ----- SEND BLUETOOTH MESSAGE ----------
          BTSerial.print("S" + String(diff) + '\n');
        }
      }
      else
      {
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
  if (!isLocked)
  {
    float theta = _imu.getTheta();
    float phi = _imu.getPhi();
    Serial.println("phi: " + String(phi) + " / theta: " + String(theta));
    // ----- SEND BLUETOOTH MESSAGE ----------
    BTSerial.print("P" + String(phi) + "T" + String(theta) + '\n');
  }

  startUp = false;
  delay(10);
}

// press toggles lock on/off
// holding down for holdTime enters calibration mode
char checkInput_LockCalibrate()
{
  // check if input is initially activated
  if (digitalRead(LOCK_INPUT_PIN) == HIGH && !lockInputActive)
  {
    lockInputActive = true;
    lockStartTime = millis();
  }

  // check if input is held for holdTime, lock launcher for calibration
  else if (lockInputActive && millis() - lockStartTime == holdTime)
  {
    isLocked = true;
    inCalibration = true;
    myDFPlayer.playMp3Folder(14);
  }

  // check if input is released
  else if (digitalRead(LOCK_INPUT_PIN) == LOW && lockInputActive)
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
      if (!isLocked)
      {
        myDFPlayer.playMp3Folder(12);
      }
      else
      {
        myDFPlayer.playMp3Folder(11);
      }
    }
  }
}

// press adds 10 to speed
// holding down for holdTime fire ball
char checkInput_SpeedFire()
{
  // check if input is initially activated
  if (digitalRead(SPEED_INPUT_PIN) == HIGH && !speedInputActive)
  {
    speedInputActive = true;
    speedStartTime = millis();
  }

  // check if input is held for holdTime
  else if (speedInputActive && millis() - speedStartTime == holdTime)
  {
    inFire = true;
    myDFPlayer.playMp3Folder(15);
  }

  // check if input is released
  else if (digitalRead(SPEED_INPUT_PIN) == LOW && speedInputActive)
  {
    speedInputActive = false;
    if (inFire)
    {
      inFire = false;
      // ----- SEND BLUETOOTH MESSAGE ----------
      BTSerial.print("F\n");
      // myDFPlayer.playMp3Folder(15); play a different message for firing?
    }
    else
    {
      currSpeed+=10;
      if(currSpeed>100) {
        currSpeed = 10;
        // ----- SEND BLUETOOTH MESSAGE ----------
        BTSerial.print("S" + String(-90) + '\n');
      }
      else {
        // ----- SEND BLUETOOTH MESSAGE ----------
        BTSerial.print("S" + String(10) + '\n');
      }
    }
  }
}

/* Unecessary?
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
*/