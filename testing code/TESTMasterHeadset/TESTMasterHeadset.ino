//#include <AltSoftSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <avr/io.h>
#include <stdlib.h>
#include <Arduino.h>
#include <Servo.h>

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



Servo myServo;
const int maxAngle = 30;
const int servoPin = 9;
int servoPos = 0;

const int sensLevel1 = 2;
const int sensLevel2 = 5;
const int sensLevel3 = 10;
const int sensLevel4 = 20;

const int minAngle1 = 10;
const int minAngle2 = 20;
const int minAngle3 = 30;
const int minAngle4 = 40;

void setup()
{
    //myServo.writeMicroseconds(0);
    myServo.attach(servoPin);

    Serial.begin(19200);
    SpeechSerial.begin(9600);
    BTSerial.begin(9600);
    
    /*
     (val): Control name
    (1-10): Speed control - sets speed to val*10 speed
        11: Lock Launcher
        12: Unlock Launcher
        13: Sensor Calibrated
        14: Face Forward to Calibrate
        15: Fire
    */
    //DFSerial.begin(9600);
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
        myServo.write(180);
        delay(2000);
        myServo.write(90);
        delay(2000);
        myServo.write(100);


    }
    startUp = false;
    delay(10);
}

void moveServo(float theta) {
  int sensLevel1 = 2;
  int sensLevel2 = 5;
  int sensLevel3 = 10;
  int sensLevel4 = 20;
  Serial.println(servoPos);

  while

  // use this
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(1);                       // waits 15 ms for the servo to reach the position
    if()
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(1000);                       // waits 15 ms for the servo to reach the position
  }

  if(servoPos<45 && servoPos>-45) {
    if(theta>minAngle4) {
      servoPos = servoPos + sensLevel4;
      myServo.write(servoPos);
    }
    else if(theta>minAngle3) {
      servoPos = servoPos + sensLevel3;
      myServo.write(servoPos);
    }
    else if(theta>minAngle2) {
      servoPos = servoPos + sensLevel2;
      myServo.write(servoPos);
    }
    else if(theta>minAngle1) {
      servoPos = servoPos + sensLevel1;
      myServo.write(servoPos);
    }
    else if(theta<minAngle1 && theta>-minAngle1) {
      servoPos = 90;
      myServo.write(servoPos);
    }
    else if(theta<-minAngle1) {
      servoPos = servoPos - sensLevel1;
      myServo.write(servoPos);
    }
    else if(theta<-minAngle2) {
      servoPos = servoPos - sensLevel2;
      myServo.write(servoPos);
    }
    else if(theta<-minAngle3) {
      servoPos = servoPos - sensLevel3;
      myServo.write(servoPos);
    }
    else if(theta<-minAngle4) {
      servoPos = servoPos - sensLevel4;
      myServo.write(servoPos);
    }
  }
  delay(80);
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
    //sender.write(data.charAt(i));
    // Serial.print("Data sent: ");
    // Serial.println(data.charAt(i));
    }
}