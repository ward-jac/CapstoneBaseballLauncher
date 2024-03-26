//  Pins for BT connection
//  BT VCC to Arduino 5V out.
//  BT GND to GND
//  Arduino 48 (Mega) RX -> BT TX no need voltage divider
//  Arduino 46 (Mega) TX -> BT RX through a voltage divider (5v to 3.3v)

// https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
#include <AltSoftSerial.h>
#include <avr/io.h>
#include <stdlib.h>
#include "include/IMU.h"

// to define the IMU
// TODO: edit for BNO085
const int IMU_ADDRESS = 55;  // the model of the IMU
IMU _imu(IMU_ADDRESS);

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
            speaker.play("Release to calibrate");
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
                if(!isLocked){
                    speaker.play("Launcher unlocked");
                }
                else {
                    speaker.play("Launcher locked");
                }
            }
        }
    }
}

// press adds 10 to speed
// holding down for holdTime toggles sensitivity
void checkSpeedInput() {
  // check if input is initially activated
  if (digitalRead(SPEED_INPUT_PIN) == LOW && !speedPressed) {
    speedPressed = true;
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

void checkFireInput() {
  // check if input is active
  if (digitalRead(AUTOLOAD_INPUT_PIN) == LOW) {
  }
}

void sendMessage(float theta, float phi) {
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

void setup() {
  // initialize the serial monitor for testing purposes
  Serial.begin(9600);

  // initialize the sender BT
  sender.begin(9600);

  // start and calibrate the IMU
  _imu.begin();
  delay(1000);
  _imu.calibrate();

  // crucial
  delay(1000);
}

    speechRecognition();
    checkLockInput();
    checkSpeedInput();
    checkFireInput();

  // do not send IMU data if lock is enabled
  if (!isLocked) {
    float theta = _imu.getTheta();
    float phi = _imu.getPhi();
    //sendMessage function

    Serial.println("phi: " + String(phi) + " / theta: " + String(theta));
  }

  delay(10);
}