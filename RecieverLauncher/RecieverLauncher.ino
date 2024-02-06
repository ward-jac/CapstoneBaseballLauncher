#include <SoftwareSerial.h>
#include "include/Launcher.h"

// IMU tick rate (500ms) + Microphone 
const int launcherTickRate = 500;

// bluetooth
const int RX_PIN;
const int TX_PIN;

// launcher pins
const int ACTUATOR_PIN;
const int SERVO_PIN;

// peripherals
const int FIRE_BUTTON_PIN;

SoftwareSerial bluetoothSerial(RX_PIN, TX_PIN);
Launcher _launcher(ACTUATOR_PIN, SERVO_PIN);

char speechRecognition() {
}

void setup() {
    // Open serial communications and wait for port to open:
    Serial.begin(19200);
    // set the data rate for the SoftwareSerial port
    bluetoothSerial.begin(9600);
}

void loop() {

  // reading values from IMU over bluetooth to write to actuator and servo
  if(bluetoothSerial.read() =='P') {
    char phiArray[7];
    char thetaArray[7];
    char curr = bluetoothSerial.read();
    int idx = 0;
    while(curr!='T') {
      phiArray[idx] = curr;
      idx++;
      curr = bluetoothSerial.read();
    }
    idx = 0;
    while(curr!='\n') {
      thetaArray[idx] = curr;
      idx++;
      curr = bluetoothSerial.read();
    }

    Serial.println("phi: " + String(atof(phiArray)) + " / theta: " + String(atof(thetaArray)));
  
    _launcher.moveAct(atof(phiArray));
    _launcher.moveServo(atof(thetaArray));
  }

  delay(launcherTickRate);
}
