#include <SoftwareSerial.h>
#include "include/Launcher.h"

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
  if(bluetoothSerial.read() =='P') {
    char curr = bluetoothSerial.read();
    char phiArray[7];
    char thetaArray[7];
    int idx = 0;
    while(curr!='T') {
      phiArray[idx] = curr;
      idx++;
      curr = bluetoothSerial.read();
    }
    while(curr!='\n') {
      thetaArray[idx] = curr;
      idx++;
      curr = bluetoothSerial.read();
    }
  }


  _launcher.moveAct();
  _launcher.moveServo();

  delay(launcherTickRate);
}
