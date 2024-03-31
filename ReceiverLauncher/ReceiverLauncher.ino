#include <AltSoftSerial.h>
#include "include/Launcher.h"

const int launcherProcessingSpeed = 500;

const int SPEEDUP_PIN = 123;
const int SPEEDDOWN_PIN = 123;

AltSoftSerial bluetoothSerial;
Launcher _launcher;

String btMsg = "";

char speechRecognition() {
}

void clearBluetoothBuffer() {
  while(bluetoothSerial.available()) {
    char clear = bluetoothSerial.read();
  }
}

void setup() {
    // Open serial communications and wait for port to open:
    Serial.begin(19200);
    // set the data rate for the SoftwareSerial port
    bluetoothSerial.begin(9600);
}

void loop() {
  // read PHI THETA values from Master IMU to write to Reciever actuator and servo
  //Serial.println(String(bluetoothSerial.read()));

  //float val = bluetoothSerial.parseFloat();
  if(bluetoothSerial.read()=='\n'){
    // indicated end
  }

  if(bluetoothSerial.read() =='P') 
  { 
    //Serial.println("Recieved P");
    //float phi = readB();
    //float theta = readB();

    /*
    String str = bluetoothSerial.readString();
    float phi = str.substring(0, str.indexOf(" ")).toFloat();
    float theta = str.substring(str.indexOf(" "), str.length()).toFloat();
    */

    Serial.println("phi: " + String(phi) + " / theta: " + String(theta));
  
    _launcher.moveAct(phi);
    _launcher.moveServo(theta);
  }

  else if(bluetoothSerial.read() =='S')
  {
    //float num = readB();

    while(bluetoothSerial.available()>0) {
        char c = bluetoothSerial.read();
        btMsg += c;
        if(c=='\n') {
            btMsg = btMsg.substring(0,btMsg.length()-1);
            Serial.println(btMsg);
            int val = btMsg.toInt();
            btMsg = "";
        }
    }

    //Serial.println("Recieved: " + String(num));
    //_launcher.addSpeed(num);
  }

  else if(bluetoothSerial.read() =='H')
  {
    _launcher.setSensitivity('H');
  }

  else if(bluetoothSerial.read() =='L')
  {
    _launcher.setSensitivity('L');
  }
}
