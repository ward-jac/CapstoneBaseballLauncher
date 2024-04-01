#include "include/Launcher.h"

//AltSoftSerial bluetoothSerial;
#define BTSerial Serial3

Launcher _launcher;

int theta = 0;

int parseBluetooth() {
  String btMsg = "";
  while (BTSerial.available() > 0)
  {
    char c = BTSerial.read();
    btMsg += c;
    if (c == '\n')
    {
      btMsg = btMsg.substring(0, btMsg.length() - 1);
      Serial.println(btMsg);
      int val = btMsg.toInt();
      return val;
    }
  }
}

void setup() {
    Serial.begin(19200);
    BTSerial.begin(9600);
}

void loop() {
  if(BTSerial.read()=='F'){
    _launcher.driveAutoLoad();
  }

  else if(BTSerial.read() =='P') 
  {
    int phi = parseBluetooth();
    _launcher.moveAct(phi);
  }

  else if(BTSerial.read() =='T') 
  {
    theta = parseBluetooth();
  }

  else if(BTSerial.read() =='S')
  { 
    int num = parseBluetooth();
    _launcher.addSpeed(num);
  }

  _launcher.updateServo(theta);
}
