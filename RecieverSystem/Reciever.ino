#include <SoftwareSerial.h>

const int RX_PIN;
const int TX_PIN;
SoftwareSerial bluetoothSerial(RX_PIN, TX_PIN);

void setup() {
    // Open serial communications and wait for port to open:
    Serial.begin(19200);
    // set the data rate for the SoftwareSerial port
    bluetoothSerial.begin(9600);
}

void loop() {
  bluetoothSerial.read();
}
