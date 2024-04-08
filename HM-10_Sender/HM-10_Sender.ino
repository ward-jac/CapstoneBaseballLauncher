// MAC Address: B0D2786E2DBF
// AT+ROLE1 to set the sender module to Central mode
// AT+DISC? to scan for receiver module
// AT+CONN[ADDR] to connect, where [ADDR] is the MAC address of the receiver
// AT+IMME0 to set the sender module to auto connect on start up

// for BT communication
#define BTSerial Serial1

char c = ' ';
boolean NL = true;

void setup() {
  Serial.begin(9600);
  Serial.print("Sketch:   ");
  Serial.println(__FILE__);
  Serial.print("Uploaded: ");
  Serial.println(__DATE__);
  Serial.println(" ");

  BTSerial.begin(9600);
  Serial.println("BTSerial started at 9600");
}

void loop() {
  // Read from the Bluetooth module and send to the Arduino Serial Monitor
  if (BTSerial.available()) {
    c = BTSerial.read();
    Serial.write(c);
  }


  // Read from the Serial Monitor and send to the Bluetooth module
  if (Serial.available()) {
    c = Serial.read();

    // do not send line end characters to the HM-10
    if (c != 10 & c != 13) {
      BTSerial.write(c);
    }

    // Echo the user input to the main window.
    // If there is a new line print the ">" character.
    if (NL) {
      Serial.print("\r\n>");
      NL = false;
    }
    Serial.write(c);
    if (c == 10) { NL = true; }
  }
}