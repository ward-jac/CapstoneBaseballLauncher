// required for IMU calculations
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// for sign operations
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

// the sample rate delay and IMU
#define BNO055_SAMPLERATE_DELAY_MS (500)
Adafruit_BNO055 myIMU = Adafruit_BNO055(55);

//  Pins
//  BT VCC to Arduino 5V out.
//  BT GND to GND
//  Arduino 48 (Mega) RX -> BT TX no need voltage divider
//  Arduino 46 (Mega) TX -> BT RX through a voltage divider (5v to 3.3v)

// https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
#include <AltSoftSerial.h>
AltSoftSerial receiver;

// start of text char
char stx = 2;

// end of text char
char etx = 3;

// to store the received data
String data = "";

char c = ' ';
float theta = 0.0;
float phi = 0.0;

// obtain the first number in the string (theta)
float stringToTheta(String str) {
  return str.substring(0, str.indexOf(" ")).toFloat();
}

// obtain the second number in the string (phi)
float stringToPhi(String str) {
  return str.substring(str.indexOf(" ") + 1, str.indexOf(etx)).toFloat();
}

// safely read and process a character from BT
char readSafe() {
  // the character to read
  char ch = -1;

  // read until we have a valid read
  while (!isValidChar(ch)) {
    // read while there is information to read
    while (receiver.available() > 0) {
      ch = char(receiver.read());
    }
  }

  // return the valid char
  return ch;
}

// returns whether or not the given char is valid
bool isValidChar(char c) {
  // a valid char is one of [stx (2), etx (3), space (32), period (46), number (48-57)]
  char validChars[14] = { 2, 3, 32, 46, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57 };

  // assume the character is invalid
  bool isValid = false;

  // check the char against the list of valid chars
  for (int i = 0; i < 14; i++) {
    isValid = isValid || (c == validChars[i]);
  }

  return isValid;
}

void setup() {
  // start serial monitor communication at 9600 baud rate
  Serial.begin(9600);
  Serial.print("Sketch:   ");
  Serial.println(__FILE__);
  Serial.print("Uploaded: ");
  Serial.println(__DATE__);
  Serial.println(" ");

  // start receiver BT at 9600 baud rate
  receiver.begin(9600);
  Serial.println("Receiver started at 9600");
}

void loop() {
  // read until we reach the beginning of the data string
  while (c != 2) {
    c = readSafe();
  }

  // read the next char (the first char of interest in the data string)
  c = readSafe();

  // continue reading and constructing the data string until etx or no more chars to read
  while (c != 3) {
    // append the most recent char to the data string
    data += String(c);

    // read the next char
    c = readSafe();
  }

  // check to see if the entire string was received (last char is etx)
  if (c == 3) {
    // update theta and phi and reset the data string
    theta = stringToTheta(data);
    phi = stringToPhi(data);

    // for (int i = 0; i < data.length(); i++) {
    //   Serial.print(data.charAt(i));
    // }

    // Serial.println("");

    data = "";

    Serial.println("Theta: " + String(theta));
    Serial.println("Phi: " + String(phi));
  }
}