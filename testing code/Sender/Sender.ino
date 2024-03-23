// required for IMU calculations
#include "Adafruit_BNO08x_RVC.h"

// for sign operations
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

// the sample rate delay and IMU
#define SAMPLE_RATE_DELAY (500)
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();

// to obtian the IMU readings
BNO08x_RVC_Data heading;

//  Pins
//  BT VCC to Arduino 5V out.
//  BT GND to GND
//  Arduino 48 (Mega) RX -> BT TX no need voltage divider
//  Arduino 46 (Mega) TX -> BT RX through a voltage divider (5v to 3.3v)

// https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
#include <AltSoftSerial.h>
AltSoftSerial sender;

// the curent angles at the time of calibration
float shift_theta = 0.0;
float shift_phi = 0.0;
int updateCount = 0;

// to keep track of the changing angles
float theta = 0.0;
float phi = 0.0;

// the maximum values of theta and phi
int maxAngle = 30;

// to deal with gimbal lock
int dangerZone = 60;

// to keep track of the current firing mode
// 0 = fine
// 1 = coarse
int sensitivityMode = 1;

// the minimum angle needed to activate
int sensitivity[] = { 7, 15 };

// maps input to output positions
float mapFloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

// returns the shifted value of theta after calibration
float getTheta(BNO08x_RVC_Data heading) {
  return heading.yaw + (-1.0 * shift_theta);
}

// returns the shifted value of phi after calibration
float getPhi(BNO08x_RVC_Data heading) {
  return heading.pitch + (-1.0 * shift_phi);
}

// updates the euler shift angles (theta and phi)
void updateShifts() {
  if (!rvc.read(&heading)) {
    return;
  }

  // euler x = theta
  shift_theta = heading.yaw;

  // euler z = phi
  shift_phi = heading.pitch;
}

void setup() {
  // start serial monitor communication at 9600 baud rate
  Serial.begin(9600);
  Serial.print("Sketch:   ");
  Serial.println(__FILE__);
  Serial.print("Uploaded: ");
  Serial.println(__DATE__);
  Serial.println(" ");

  // start sender BT at 9600 baud rate
  sender.begin(9600);
  Serial.println("Sender started at 9600");

  // crucial
  delay(1000);
}

void loop() {
  // zero the IMU at the very beginning of the program
  if (updateCount < 1) {
    updateShifts();
    Serial.println("Calibrated\n");
    updateCount++;
  }

  if (!rvc.read(&heading)) {
    Serial.println("Error reading the IMU.");
  }

  // obtain theta and phi
  theta = getTheta(heading);
  phi = getPhi(heading);

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

  // delay for the designated sample rate delay
  delay(SAMPLE_RATE_DELAY);
}