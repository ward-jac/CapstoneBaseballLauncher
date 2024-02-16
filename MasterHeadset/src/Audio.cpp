#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include "../include/Audio.h"

Audio::Audio(int rx_pin, int tx_pin) {
    SoftwareSerial bluetoothSerial(rx_pin, tx_pin);
}

void Audio::play(String s) {
    if(s == "Face To Calibrate") {

    }
    else if(s == "Calibrated") {

    }
    else if(s == "Add 10 Speed") {

    }
    else if(s == "Changing Sensitivity") {

    }
}