#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>
//#include <Arduino.h>
#include "../include/Audio.h"

Audio::Audio(int rx_pin, int tx_pin) {
    SoftwareSerial bluetoothSerial(rx_pin, tx_pin);
}

void Audio::play(char s) {
    switch(s) {
        case 's':
        break;
        case 'h':
        break;
    }
}