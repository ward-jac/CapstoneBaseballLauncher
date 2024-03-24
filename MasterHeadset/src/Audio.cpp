#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include "../include/Audio.h"

Audio::Audio(int rx_pin, int tx_pin) {
    SoftwareSerial audioSerieal(rx_pin, tx_pin);
    myDFPlayer.volume(10);
    myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
    myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
}

void Audio::play(String s) {
    if(s == "Setting speed to 10") {
        myDFPlayer.playMp3Folder(1);
    }
    else if(s == "Setting speed to 20") {
        myDFPlayer.playMp3Folder(2);
    }
    else if(s == "Setting speed to 30") {
        myDFPlayer.playMp3Folder(3);
    }
    else if(s == "Setting speed to 40") {
        myDFPlayer.playMp3Folder(4);
    }
    else if(s == "Setting speed to 50") {
        myDFPlayer.playMp3Folder(5);
    }
    else if(s == "Setting speed to 60") {
        myDFPlayer.playMp3Folder(6);
    }
    else if(s == "Setting speed to 70") {
        myDFPlayer.playMp3Folder(7);
    }
    else if(s == "Setting speed to 80") {
        myDFPlayer.playMp3Folder(8);
    }
    else if(s == "Setting speed to 90") {
        myDFPlayer.playMp3Folder(9);
    }
    else if(s == "Setting speed to 100") {
        myDFPlayer.playMp3Folder(10);
    }
    else if(s == "Launcher locked") {
        myDFPlayer.playMp3Folder(11);
    }
    else if(s == "Launcher unlocked") {
        myDFPlayer.playMp3Folder(12);
    }
    else if(s == "Calibrated") {
        myDFPlayer.playMp3Folder(13);
    }
    else if(s == "Release to calibrate") {
        myDFPlayer.playMp3Folder(14);
    }
    else if (s=="Firing") {
        myDFPlayer.playMp3Folder(15);
    }
}