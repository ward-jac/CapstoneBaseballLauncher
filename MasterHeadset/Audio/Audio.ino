#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>
#include <Arduino.h>

SoftwareSerial audioSerial;
DFRobotDFPlayerMini myDFPlayer;

void setup() {
    audioSerial.begin(9600);
    myDFPlayer.volume(10);
    myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
    myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
}
/*   (val): Control name

    (1-10): Speed control
    - sets speed to val*10 speed
    11: Lock Launcher
    12: Unlock Launcher
    13: Sensor Calibrated
    14: Face Forward to Calibrate
    15: Fire
*/
void loop() {
    String msg = "";
    while(audioSerial.available() > 0) {
        char c = audioSerial.read();
        msg += c;
        if(c=='\n') {
            msg = msg.substring(0,msg.length()-1);
            int val = msg.toInt();
            msg = "";
            myDFPlayer.playMp3Folder(val);
        }
    }
}