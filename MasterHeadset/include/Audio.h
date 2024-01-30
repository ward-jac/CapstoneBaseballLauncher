#ifndef Audio_h
#define Audio_h
#include <DFRobotDFPlayerMini.h>

class Audio {
public:
    Audio(int rx_pin, int tx_pin);
    void play(char s);

private:
    int _pin;
    DFRobotDFPlayerMini myDFPlayer;
};
#endif