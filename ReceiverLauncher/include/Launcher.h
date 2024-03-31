#ifndef Launcher_h
#define Launcher_h

class Launcher {
    public:
        Launcher();
        void moveAct(float phi);
        void updateServo(float theta);
        void setSensitivity(char c);

    private:
        void driveActuator(int Direction, int Speed);
        float mapFloat(long x, long in_min, long in_max, long out_min, long out_max);
        int act_pin;
        int act_RPWM;
        int act_LPWM;

        // Actuator state variables
        int actReading;               // the value read by the linear actuator potentiometer
        float actSpeed;               // speed of the linear actuator (value from 0-255)
        float strokeLength;           // length of linear actuator stroke
        int maxAnalogReading;         // max value that linear actuator is allowed to move to
        int minAnalogReading;         // min value that linear actuator is allowed to move to
        int actPos;                   // the set position that the linear actuator is being moved to

        // Servo state variables
        float servoSpeed;             // the scaled rotation of the servo (value from 0-1)
        float servoPos;               // the set position that the servo is being moved to
        int microPos;                 // the servo position in microseconds that it is moving to

        // to keep track of the current firing mode
        // 0 = fine
        // 1 = coarse
        int sensitivityMode;

        // the minimum angle needed to activate
        int sensitivity[];
};

#endif