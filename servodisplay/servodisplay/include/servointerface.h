#ifndef SERVO_INTERFACE_H
#define SERVO_INTERFACE_H

#define LIBRARY_MIN_ANGLE 0
#define LIBRARY_MAX_ANGLE 180

// Need 'Servo' in platformio.ini under lib_deps

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoInterface
{
    public:
        ServoInterface();
        void setup(int pin, int range, int lowerPulseWidth, int upperPulseWidth); // lowerPulseWidth and upperPulseWidth are in microseconds
        void setAngle(int angle);
        void detach();
        void reset();
        String toString();
        bool isAttached();
        void incrementAngle(int increment); // increments the angle by increment (can be negative)
        void setPercentAngle(int percentage); // sets the angle to a percentage of the range (0-100)
        void setSerialPrint(bool set = false);

    private:
        Servo servo;
        int range; // 0 to how many degrees the servo is out of (180, 270, etc. ). The interface will scale the input to this range
        int angle;
        bool doSerialPrint = false;


};

#endif // SERVO_INTERFACE_H