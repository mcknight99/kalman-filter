#include "servointerface.h"
#include <iostream>

ServoInterface::ServoInterface()
{
    angle = 0;
}

void ServoInterface::setup(int pin, int range, int lowerPulseWidth, int upperPulseWidth)
{
    servo.attach(pin, lowerPulseWidth, upperPulseWidth);
    this->range = range;
    setAngle(0);
    if (doSerialPrint)
    {
        Serial.printf("Servo attached to pin %d with range %d\n", pin, range);
    }
}

void ServoInterface::setAngle(int angle)
{
    this->angle = angle;
    servo.write(round(angle * LIBRARY_MAX_ANGLE / range)); // ESP32Servo has a range 0-180 but can be set to other maximums through scaling
    if (doSerialPrint)
    {
        Serial.printf("Servo angle set to %d degrees\n", angle);
    }
}

void ServoInterface::detach()
{
    servo.detach();
    if (doSerialPrint)
    {
        Serial.println("Servo detached");
    }
}

void ServoInterface::reset()
{
    servo.write(0);
    if (doSerialPrint)
    {
        Serial.println("Servo reset");
    }
}

String ServoInterface::toString()
{
    return "Servo with range " + String(range) + " is at angle " + String(angle);
}

bool ServoInterface::isAttached()
{
    return servo.attached();
}

void ServoInterface::incrementAngle(int increment)
{
    angle += increment;
    if (angle < 0)
    {
        angle = 0;
    }
    else if (angle > range)
    {
        angle = range;
    }
    setAngle(angle);
    if (doSerialPrint)
    {
        Serial.printf("Servo angle incremented by %d\n", increment);
    }
}

void ServoInterface::setPercentAngle(int percentage)
{
    angle = round(range * percentage / 100);
    setAngle(angle);
    if (doSerialPrint)
    {
        Serial.printf("Servo angle set to %d percent\n", percentage);
    }
}

void ServoInterface::setSerialPrint(bool set)
{
    doSerialPrint = set;
}
