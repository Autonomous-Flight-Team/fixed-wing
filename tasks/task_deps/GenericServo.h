// Author: https://github.com/maximamolchanov
#pragma once

#include <Arduino.h>
#include <Servo.h>

// Generic servo class that utilizes the Arduino Servo library
// and encompasses the basic information and actions a servo
// for a UAV would need to perform.
class GenericServo {
    protected:
        uint8_t pin; //PWM pin the servo is attached to
        Servo servo; // Underlying servo object
        int neutralAngle; // Center angle for servo
        int maxAngle; // Maximum angle for servo
        int minAngle; // Minimum angle for servo
        int latestAngle; // Most recent angle for debugging purposes
        bool inverted; // State for whether the servo should act inverted
    public:
        //Constrtuctor for the protected fields.
        // If not given maxAngle, minAngle, and inverted will assume
        // the defaulted values below.
        GenericServo(uint8_t pin, Servo servo, int neutralAngle,
                    int maxAngle = 0, int minAngle = 180, bool inverted = false);
        
        void attach();
        void detatch();

        void writeAngle(int angle);
        void writeMicroseconds(int microseconds);

        void command(float pidOutput);
        
        int getMinAngle() const { return minAngle; }
        int getMaxAngle() const { return maxAngle; }
        int getLatestAngle() const { return latestAngle; }

        void changeAngleLimits (int minAngle, int maxAngle);
        void setInverted (bool newState);
};
