// Author: https://github.com/maximamolchanov  
#include "GenericServo.h"

// Constructor that sets up the initial servo configuration and constrains
// the angles to the angle limits if needed.
GenericServo::GenericServo(uint8_t pin, Servo servo, int neutralAngle,
                           int maxAngle, int minAngle, bool inverted)
    : pin(pin),
      servo(servo),
      neutralAngle(neutralAngle),
      maxAngle(maxAngle),
      minAngle(minAngle),
      latestAngle(neutralAngle),
      inverted(inverted) {

    if (this->neutralAngle < this->minAngle) {
        neutralAngle = minAngle;
    } else if (neutralAngle > maxAngle) {
        neutralAngle = maxAngle;
    }
}

// Attaches the servo to its pin and will set it to the neutral position.
void GenericServo::attach() {
    servo.attach(pin);
    writeAngle(neutralAngle);
}

// Detatches the servo from its pin
void GenericServo::detatch() {
    servo.detach();
}

// Writes the target to sero to the angle in degrees - constrained between
// minAngle and maxAngle.
void GenericServo::writeAngle(int angle) {
    if (angle < minAngle) {
      angle = minAngle;
    } 
    else if (angle > maxAngle) {
      angle = maxAngle;
    }

    latestAngle = angle;
    servo.write(angle);
}

// Writes the pulse width directly to the servo.
// Typically 1000 us is all the way one way (left), 
// 1500 us is center, and 2000 us is fully the other way (right).
void GenericServo::writeMicroseconds(int microseconds) {
    servo.writeMicroseconds(microseconds);
}

// Commands the servo using a supposed PID output from -1.0 to 1.0.
void GenericServo::command(float pidOutput) {
    if (pidOutput > 1.0f) {
        pidOutput = 1.0f;
    } else if (pidOutput < -1.0f) {
        pidOutput = -1.0f;
    }

    float angle = neutralAngle;

    if (pidOutput > 0.0f) {
        // Positve PID - moves away from neutral angle to maxAngle
        float rangePos = maxAngle - neutralAngle;
        angle = neutralAngle + pidOutput * rangePos;
    } else if (pidOutput < 0.0f) {
        // Negative PID - moves away from neutral angle to minAngle
        float rangeNeg = neutralAngle - minAngle;
        angle = neutralAngle + pidOutput * rangeNeg;
    }

    writeAngle(static_cast<int>(angle + 0.5f));
}

// Alters the current angle limits for the servo
void GenericServo::changeAngleLimits(int newMinAngle, int newMaxAngle) {
    minAngle = newMinAngle;
    maxAngle = newMaxAngle;

    if (neutralAngle < minAngle) neutralAngle = minAngle;
    if (neutralAngle > maxAngle) neutralAngle = maxAngle;

    if (latestAngle < minAngle) latestAngle = minAngle;
    if (latestAngle > maxAngle) latestAngle = maxAngle;
}

// Sets the the servo's current inverted state.
// Note, this does not directly affect the servo.
void GenericServo::setInverted(bool newState) {
    inverted = newState;
}

