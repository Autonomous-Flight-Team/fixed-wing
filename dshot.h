#ifndef DSHOT_H
#define DSHOT_H

#include <cstdint>

using std::uint16_t;

#define NUM_MOTORS  2

//TODO: replace with real (FlexPWM-capable) pin
#define MOTOR_LEFT_PIN  5   
#define MOTOR_RIGHT_PIN  6


extern const int motorPins[NUM_MOTORS];

extern uint16_t motorDshotValue[NUM_MOTORS];


// 0      = STOP command
// 1-47  = special commands (not used as throttle)
// 48-2047 = usable throttle range
#define MIN_DSHOT_VALUE  48
#define MAX_DSHOT_VALUE 2047

//MAX_DELTA_DSHOT controls how quickly the DShot value is allowed to change per MotorTask update.
//Example: if MAX_DELTA_DSHOT = 50, then the DShot value can change at most +/- 50 per loop.
//This helps to avoid super aggressive jumps.
#define MAX_DELTA_VALUE 50 //TODO: tune after testing


// Call this once at startup (from MotorTask before the loop).
// Later: will configure FlexPWM, DMA, etc.
// For now, it's just a stub/placeholder.
void DShot_Init();


// Inside, this will:
// 1) clamp pidCmd [-1.0, 1.0]
// 2) convert pidCmd to delta in DShot units
// 3) update and clamp motorDshotValue[motorIndex]
// 4) call DShot_SendFrame(...) to actually send it
void DShot_ApplyPidToSingleMotor(int motorIndex, float pidCmd);


// Sends one DShot frame to ESC for given motor index.
// For now it's just a stub (no real bit timing yet).
void DShot_SendFrame(int motorIndex, uint16_t dshot_Value);


#endif