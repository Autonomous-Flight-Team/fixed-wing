// Authors:
//  - Zakharii Kovalchuk github.com/Zakkoval

#include "dshot.h"

#define DSHOT_BIT_COUNT  16

// Motor index -> physical pin mapping.
// motorIndex = 0 --> MOTOR_LEFT_PIN
// motorIndex = 1 --> MOTOR_RIGHT_PIN
const int motorPins[NUM_MOTORS] = {MOTOR_LEFT_PIN, MOTOR_RIGHT_PIN};



// Current DShot command (0–2047) for each motor.
// Updated each loop based on PID output (delta-based)
uint16_t motorDshotValue[NUM_MOTORS] = { 0 };  // all start at 0 (stopped)

static uint16_t DShot_BuildPayload(uint16_t throttle, bool telemetryRequest);
static uint16_t DShot_ComputeChecksum(uint16_t payload12);
static uint16_t DShot_BuildPacket(uint16_t throttle, bool telemetryRequest);

// This is the hardware-specific backend placeholder.
// Later: this will use FlexPWM+DMA on Teensy.
static void DShot_SendPacketHardware(int motorIndex, uint16_t packet);


void DShot_Init()
{
    // TODO: 
    // 1) Configure motor pins as FlexPWM outputs;
    // 2) Configure FlexPWM period for DShot bit time;
    // 3) Set up DMA channel(-s) for each motor
}



void DShot_ApplyPidToSingleMotor(int motorIndex, float pidCmd){
    if(motorIndex < 0 || motorIndex >= NUM_MOTORS){ //make sure motorIndex is in range.
        return;
    }

// Clamp PID (pidCmd) to [-1.0, 1.0]
    if(pidCmd > 1.0f){
        pidCmd = 1.0f;
    }
    else if (pidCmd < -1.0f)
    {
        pidCmd = -1.0f;
    }

// Convert PID to delta in DShot units
    int delta = static_cast<int>(pidCmd * static_cast<float>(MAX_DELTA_VALUE));

// Update current throttle value
    int newValue = static_cast<int>(motorDshotValue[motorIndex]) + delta;


// 4) Enforce DShot throttle rules:
//    - <= 0     -> STOP command
//    - 1–47     -> reserved commands, avoid by snapping to MIN_DSHOT_VALUE
//    - > 2047   -> clamp to MAX
    if (newValue <= 0)
    {
        newValue = 0; //0 is explicit STOP command
    }
    else if (newValue < MIN_DSHOT_VALUE)
    {
        newValue = MIN_DSHOT_VALUE; // avoid reserved command zone (1-47)
    }
    else if (newValue > MAX_DSHOT_VALUE)
    {
        newValue = MAX_DSHOT_VALUE;
    }
    

// Store updated command
    motorDshotValue[motorIndex] = static_cast<uint16_t>(newValue);

// Transmit command to ESC
    DShot_SendFrame(motorIndex, motorDshotValue[motorIndex]);
}


// Build and send one DShot frame
void DShot_SendFrame(int motorIndex, uint16_t dshotValue){
    if (motorIndex < 0 || motorIndex >= NUM_MOTORS)
    {
        return;
    }
    
    //For now we do not request telemetry
    bool telemetryRequest = false;
    
    //Build the 16-bit DShot packet
    uint16_t packet = DShot_BuildPacket(dshotValue, telemetryRequest);
    
    // Hardware-specific send (FlexPWM + DMA later)
    DShot_SendPacketHardware(motorIndex, packet);
}



//Build 12-bit DShot data: 11-throttle value, 1-YES/NO telemetry--------------------------------------------------------------
static uint16_t DShot_BuildPayload(uint16_t throttle, bool telemetry){
    //make sure throttle is not above DShot's maximum
    if (throttle > MAX_DSHOT_VALUE)
    {
        throttle = MAX_DSHOT_VALUE;
    }
    
    uint16_t payload = static_cast<uint16_t>((throttle<<1) | (telemetry ? 1u : 0u));
    
    return payload; //12-bit value stored in 16-bit container
}



//Compute 4-bit checksum--------------------------------------------------------------------------------------------------------
static uint16_t DShot_ComputeChecksum(uint16_t payload12){
    uint16_t csum = 0;
    uint16_t csum_data = payload12;

    for (int i = 0; i < 3; ++i) {
        // Take lowest 4 bits
        uint16_t nibble = (csum_data & 0xF);
        csum ^= nibble;
        csum_data >>= 4;
    }

    csum &= 0xF; // Ensure we only keep 4 bits      
    return csum;    
}





//Assemble 16-bit DShot frame---------------------------------------------------------------------------------------------------------
static uint16_t DShot_BuildPacket(uint16_t throttle, bool telemetryRequest){
    
    //Build 12-bit payload (throttle + telemetry)
    uint16_t payload = DShot_BuildPayload(throttle, telemetryRequest);
    //Compute 4-bit checksum for this payload
    uint16_t csum4 = DShot_ComputeChecksum(payload);
    //Combine into one 16-bit packet
    uint16_t packet = static_cast<uint16_t>((payload<<4) | csum4);

    return packet;
}

// Until the Teensy 4.1 is available, this is intentionally empty.
static void DShot_SendPacketHardware(int motorIndex, uint16_t packet)
{
    (void)motorIndex;
    (void)packet;

    // TODO (with Teensy 4.1):
    //  1) Convert 'packet' (16 bits) into an array of 16 duty values:------------------------------------------------------------------------------------
    //       - 1-bit  -> "long HIGH, short LOW"-----------------------------------------------------------------------------------------------------------
    //       - 0-bit  -> "short HIGH, long LOW"-----------------------------------------------------------------------------------------------------------
    //  2) Use DMAChannel to feed those duty values into the appropriate----------------------------------------------------------------------------------
    //     FlexPWM VALx registers for the motor's PWM output----------------------------------------------------------------------------------------------
    //  3) Trigger the DMA transfer so that one PWM period = one DShot bit--------------------------------------------------------------------------------
    //  4) Optionally, use an interrupt or flag to know when the frame is done----------------------------------------------------------------------------
}
















//goes to ESC-----------------------------------------------------------------------------------------------------------------------