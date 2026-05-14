#ifndef TYPES_H
#define TYPES_H

#include <MAVLink.h>
#include "FreeRTOS.h" // 1. Base definitions (TickType_t lives here)

// TODO: Do we need an emergency landing mode?
typedef enum
{
    MANUAL = 0,
    MISSION_ABORT = 1, // Communication failure fail-safe - need landing sequence

    // Autonomous Modes
    FULL_AUTONOMOUS = 2, // Future complete autonomous mode

    AUTO_CRUISE = 3,  // Specifying Autonomous modes will make mission profile easier to encapsulate
    AUTO_LAND = 4,    // Future self-landing mechanism beyond MISSION_ABORT
    AUTO_TAKEOFF = 5, // Future self-takeoff mode

} DroneMode;

// Flaps positions
enum FlapsPosition
{
    FLAPS_UP = 0,
    FLAPS_MID = 1,
    FLAPS_DOWN = 2
};

extern FlapsPosition mavlinkFlapsPosition;

// Mavlink relevant types
typedef enum
{
    LINK_900MHZ = 0,
    LINK_24GHZ = 1
} MavlinkLink_t;

typedef struct
{
    MavlinkLink_t link;
    mavlink_message_t msg;
} MavlinkRxPacket_t;

template <typename T>
struct Log
{
    T data;
    // Default constructor: handles cases where no value is provided
    Log() : data() {}

    // Parameterized constructor: takes a value and moves it into 'data'
    Log(T value) : data(value) {}

    TickType_t timestamp;
};

typedef struct
{
    // Linear Acceleration
    float ax, ay, az;
    // Rotational Velocity
    float gx, gy, gz;
} IMUData_t;

typedef struct
{
    // Barometer
    float altitude;
    float pressure;
    float temp;
} BaroData_t;

typedef struct
{
    // GPS
    // Latitude
    double lat;
    // Longitude
    double lon;
    float gps_altitude;
    // Linear Velocity
    float vs;
} GPSData_t;

typedef struct {} PitotData_t;


typedef struct
{
    // IMU
    // Linear Acceleration
    float ax, ay, az;
    // Rotational Velocity
    float gx, gy, gz;
    // Barometer
    float altitude;
    float pressure;
    float temp;
    // GPS
    // Latitude
    double lat;
    // Longitude
    double lon;
    float gps_altitude;
    // Linear Velocity
    float vs;

    // Sensor Data Size:
} SensorData_t;

typedef struct
{
    // Linear Position
    double x, y, z;
    // Linear Velocity
    double u, v, w;
    // Angular Velocity
    double phi, theta, psi;
    // Angular Acceleration
    double p, q, r;
} StateVector_t;

typedef struct
{
    StateVector_t state;
    SensorData_t sensor;
} GSATxPacket_t;

typedef struct
{
    // Normalized commands in [-1, 1] except throttle in [0, 1].
    float aileron;
    float elevator;
    float rudder;
    float throttle;
    // Servo-equivalent PWM outputs for easy hardware mapping.
    uint16_t aileron_pwm;
    uint16_t elevator_pwm;
    uint16_t rudder_pwm;
    uint16_t throttle_pwm;
    // True when a fresh manual input frame has been received recently.
    bool link_ok;
} ControlOutput_t;

typedef struct
{
    bool on;
} BlinkState_t;

// Stores the state of the controller that has been communicated to the fixed wing
typedef struct
{

    // Triggers -control throttle
    float left_trig, right_trig;
    // Bumpers -control yaw
    float left_bump, right_bump;
    // Left joy stick -controls pitch
    float pitch_joystick;
    // Right joy stick -controls roll
    float roll_joystick;

    // Buttons
    // Y deploy flaps
    bool Y;
    // A retract flaps
    bool A;
    // X release drone
    bool X;
    // B set control surfaces to neutral
    bool B;

} Controller_t;

// Holds Servo states
typedef struct
{
    float set_throttle;
    float set_aileron, set_elevator, set_rudder;
    bool flaps;
    bool release_drone;
} SetServoStates_t;

// New struct for putting logs into sdcard in binary - gets rid of packing that can confuse binary
// decoder - efficiency and separation between ram and persistence memory
#pragma pack(push, 1)
template <typename T>
struct SD_Log_t
{
    T data;
    // Default constructor: handles cases where no value is provided
    SD_Log_t() : data() {}

    // Parameterized constructor: takes a value and moves it into 'data'
    SD_Log_t(T value) : data(value) {}

    TickType_t timestamp;
};
#pragma pack(pop) // End: Return to normal

#endif
