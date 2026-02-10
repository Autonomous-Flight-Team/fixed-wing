#ifndef TYPES_H
#define TYPES_H

#include <MAVLink.h>

// TODO: Do we need an emergency landing mode?
typedef enum { 
    MANUAL = 0,
    MISSION_ABORT = 1,  // Communication failure fail-safe - need landing sequence

    // Autonomous Modes
    FULL_AUTONOMOUS = 2, // Future complete autonomous mode
    
    AUTO_CRUISE = 3,  // Specifying Autonomous modes will make mission profile easier to encapsulate
    AUTO_LAND = 4, // Future self-landing mechanism beyond MISSION_ABORT
    AUTO_TAKEOFF = 5,  // Future self-takeoff mode
    
} DroneMode;

// Mavlink relevant types
typedef enum {
    LINK_900MHZ = 0,
    LINK_24GHZ  = 1
} MavlinkLink_t;

typedef struct {
    MavlinkLink_t link;
    mavlink_message_t msg;
} MavlinkRxPacket_t;


typedef struct {
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

typedef struct {
    // Linear Position
    double x, y, z;
    // Linear Velocity
    double u, v, w;
    // Angular Velocity
    double phi, theta, psi;
    // Angular Acceleration
    double p, q, r;
} StateVector_t;

typedef struct {
  double somethingneedstobehere;
    //TODO
} ControlOutput_t;

typedef struct {
    bool on;
} BlinkState_t;


#endif
