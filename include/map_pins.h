#ifndef MAP_PINS_HEADER
#define MAP_PINS_HEADER

#include <Arduino.h>
#include <Servo.h>

// Pins
extern int left_aileron_pin;
extern int right_aileron_pin;
extern int elevator_pin;
extern int rudder_pin;
extern int right_flap_pin;
extern int left_flap_pin;
extern int throttle_pin;
extern int control_surface_pin;
extern int drone_release_pin;

// Servos
extern Servo left_aileron_servo;
extern Servo right_aileron_servo;
extern Servo elevator_servo;
extern Servo rudder_servo;
extern Servo right_flap_servo;
extern Servo left_flap_servo;
extern Servo drone_release_servo;

// Limits
constexpr float aileron_limit = 32.0f;
constexpr float elevator_limit = 40.0f;
constexpr float rudder_limit = 60.0f;
constexpr float throttle_limit = 1.0f;

constexpr float aileron_neutral = aileron_limit / 2;   // 16
constexpr float elevator_neutral = elevator_limit / 2; // 20
constexpr float rudder_neutral = rudder_limit / 2;     // 30

// Flap/release positions
extern int flaps_deployed_loc;
extern int flaps_retracted_loc;
extern int drone_release_loc;

// Current locations
extern int aileron_loc;
extern int elevator_loc;
extern int rudder_loc;

// Rate controls
extern float aileron_rate;
extern float elevator_rate;
extern float rudder_rate;
extern float throttle_rate;

#endif