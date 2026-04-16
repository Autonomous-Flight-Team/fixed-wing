#include "map_pins.h"

// Pins
int left_aileron_pin = 6;
int right_aileron_pin = 9;
int elevator_pin = 37;
int rudder_pin = 23;
int flap_pin = 29;

int throttle_pin = 2;
int control_surface_pin = 23; // Set control surfaces to neutral button push
int drone_release_pin = 11;

int esc_pin = 0;

// Servos
Servo left_aileron_servo;
Servo right_aileron_servo;
Servo elevator_servo;
Servo rudder_servo;
Servo flap_servo;

Servo drone_release_servo;
Servo ESC;

float throttle = 0.0f;

// Flap/release positions
int flaps_deployed_loc = 45;
int flaps_retracted_loc = 0;
int drone_release_close = 120;
int drone_release_loc = 80;

// Current locations
int aileron_loc = aileron_limit / 2;
int elevator_loc = elevator_limit / 2;
int rudder_loc = rudder_limit / 2;

// Rate controls
float aileron_rate = 0.5f;
float elevator_rate = 0.5f;
float rudder_rate = 0.5f;
float throttle_rate = 0.01f;

bool armed = false;
