// Authors
// Marie Andken
#include <FreeRTOS.h>
#include "map_pins.h"
#include "task.h"

void servos_to_neutral();

bool initialize_servo_control()
{
    // Set all digital pins to OUTPUT
    pinMode(left_aileron_pin, OUTPUT);
    pinMode(right_aileron_pin, OUTPUT);
    pinMode(elevator_pin, OUTPUT);
    pinMode(rudder_pin, OUTPUT);

    pinMode(throttle_pin, OUTPUT);
    pinMode(right_flap_pin, OUTPUT);
    pinMode(left_flap_pin, OUTPUT);

    pinMode(control_surface_pin, OUTPUT);
    pinMode(drone_release_pin, OUTPUT);

    // Attach servos to their pins
    left_aileron_servo.attach(left_aileron_pin);
    right_aileron_servo.attach(right_aileron_pin);
    elevator_servo.attach(elevator_pin);
    rudder_servo.attach(rudder_pin);
    left_flap_servo.attach(left_flap_pin);
    right_flap_servo.attach(right_flap_pin);
    drone_release_servo.attach(drone_release_pin);

    servos_to_neutral();

    return true;
}

bool initialize_manual_control()
{
    return initialize_servo_control();
}
