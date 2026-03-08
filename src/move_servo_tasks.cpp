// Authors
// Marie Andken
#include <FreeRTOS.h>

#include "map_pins.h"
#include "tasks.h"
#include "types.h"

template <typename T>

constexpr T clamp(T v, T lo, T hi)
{
    return std::min(std::max(v, lo), hi);
}

void servos_to_neutral()
{
    left_aileron_servo.write(aileron_neutral);
    right_aileron_servo.write(aileron_neutral);
    elevator_servo.write(elevator_neutral);
    rudder_servo.write(rudder_neutral);
    right_flap_servo.write(flaps_retracted_loc);
    left_flap_servo.write(flaps_retracted_loc);
    drone_release_servo.write(0);
}

void set_state(Controller_t *controllerData, SetServoStates_t *servoStates)
{
    servoStates->set_throttle = clamp((servoStates->set_throttle + (controllerData->left_trig * throttle_rate) - (controllerData->right_trig * throttle_rate)), (float)0, throttle_limit);
    servoStates->set_elevator = clamp(servoStates->set_elevator + controllerData->pitch_joystick * elevator_rate, (float)0, elevator_limit);
    servoStates->set_aileron = clamp(servoStates->set_aileron + controllerData->roll_joystick * aileron_rate, (float)0, aileron_limit);
    servoStates->set_rudder = clamp(servoStates->set_rudder + (controllerData->left_bump * -rudder_rate) + (controllerData->right_bump * rudder_rate), (float)0, rudder_limit);
    // Serial.print("Elevator: ");
    // Serial.println(servoStates->set_elevator);
    //  Serial.print(" Rudder:");
    //  Serial.print(servoStates->set_rudder);
    //  Serial.print(" Aileron:");
    //  Serial.println(servoStates->set_aileron);

    if (controllerData->Y && !(servoStates->flaps))
    {
        servoStates->flaps = true;
    }
    else if (controllerData->A && servoStates->flaps)
    {
        servoStates->flaps = false;
    }

    if (controllerData->X)
    {
        servoStates->release_drone = true;
    }

    if (controllerData->B)
    {
        servoStates->set_aileron = aileron_neutral;
        servoStates->set_rudder = rudder_neutral;
        servoStates->set_elevator = elevator_neutral;
    }
}

void set_servos(const SetServoStates_t *servoStates) // currently doesn't set throttle
{
    // Serial.print("Elevator:");
    // Serial.print(servoStates->set_elevator);
    // Serial.print(" Rudder:");
    // Serial.print(servoStates->set_rudder);
    // Serial.println(" Aileron:");
    // Serial.print(servoStates->set_aileron);

    elevator_servo.write(servoStates->set_elevator);
    rudder_servo.write(servoStates->set_rudder);
    left_aileron_servo.write(servoStates->set_aileron);
    right_aileron_servo.write(aileron_limit - servoStates->set_aileron);

    if (servoStates->flaps)
    {
        left_flap_servo.write(flaps_deployed_loc);
        right_flap_servo.write(flaps_deployed_loc);
    }
    else
    {
        left_flap_servo.write(flaps_retracted_loc);
        right_flap_servo.write(flaps_retracted_loc);
    }

    if (servoStates->release_drone)
    {
        drone_release_servo.write(drone_release_loc);
    }
}

// Tasks
void updateStatesTask(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(50);

    for (;;)
    {
        Controller_t localController;
        if (xSemaphoreTake(controllerMutex, portMAX_DELAY))
        {
            localController = controllerData;
            xSemaphoreGive(controllerMutex);
        }
        if (xSemaphoreTake(stateMutex, portMAX_DELAY))
        {
            set_state(&localController, &servoStateData);
            xSemaphoreGive(stateMutex);
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}

// Run as often as possible //But has lowest priority
void writeServoTask(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(20);
    for (;;)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY))
        {
            set_servos(&servoStateData);

            xSemaphoreGive(stateMutex);
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}