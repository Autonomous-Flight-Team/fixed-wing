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

int map_throttle(int cont_throttle)
{ // recives 0 -1000 -> needs to be maped to 0 to 100
    return cont_throttle / 10;
}

void servos_to_neutral()
{
    left_aileron_servo.write(aileron_neutral);
    right_aileron_servo.write(aileron_neutral);
    elevator_servo.write(elevator_neutral);
    rudder_servo.write(rudder_neutral);
    flap_servo.write(flaps_retracted_loc);
    drone_release_servo.write(drone_release_close);
}

void set_state(mavlink_manual_control_t *controllerData, SetServoStates_t *servoStates)
{
    // set deadzones
    if (abs(controllerData->z) - DEADZONE < 0)
    {
        controllerData->z = 0;
    }
    if (abs(controllerData->x) - DEADZONE < 0)
    {
        controllerData->x = 0;
    }
    if (abs(controllerData->y) - DEADZONE < 0)
    {
        controllerData->y = 0;
    }
    if (abs(controllerData->r) - DEADZONE < 0)
    {
        controllerData->r = 0;
    }

    servoStates->set_throttle = map_throttle(controllerData->z); // passes 0 -1000 -> needs to be maped to 0 to 100
    servoStates->set_elevator = clamp(servoStates->set_elevator + controllerData->x * elevator_rate, (float)0, elevator_limit);
    servoStates->set_aileron = clamp(servoStates->set_aileron + controllerData->y * aileron_rate, (float)0, aileron_limit);
    servoStates->set_rudder = clamp(servoStates->set_rudder + controllerData->r * rudder_rate, (float)0, rudder_limit);

    bool buttonState[16];
    for (int i = 0; i < 16; i++)
    {
        buttonState[i] = (controllerData->buttons >> i) & 1;
        // Serial.print(buttonState[i]);
    }
    // Serial.println();

    if (buttonState[2])
    {
        servoStates->release_drone = true;
    }
    else
    {
        servoStates->release_drone = false;
    }

    if (buttonState[3])
    {
        servoStates->set_aileron = aileron_neutral;
        servoStates->set_rudder = rudder_neutral;
        servoStates->set_elevator = elevator_neutral;
    }

    if (buttonState[11])
    {
        armed = true;
    }

    // flaps
    if (true)
    {
        servoStates->flaps = flaps_retracted_loc;
    }
}

void set_servos(const SetServoStates_t *servoStates) // currently doesn't set throttle
{

    elevator_servo.write(servoStates->set_elevator);
    rudder_servo.write(servoStates->set_rudder);
    left_aileron_servo.write(aileron_limit - servoStates->set_aileron);
    right_aileron_servo.write(servoStates->set_aileron);
    flap_servo.write(servoStates->flaps);

    if (servoStates->release_drone)
    {
        drone_release_servo.write(drone_release_loc);
        Serial.println("Release");
    }
    else
    {
        drone_release_servo.write(drone_release_close);
        Serial.println("Close");
    }

    if (armed)
    {
        ESC.writeMicroseconds(servoStates->set_throttle);
    }
    else
    {
        ESC.writeMicroseconds(1000);
    }
}

// Tasks
void updateStatesTask(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(50);

    for (;;)
    {
        mavlink_manual_control_t localController;
        if (xSemaphoreTake(mavlinkDataMutex, portMAX_DELAY))
        {
            localController = manual_control_data;
            xSemaphoreGive(mavlinkDataMutex);
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