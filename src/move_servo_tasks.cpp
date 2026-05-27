// Authors
// Marie Andken
#include <FreeRTOS.h>
#include <Arduino.h>

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

// MIGHT NEED FOR SIM TO WORK
// namespace
// {
// float NormalizeManualAxis(int16_t axis)
// {
//     return clamp(static_cast<float>(axis) / 1000.0f, -1.0f, 1.0f);
// }

// float NormalizeManualThrottle(int16_t throttle)
// {
//     return clamp(static_cast<float>(throttle) / 1000.0f, 0.0f, 1.0f);
// }
// } // namespace

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

    servoStates->set_throttle = map_throttle(controllerData->z) + 1000; // passes 0 -1000 -> needs to be maped to 0 to 100

    servoStates->set_elevator = clamp(servoStates->set_elevator + controllerData->x * elevator_rate, (float)0, elevator_limit);
    servoStates->set_aileron = clamp(servoStates->set_aileron + controllerData->y * aileron_rate, (float)0, aileron_limit);
    servoStates->set_rudder = clamp(servoStates->set_rudder + controllerData->r * rudder_rate, (float)0, rudder_limit);

    bool buttonState[16];
    for (int i = 0; i < 16; i++)
    {
        buttonState[i] = (controllerData->buttons >> i) & 1;
        Serial.print(buttonState[i]);
    }
    Serial.println();

    if (buttonState[5])
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

    if (buttonState[0])
    {
        armed = true;
    }
    else
    {
        armed = false;
    }

    // flaps

    if (buttonState[2])
    {
        servoStates->set_flaps = flaps_retracted_loc;
    }

    else if (buttonState[4])
    {
        servoStates->set_flaps = flaps_deployed_loc;
    }

    else
    {
        servoStates->set_flaps = flaps_mid;
    }
    // i thought i needed this for simulation
    // const float throttleInput = NormalizeManualThrottle(controllerData->z);
    // const float elevatorInput = NormalizeManualAxis(controllerData->x);
    // const float aileronInput = NormalizeManualAxis(controllerData->y);
    // const float rudderInput = NormalizeManualAxis(controllerData->r);
    /*
    if (mavlinkVehicleArmed == true)
    {
        servoStates->set_throttle =
            clamp(servoStates->set_throttle + (throttleInput * throttle_rate), 0.0f, throttle_limit);
    }
    else
    {
        servoStates->set_throttle = 0;
    }
    servoStates->set_elevator =
        clamp(servoStates->set_elevator + (elevatorInput * elevator_rate), 0.0f, elevator_limit);
    servoStates->set_aileron =
        clamp(servoStates->set_aileron + (aileronInput * aileron_rate), 0.0f, aileron_limit);
    servoStates->set_rudder =
        clamp(servoStates->set_rudder + (rudderInput * rudder_rate), 0.0f, rudder_limit);*/
  
  
    // Serial.print("Elevator: ");
    // Serial.println(servoStates->set_elevator);
    // Serial.print(" Rudder:");
    // Serial.print(servoStates->set_rudder);
    // Serial.print(" Aileron:");
    // Serial.println(servoStates->set_aileron);

    // if (controllerData->Y && !(servoStates->flaps))
    // {
    //     servoStates->flaps = true;
    // }
    // else if (controllerData->A && servoStates->flaps)
    // {
    //     servoStates->flaps = false;
    // }

    // if (controllerData->X)
    // {
    //     servoStates->release_drone = true;
    // }

    // if (controllerData->B)
    // {
    //     servoStates->set_aileron = aileron_neutral;
    //     servoStates->set_rudder = rudder_neutral;
    //     servoStates->set_elevator = elevator_neutral;
    // }
}

void set_servos(const SetServoStates_t *servoStates) // currently doesn't set throttle
{

    elevator_servo.write(servoStates->set_elevator);
    rudder_servo.write(servoStates->set_rudder);
    left_aileron_servo.write(aileron_limit - servoStates->set_aileron);
    right_aileron_servo.write(servoStates->set_aileron);
    flap_servo.write(servoStates->set_flaps);

    if (servoStates->release_drone)
    {
        drone_release_servo.write(drone_release_loc);
        // Serial.println("Release");
    }
    else
    {
        drone_release_servo.write(drone_release_close);
        // Serial.println("Close");
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
    uint32_t lastPilotLatencyPrintMs = 0U;
    for (;;)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY))
        {
            set_servos(&servoStateData);

            xSemaphoreGive(stateMutex);
        }

        const uint32_t nowUs = micros();
        const uint32_t lastInputUs = mavlinkLastManualInputUs;
        const uint32_t oneWayUs = mavlinkLatencyOneWayAvgUs;
        const uint32_t fcAgeUs = (lastInputUs == 0U) ? 0U : (nowUs - lastInputUs);
        mavlinkPilotLatencyEstimateUs = oneWayUs + fcAgeUs;

        const uint32_t nowMs = millis();
        if ((nowMs - lastPilotLatencyPrintMs) >= 250U && lastInputUs != 0U && oneWayUs != 0U)
        {
            lastPilotLatencyPrintMs = nowMs;
            Serial.print("[MAVLINK][PILOT_LATENCY_ESTIMATE] pilot_end_to_end_estimate_ms=");
            Serial.print(mavlinkPilotLatencyEstimateUs / 1000U);
            Serial.print(" radio_link_one_way_avg_ms=");
            Serial.print(oneWayUs / 1000U);
            Serial.print(" flight_controller_input_age_ms=");
            Serial.println(fcAgeUs / 1000U);
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}

// Testing task

void printControllerTask(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(5000);
    for (;;)
    {
        mavlink_manual_control_t localController;
        if (xSemaphoreTake(mavlinkDataMutex, portMAX_DELAY))
        {
            localController = manual_control_data;
            xSemaphoreGive(mavlinkDataMutex);
        }

        char buf[16];

        Serial.println("=== MANUAL CONTROL DATA ===");
        Serial.print("target:             ");
        Serial.println(localController.target);
        Serial.print("x (pitch):          ");
        Serial.println(localController.x);
        Serial.print("y (roll):           ");
        Serial.println(localController.y);
        Serial.print("z (thrust):         ");
        Serial.println(localController.z);
        Serial.print("r (yaw):            ");
        Serial.println(localController.r);

        sprintf(buf, "0x%04X", localController.buttons);
        Serial.print("buttons  (1-16):    ");
        Serial.println(buf);

        sprintf(buf, "0x%04X", localController.buttons2);
        Serial.print("buttons2 (17-32):   ");
        Serial.println(buf);

        sprintf(buf, "0x%02X", localController.enabled_extensions);
        Serial.print("enabled_extensions: ");
        Serial.println(buf);

        Serial.print("s (pitch-ext):      ");
        Serial.println(localController.s);
        Serial.print("t (roll-ext):       ");
        Serial.println(localController.t);
        Serial.print("aux1:               ");
        Serial.println(localController.aux1);
        Serial.print("aux2:               ");
        Serial.println(localController.aux2);
        Serial.print("aux3:               ");
        Serial.println(localController.aux3);
        Serial.print("aux4:               ");
        Serial.println(localController.aux4);
        Serial.print("aux5:               ");
        Serial.println(localController.aux5);
        Serial.print("aux6:               ");
        Serial.println(localController.aux6);

        Serial.println("--- Buttons (1-16) ---");
        for (int i = 0; i < 16; i++)
        {
            Serial.print("  Button ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.println((localController.buttons >> i) & 1);
        }

        Serial.println("--- Buttons2 (17-32) ---");
        for (int i = 0; i < 16; i++)
        {
            Serial.print("  Button ");
            Serial.print(i + 17);
            Serial.print(": ");
            Serial.println((localController.buttons2 >> i) & 1);
        }
        Serial.println("==================");
        Serial.println("\n=== CHANGED VALUES ===");
        static mavlink_manual_control_t lastController = {0};

        for (int i = 0; i < 16; i++)
        {
            if (((localController.buttons >> i) & 1) != ((lastController.buttons >> i) & 1))
            {
                Serial.print("Button ");
                Serial.print(i + 1);
                Serial.print(" changed to ");
                Serial.println((localController.buttons >> i) & 1);
            }
        }

        if (localController.aux1 != lastController.aux1)
            Serial.println("aux1 changed: " + String(localController.aux1));
        if (localController.aux2 != lastController.aux2)
            Serial.println("aux2 changed: " + String(localController.aux2));
        if (localController.aux3 != lastController.aux3)
            Serial.println("aux3 changed: " + String(localController.aux3));
        if (localController.aux4 != lastController.aux4)
            Serial.println("aux4 changed: " + String(localController.aux4));
        if (localController.aux5 != lastController.aux5)
            Serial.println("aux5 changed: " + String(localController.aux5));
        if (localController.aux6 != lastController.aux6)
            Serial.println("aux6 changed: " + String(localController.aux6));

        lastController = localController;

        vTaskDelayUntil(&lastWake, freq);
    }
}
