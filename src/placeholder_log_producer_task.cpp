/*
Authors:
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"

namespace {
const TickType_t kPlaceholderPeriod = pdMS_TO_TICKS(100); // 10 Hz
}

// Temporary producer for streams that are not wired to real sensing/control yet.
// Replace these payloads with true upstream sources when available.
void PlaceholderLogProducerTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        StateVector_t dummyState = {};
        dummyState.x = 0.0;
        dummyState.y = 0.0;
        dummyState.z = 0.0;
        dummyState.u = 0.0;
        dummyState.v = 0.0;
        dummyState.w = 0.0;
        dummyState.phi = 0.0;
        dummyState.theta = 0.0;
        dummyState.psi = 0.0;
        dummyState.p = 0.0;
        dummyState.q = 0.0;
        dummyState.r = 0.0;

        ControlOutput_t dummyControl = {};
        dummyControl.aileron = 0.0f;
        dummyControl.elevator = 0.0f;
        dummyControl.rudder = 0.0f;
        dummyControl.throttle = 0.0f;
        dummyControl.aileron_pwm = 1500U;
        dummyControl.elevator_pwm = 1500U;
        dummyControl.rudder_pwm = 1500U;
        dummyControl.throttle_pwm = 1000U;
        dummyControl.link_ok = false;

        PitotData_t dummyPitot = {};

        ConstructLogAndFillQueue(dummyState);
        ConstructLogAndFillQueue(dummyControl);
        ConstructLogAndFillQueue(dummyPitot);

        vTaskDelayUntil(&lastWake, kPlaceholderPeriod);
    }
}
