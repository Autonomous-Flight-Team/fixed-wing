#include "tasks.h"

const int MS_PER_TICK = 10; // Replace with (1 / Hz)

void MotorTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);

    for (;;) {
        // DO STUFF HERE
        vTaskDelayUntil(&lastWake, freq);
    }
}