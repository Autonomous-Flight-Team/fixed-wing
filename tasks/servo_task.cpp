#include "tasks.h"

const int MS_PER_TICK = 0 // REPLACE WITH (1 / Hz)

void ServoTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);

    for (;;) {
        // DO STUFF HERE
        vTaskDelayUntil(&lastWake, freq);
    }

}