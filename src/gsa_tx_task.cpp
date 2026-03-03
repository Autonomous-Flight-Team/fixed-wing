#include "tasks.h"
#include "types.h"
#include "hardware.h"

const int MS_PER_TICK = 10; // Replace with (1 / Hz)

void GSATxTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);
    GSATxPacket_t buf;    

    // TODO: Do we even need a queue here? Can't we just send the current
    // state vector and sensor data directly?
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(MS_PER_TICK))) {
        GSATxPacket_t msg = {stateVector, sensorData};
        xQueueSendToBack(gsaTxQueue,&msg,pdMS_TO_TICKS(MS_PER_TICK));
        xSemaphoreGive(dataMutex);
    }

    for (;;) {
        if (xQueueReceive(gsaTxQueue, &buf, pdMS_TO_TICKS(MS_PER_TICK))) {
            MAVLINK_SERIAL_900.write((uint8_t*)&buf, sizeof(buf));
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}
