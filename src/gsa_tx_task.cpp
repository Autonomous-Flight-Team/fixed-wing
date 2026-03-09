#include "tasks.h"
#include "types.h"
#include "hardware.h"

const int MS_PER_TICK = 10; // Replace with (1 / Hz)

void GSATxTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);
    GSATxPacket_t tx = {};

    // Initialize from shared state so TX starts with a valid packet.
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(MS_PER_TICK)) == pdTRUE) {
        tx.state = stateVector;
        tx.sensor = sensorData;
        xSemaphoreGive(dataMutex);
    }

    for (;;) {
        Log<StateVector_t> stateLog = {};
        if (xQueuePeek(stateVector_latest_queue, &stateLog, 0) == pdTRUE) {
            tx.state = stateLog.data;
        }

        Log<SensorData_t> sensorLog = {};
        if (xQueuePeek(sensorData_latest_queue, &sensorLog, 0) == pdTRUE) {
            tx.sensor = sensorLog.data;
        }

        MAVLINK_SERIAL_900.write((uint8_t *)&tx, sizeof(tx));
        vTaskDelayUntil(&lastWake, freq);
    }
}
