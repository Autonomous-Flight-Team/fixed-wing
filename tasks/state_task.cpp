// Authors
//  - Colin Faletto github.com/faletto

#include "tasks.h"

const int MS_PER_TICK = 10; // 100Hz

// Estimate state using Unscented Kalman Filter (UKF)
void StateTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);

    for (;;) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            stateVector = estimate_state_with_real_measurements(
                sensorData.lat,
                sensorData.lon,
                sensorData.altitude,                
                sensorData.vs,
                sensorData.gx,
                sensorData.gy,
                sensorData.gz,
                sensorData.ax,
                sensorData.ay,
                sensorData.az               
            )
            xSemaphoreGive(dataMutex);
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}