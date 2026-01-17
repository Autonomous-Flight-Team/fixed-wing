// Authors
//  - Colin Faletto github.com/faletto

#include "tasks.h"
// #include "ukf.h"
const int MS_PER_TICK = 10; // 100Hz

// Estimate state using Unscented Kalman Filter (UKF)
void StateTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);

    for (;;) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            // stateVector = estimate_state_with_real_measurements(
            //     (float)sensorData.lat,
            //     (float)sensorData.lon,
            //     (float)sensorData.altitude,                
            //     (float)sensorData.vs,
            //     (float)sensorData.gx,
            //     (float)sensorData.gy,
            //     (float)sensorData.gz,
            //     (float)sensorData.ax,
            //     (float)sensorData.ay,
            //     (float)sensorData.az
            // );
            
               
            xSemaphoreGive(dataMutex);
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}