// Authors
// - Colin Faletto github.com/faletto

#include "tasks.h"
#include "hardware.h"


const float SEA_LEVEL_PRESSURE = 1013.25;
const float KMPH_MPS_CONVERT_RATE = 3.6;

const int MS_PER_TICK = 5; // 200 Hz

// Reads data from the IMU
void ReadIMU(SensorData_t *data) { 
    if (imu.dataReady()) {
        imu.getAGMT();
        data -> ax = imu.accX();
        data -> ay = imu.accY();
        data -> az = imu.accZ();
        data -> gx = imu.gyrX();
        data -> gy = imu.gyrY();
        data -> gz = imu.gyrZ();
    }
}

// Reads data from the barometer
void ReadBaro(SensorData_t *data) {
    data -> altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    data -> pressure = bmp.readPressure();
    data -> temp = bmp.readTemperature();
}

// Combines IMU and Barometer data into a single data structure
SensorData_t ReadImuBaro() {
    SensorData_t data = {0};
    ReadIMU(&data); 
    ReadBaro(&data);
    return data;
}

// Gets IMU and Barometer data, 
void ImuBaroTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);
    
    for (;;) {
        SensorData_t newData = ReadImuBaro();
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            // Serial.print("ax: ");
            // Serial.println(newData.ax);
            sensorData = newData;

            xSemaphoreGive(dataMutex);
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}
