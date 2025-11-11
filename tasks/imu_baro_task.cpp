#include "tasks.h"
#include "hardware.h"


const float SEA_LEVEL_PRESSURE = 1013.25;
const float KMPH_MPS_CONVERT_RATE = 3.6;
const int MS_PER_TICK = 5; // 200 Hz

void ImuBaroTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);

    for (;;) {
        SensorData_t newData = ReadAllSensors();
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            sensorData = newData;
            xSemaphoreGive(dataMutex);
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}

SensorData_t ReadAllSensors() {
    SensorData_t data = {0};
    ReadIMU(&data); ReadBaro(&data); ReadGPS(&data); 
    return data;
}

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



void ReadBaro(SensorData_t *data) {
    data -> alt = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    data -> pressure = bmp.readPressure();
    data -> temp = bmp.readTemperature();
}

