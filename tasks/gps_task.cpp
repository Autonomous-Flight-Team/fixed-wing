#include "tasks.h"

const int MS_PER_TICK = 20; // 50 Hz

void GPSTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);

    for (;;) {
        SensorData_t gpsData = ReadGPS();
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            sensorData.lat = gpsData.lat;
            sensorData.lon = gpsData.lon;
            sensorData.gps_altitude = gpsData.gps_altitude;
            sensorData.vs = gpsData.vs;
            xSemaphoreGive(dataMutex);
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}

SensorData_t ReadGPS() {
    SensorData_t data = {0};
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
        data.lat = gps.location.lat();
        data.lon = gps.location.lon();
        data.gps_altitude = gps.altitude.meters();
    }

    if (gps.speed.isUpdated()) {
        // TinyGPS++ only provides a single linear speed
        data.vs = gps.speed.kmph() / KMPH_MPS_CONVERT_RATE;
    }

    return data;
}