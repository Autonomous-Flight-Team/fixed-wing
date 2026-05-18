// Authors
// - Colin Faletto github.com/faletto

#include "tasks.h"
#include "hardware.h"

const int MS_PER_TICK = 20; // 50 Hz

double KMPH_MPS_CONVERT_RATE = 3.6;

// Reads GPS data and outputs it into a SensorData_t struct
SensorData_t ReadGPS() {
    SensorData_t data = {0};
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
        data.lat = gps.location.lat();
        data.lon = gps.location.lng();
        data.gps_altitude = gps.altitude.meters();
    }

    if (gps.speed.isUpdated()) {
        // TinyGPS++ only provides a single linear speed
        data.vs = gps.speed.kmph() / KMPH_MPS_CONVERT_RATE;
    }

    return data;
}
// Reads GPS data and puts the data in shared memory
void GPSTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);

    for (;;) {
        SensorData_t gpsData = ReadGPS();
        GPSData_t gpsLogData = {};
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            sensorData.lat = gpsData.lat;
            sensorData.lon = gpsData.lon;
            sensorData.gps_altitude = gpsData.gps_altitude;
            sensorData.vs = gpsData.vs;
            gpsLogData.lat = gpsData.lat;
            gpsLogData.lon = gpsData.lon;
            gpsLogData.gps_altitude = gpsData.gps_altitude;
            gpsLogData.vs = gpsData.vs;
            xSemaphoreGive(dataMutex);
        }
        ConstructLogAndFillQueue(gpsLogData);
        vTaskDelayUntil(&lastWake, freq);
    }
}
