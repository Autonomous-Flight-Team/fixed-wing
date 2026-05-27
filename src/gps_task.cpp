// Authors
// - Colin Faletto github.com/faletto

#include "tasks.h"
#include "hardware.h"

const int MS_PER_TICK = 20; // 50 Hz

double KMPH_MPS_CONVERT_RATE = 3.6;

// Reads GPS data and outputs it into a GPSData_t struct
GPSData_t ReadGPS() {
    GPSData_t data = {0};
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
        GPSData_t newGps = ReadGPS();
        // Consider GPS invalid if all fields are zero (no fix / no speed)
        const bool gpsValid = !(newGps.lat == 0.0 && newGps.lon == 0.0 && newGps.gps_altitude == 0.0 && newGps.vs == 0.0);

        if (gpsValid) {
            if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
                gpsData.lat = newGps.lat;
                gpsData.lon = newGps.lon;
                gpsData.gps_altitude = newGps.gps_altitude;
                gpsData.vs = newGps.vs;
                xSemaphoreGive(dataMutex);
            }
            ConstructLogAndFillQueue(newGps);
        } else {
            Serial.println("INVALID GPS DATA");
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}
