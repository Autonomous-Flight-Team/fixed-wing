#include "tasks.h"
#include "hardware.h"


float SEA_LEVEL_PRESSURE = 1013.25;
float KMPH_MPS_CONVERT_RATE = 3.6;


void SensorTask(void *pvParameters) {
    SensorData_t newData = ReadAllSensors();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        sensorData = newData;
        xSemaphoreGive(dataMutex);
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

void ReadGPS(SensorData_t *data) {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
        data -> lat = gps.location.lat();
        data -> lon = gps.location.lon();
        data -> gps_altitude = gps.altitude.meters();
    }

    if (gps.speed.isUpdated()) {
        // TinyGPS++ only provides a single linear speed
        data -> vs = gps.speed.kmph() / KMPH_MPS_CONVERT_RATE;
    }

}