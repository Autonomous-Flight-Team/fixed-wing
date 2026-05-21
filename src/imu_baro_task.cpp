// Authors
// - Colin Faletto github.com/faletto

#include "tasks.h"
#include "hardware.h"

const float SEA_LEVEL_PRESSURE = 1013.25;
const float KMPH_MPS_CONVERT_RATE = 3.6;

const int MS_PER_TICK = 5; // 200 Hz

// Reads data from the IMU
void ReadIMU(IMUData_t *data)
{
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);
    data->ax = a.acceleration.x;
    data->ay = a.acceleration.y;
    data->az = a.acceleration.z;

    data->gx = g.acceleration.x;
    data->gy = g.acceleration.y;
    data->gz = g.acceleration.z;
}

// Reads data from the barometer
void ReadBaro(BaroData_t *data)
{
    data->altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    data->pressure = bmp.readPressure();
    data->temp = bmp.readTemperature();
}

// Gets IMU and Barometer data, writes to the shared per-sensor globals
void ImuBaroTask(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);

    for (;;)
    {
        IMUData_t newImu = {0};
        BaroData_t newBaro = {0};
        ReadIMU(&newImu);
        ReadBaro(&newBaro);
        // Validate IMU: consider invalid if all accel and gyro samples are zero
        const bool imuValid = !(newImu.ax == 0.0f && newImu.ay == 0.0f && newImu.az == 0.0f &&
                                newImu.gx == 0.0f && newImu.gy == 0.0f && newImu.gz == 0.0f);

        // Validate Barometer: invalid if altitude, pressure and temp are all zero
        const bool baroValid = !(newBaro.altitude == 0.0f && newBaro.pressure == 0.0f && newBaro.temp == 0.0f);

        if (xSemaphoreTake(dataMutex, portMAX_DELAY))
        {
            if (imuValid) {
                imuData = newImu;
                Serial.println(imuData.ax);
            }
            if (baroValid) {
                baroData = newBaro;
            }
            xSemaphoreGive(dataMutex);
        }

        if (imuValid) {
            ConstructLogAndFillQueue(newImu);
        } else {
            Serial.println("INVALID IMU DATA");
        }

        if (baroValid) {
            ConstructLogAndFillQueue(newBaro);
        } else {
            Serial.println("INVALID BARO DATA");
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}