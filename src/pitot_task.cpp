// Authors
// - Colin Faletto github.com/faletto

#include "tasks.h"
#include "hardware.h"
#include <Wire.h>
#include <math.h>

const int MS_PER_TICK = 5; // 200 Hz

// I2C address for the Pixhawk differential airspeed sensor (adjust if needed)
#ifndef PITOT_I2C_ADDR
#define PITOT_I2C_ADDR 0x28
#endif

// Factor to convert raw sensor units to Pascals. Set to 1.0 by default and
// adjust according to the actual sensor datasheet / scaling.
#ifndef PITOT_RAW_TO_PASCAL
#define PITOT_RAW_TO_PASCAL 1.0f
#endif



void PitotTask(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);


    for (;;)
    {
        PitotData_t sample = {};

        // Read 2 bytes from the I2C differential pressure sensor.
        // Many Pixhawk kits provide a signed 16-bit differential pressure value.
        Wire1.requestFrom((uint8_t)PITOT_I2C_ADDR, (uint8_t)2);
        if (Wire1.available() >= 2)
        {
            uint8_t hi = Wire1.read();
            uint8_t lo = Wire1.read();
            int16_t raw = (int16_t)((hi << 8) | lo);
            float diff_pa = (float)raw * PITOT_RAW_TO_PASCAL;

            // Estimate local air density using barometer if available
            float rho = 1.225f; // sea-level default (kg/m^3)
            if (baroData.pressure > 0.0f)
            {
                float temp_k = baroData.temp + 273.15f;
                if (temp_k > 0.0f)
                {
                    rho = baroData.pressure / (287.05f * temp_k);
                }
            }

            float ias = 0.0f;
            if (diff_pa > 0.0f && rho > 0.0f)
            {
                ias = sqrtf(2.0f * diff_pa / rho);
            }

            sample.diff_pressure_pa = diff_pa;
            sample.ias_mps = ias;
            sample.tas_mps = ias; // TAS approximation; refine if wind/altitude correction desired
            sample.temp_c = baroData.temp;
            sample.valid = true;
        }
        else
        {
            sample.valid = false;
        }

        if (xSemaphoreTake(dataMutex, portMAX_DELAY))
        {
            if (sample.valid) {
                pitotData = sample;
            }
            xSemaphoreGive(dataMutex);
        }

        if (sample.valid) {
            ConstructLogAndFillQueue(sample);
        } else {
            Serial.println("INVALID PITOT DATA");
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}