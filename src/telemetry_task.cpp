/*
Authors:
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"
#include "hardware.h"
#include "string.h"
#include <arduino_freertos.h>
#include <queue.h>
#include <TimeLib.h>

void FillLoggingQueues(Log<StateVector_t> log)
{
    if (xQueueSend(stateVector_logging_queue, &log, 0) != pdTRUE) {
        ++stateVector_logging_drop_count;
    }

}

void FillLoggingQueues(Log<IMUData_t> log)
{
    if (xQueueSend(imu_logging_queue, &log, 0) != pdTRUE)
    {
        ++imu_logging_drop_count;
    }
}

void FillLoggingQueues(Log<BaroData_t> log)
{
    if (xQueueSend(barometer_logging_queue, &log, 0) != pdTRUE)
    {
        ++barometer_logging_drop_count;
    }
}

void FillLoggingQueues(Log<GPSData_t> log)
{
    if (xQueueSend(gps_logging_queue, &log, 0) != pdTRUE)
    {
        ++gps_logging_drop_count;
    }
}

void FillLoggingQueues(Log<PitotData_t> log)
{
    if (xQueueSend(pitotTube_logging_queue, &log, 0) != pdTRUE)
    {
        ++pitotTube_logging_drop_count;
    }
}

void FillLoggingQueues(Log<ControlOutput_t> log)
{
    if (xQueueSend(controlOutput_logging_queue, &log, 0) != pdTRUE) {
        ++controlOutput_logging_drop_count;
    }
}

void FillLoggingQueues(Log<mavlink_manual_control_t> log)
{
    if (xQueueSend(manualControl_t_logging_queue, &log, 0) != pdTRUE) {
        ++manualControl_logging_drop_count;
    }
}

void SDCardTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(2000)); // Give Serial and SD time to settle
    Serial.println("SD Task: Starting...");

    // Re-run begin inside the task context
    if (!SD.begin(BUILTIN_SDCARD))
    {
        Serial.println("SD Task: SD.begin failed inside task!");
        vTaskDelete(NULL);
        return;
    }

    time_t t = Teensy3Clock.get();
    char path[64];     // Buffer for full path

    // 1. Create the directory path (e.g., /2026/04/29/)
    char dirPath[32];
    snprintf(dirPath, sizeof(dirPath), "/%04d/%02d/%02d", year(t), month(t), day(t));

    // 2. Ensure the directories exist (mkdir handles nested paths automatically)
    SD.mkdir(dirPath);

    // 3. Create the full file path (e.g., /2026/04/29/LOG_17-11-42.txt)
    snprintf(path, sizeof(path), "%s/LOG_%02d-%02d-%02d.dat",
             dirPath, hour(t), minute(t), second(t));

    File dataFile = SD.open(path, FILE_WRITE);
    if (!dataFile)
    {
        Serial.printf("SD Task: Open Failed! Error: %d\n", 0); // Check Serial Monitor
        vTaskDelete(NULL);
        return;
    }
    else
    {
        Serial.println("SD Task: File opened successfully!");
        dataFile.println("Logging started...");
    }

    const TickType_t consumePeriod = pdMS_TO_TICKS(400);
    TickType_t lastWake = xTaskGetTickCount();

    Log<IMUData_t> imuLog = {};
    Log<ControlOutput_t> controlLog = {};
    Log<StateVector_t> stateLog = {};
    Log<mavlink_manual_control_t> manualLog = {};
    Log<BaroData_t> baroLog = {};
    Log<GPSData_t> gpsLog = {};
    Log<PitotData_t> pitotLog = {};

    for (;;)
    {
        
        if (dataFile)
        {
            if (controlOutput_logging_queue != nullptr &&
                xQueueReceive(controlOutput_logging_queue, &controlLog, 0) == pdTRUE)
            {
                SD_Log_t sdControlLog = new SD_Log_t(controlLog.data);
                sdControlLog.timestamp = controlLog.timestamp;
                writeLogBinary("CTRL", sdControlLog, dataFile);
            }

            if (stateVector_logging_queue != nullptr &&
                xQueueReceive(stateVector_logging_queue, &stateLog, 0) == pdTRUE)
            {
                SD_Log_t sdStateLog = new SD_Log_t(stateLog.data);
                sdStateLog.timestamp = stateLog.timestamp;
                writeLogBinary("State_Vector", sdStateLog, dataFile);
            }

            if (manualControl_t_logging_queue != nullptr &&
                xQueueReceive(manualControl_t_logging_queue, &manualLog, 0) == pdTRUE)
            {
                SD_Log_t sdManualLog = new SD_Log_t(manualLog.data);
                sdManualLog.timestamp = manualLog.timestamp;
                writeLogBinary("Manual_Controller", sdManualLog, dataFile);
            }

            if (imu_logging_queue != nullptr &&
                xQueueReceive(imu_logging_queue, &imuLog, 0) == pdTRUE)
            {
                SD_Log_t sdIMULog = new SD_Log_t(imuLog.data);
                sdIMULog.timestamp = imuLog.timestamp;
                writeLogBinary("IMU", sdIMULog, dataFile);
            }

            if (barometer_logging_queue != nullptr &&
                xQueueReceive(barometer_logging_queue, &baroLog, 0) == pdTRUE)
            {
                SD_Log_t sdBarometerLog = new SD_Log_t(baroLog.data);
                sdBarometerLog.timestamp = baroLog.timestamp;
                writeLogBinary("BARO", sdBarometerLog, dataFile);
                //dataFile.println(baroLog.data);
            }

            if (gps_logging_queue != nullptr &&
                xQueueReceive(gps_logging_queue, &gpsLog, 0) == pdTRUE)
            {
                SD_Log_t sdGPSLog = new SD_Log_t(gpsLog.data);
                sdGPSLog.timestamp = gpsLog.timestamp;
                writeLogBinary("GPS", sdGPSLog, dataFile);
            }

            if (pitotTube_logging_queue != nullptr &&
                xQueueReceive(pitotTube_logging_queue, &pitotLog, 0) == pdTRUE)
            {
                SD_Log_t sdPitotLog = new SD_Log_t(pitotLog.data);
                sdPitotLog.timestamp = pitotLog.timestamp;
                writeLogBinary("Pitot", sdPitotLog, dataFile);
            }

            if (dataFile)
            {                
                // Use flush instead of close to save data periodically
                dataFile.flush();
            }
        }

        vTaskDelayUntil(&lastWake, consumePeriod);
    }
}