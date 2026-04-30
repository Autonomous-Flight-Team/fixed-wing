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
    snprintf(path, sizeof(path), "%s/LOG_%02d-%02d-%02d.txt",
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
        dataFile.println("WE ARE HERE!");
        if (dataFile)
        {
            if (controlOutput_logging_queue != nullptr &&
                xQueueReceive(controlOutput_logging_queue, &controlLog, 0) == pdTRUE)
            {
                dataFile.print("CTRL,");
                dataFile.print(controlLog.timestamp);
                dataFile.print(",");
                // dataFile.println(controlLog.data);
            }

            if (stateVector_logging_queue != nullptr &&
                xQueueReceive(stateVector_logging_queue, &stateLog, 0) == pdTRUE)
            {
                dataFile.print("STATE,");
                dataFile.print(stateLog.timestamp);
                dataFile.print(",");
                //dataFile.println(stateLog.data);
            }

            if (manualControl_t_logging_queue != nullptr &&
                xQueueReceive(manualControl_t_logging_queue, &manualLog, 0) == pdTRUE)
            {
                dataFile.print("MAN,");
                dataFile.print(manualLog.timestamp);
                dataFile.print(",");
                //dataFile.println(manualLog.data);
            }

            if (imu_logging_queue != nullptr &&
                xQueueReceive(imu_logging_queue, &imuLog, 0) == pdTRUE)
            {
                dataFile.print("IMU,");
                dataFile.print(imuLog.timestamp);
                dataFile.print(",");
                //dataFile.println(imuLog.data);
            }

            if (barometer_logging_queue != nullptr &&
                xQueueReceive(barometer_logging_queue, &baroLog, 0) == pdTRUE)
            {
                dataFile.print("BARO,");
                dataFile.print(baroLog.timestamp);
                dataFile.print(",");
                //dataFile.println(baroLog.data);
            }

            if (gps_logging_queue != nullptr &&
                xQueueReceive(gps_logging_queue, &gpsLog, 0) == pdTRUE)
            {
                dataFile.print("GPS,");
                dataFile.print(gpsLog.timestamp);
                dataFile.print(",");
                //dataFile.println(gpsLog.data);
            }

            if (pitotTube_logging_queue != nullptr &&
                xQueueReceive(pitotTube_logging_queue, &pitotLog, 0) == pdTRUE)
            {
                dataFile.print("PITOT,");
                dataFile.print(pitotLog.timestamp);
                dataFile.print(",");
                //dataFile.println(pitotLog.data);
            }

            if (dataFile)
            {
                dataFile.println("Logging data...");
                // Use flush instead of close to save data periodically
                dataFile.flush();
            }
        }

        vTaskDelayUntil(&lastWake, consumePeriod);
    }
}