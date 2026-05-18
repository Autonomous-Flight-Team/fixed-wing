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
    if (stateVector_logging_queue != nullptr &&
        xQueueSend(stateVector_logging_queue, &log, 0) != pdTRUE) {
        ++stateVector_logging_drop_count;
    }

}

void FillLoggingQueues(Log<IMUData_t> log)
{
    if (imu_logging_queue != nullptr &&
        xQueueSend(imu_logging_queue, &log, 0) != pdTRUE)
    {
        ++imu_logging_drop_count;
    }
}

void FillLoggingQueues(Log<BaroData_t> log)
{
    if (barometer_logging_queue != nullptr &&
        xQueueSend(barometer_logging_queue, &log, 0) != pdTRUE)
    {
        ++barometer_logging_drop_count;
    }
}

void FillLoggingQueues(Log<GPSData_t> log)
{
    if (gps_logging_queue != nullptr &&
        xQueueSend(gps_logging_queue, &log, 0) != pdTRUE)
    {
        ++gps_logging_drop_count;
    }
}

void FillLoggingQueues(Log<PitotData_t> log)
{
    if (pitotTube_logging_queue != nullptr &&
        xQueueSend(pitotTube_logging_queue, &log, 0) != pdTRUE)
    {
        ++pitotTube_logging_drop_count;
    }
}

void FillLoggingQueues(Log<ControlOutput_t> log)
{
    if (controlOutput_logging_queue != nullptr &&
        xQueueSend(controlOutput_logging_queue, &log, 0) != pdTRUE) {
        ++controlOutput_logging_drop_count;
    }
}

void FillLoggingQueues(Log<mavlink_manual_control_t> log)
{
    if (manualControl_t_logging_queue != nullptr &&
        xQueueSend(manualControl_t_logging_queue, &log, 0) != pdTRUE) {
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

    const TickType_t consumePeriod = pdMS_TO_TICKS(20);
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t flushPeriod = pdMS_TO_TICKS(1000);
    TickType_t lastFlush = lastWake;
    const TickType_t holdWritePeriod = pdMS_TO_TICKS(1000);
    const uint16_t maxMessagesPerCycle = 200U;
    uint8_t nextQueueStart = 0U;

    Log<IMUData_t> imuLog = {};
    Log<ControlOutput_t> controlLog = {};
    Log<StateVector_t> stateLog = {};
    Log<mavlink_manual_control_t> manualLog = {};
    Log<BaroData_t> baroLog = {};
    Log<GPSData_t> gpsLog = {};
    Log<PitotData_t> pitotLog = {};

    SD_Log_t<ControlOutput_t> heldControl = {};
    SD_Log_t<StateVector_t> heldState = {};
    SD_Log_t<PitotData_t> heldPitot = {};
    TickType_t lastControlSeen = lastWake;
    TickType_t lastStateSeen = lastWake;
    TickType_t lastPitotSeen = lastWake;

    for (;;)
    {
        
        if (dataFile)
        {
            uint16_t drainedThisCycle = 0U;
            bool madeProgress = true;
            while (drainedThisCycle < maxMessagesPerCycle && madeProgress)
            {
                madeProgress = false;
                for (uint8_t i = 0; i < 7U && drainedThisCycle < maxMessagesPerCycle; ++i)
                {
                    const uint8_t queueIndex = static_cast<uint8_t>((nextQueueStart + i) % 7U);
                    switch (queueIndex)
                    {
                    case 0:
                        if (controlOutput_logging_queue != nullptr &&
                            xQueueReceive(controlOutput_logging_queue, &controlLog, 0) == pdTRUE)
                        {
                            SD_Log_t<ControlOutput_t> sdControlLog(controlLog.data);
                            sdControlLog.timestamp = controlLog.timestamp;
                            writeLogBinary("CTRL", sdControlLog, dataFile);
                            heldControl = sdControlLog;
                            lastControlSeen = xTaskGetTickCount();
                            ++drainedThisCycle;
                            madeProgress = true;
                        }
                        break;
                    case 1:
                        if (stateVector_logging_queue != nullptr &&
                            xQueueReceive(stateVector_logging_queue, &stateLog, 0) == pdTRUE)
                        {
                            SD_Log_t<StateVector_t> sdStateLog(stateLog.data);
                            sdStateLog.timestamp = stateLog.timestamp;
                            writeLogBinary("State_Vector", sdStateLog, dataFile);
                            heldState = sdStateLog;
                            lastStateSeen = xTaskGetTickCount();
                            ++drainedThisCycle;
                            madeProgress = true;
                        }
                        break;
                    case 2:
                        if (manualControl_t_logging_queue != nullptr &&
                            xQueueReceive(manualControl_t_logging_queue, &manualLog, 0) == pdTRUE)
                        {
                            SD_Log_t<mavlink_manual_control_t> sdManualLog(manualLog.data);
                            sdManualLog.timestamp = manualLog.timestamp;
                            writeLogBinary("Manual_Controller", sdManualLog, dataFile);
                            ++drainedThisCycle;
                            madeProgress = true;
                        }
                        break;
                    case 3:
                        if (imu_logging_queue != nullptr &&
                            xQueueReceive(imu_logging_queue, &imuLog, 0) == pdTRUE)
                        {
                            SD_Log_t<IMUData_t> sdIMULog(imuLog.data);
                            sdIMULog.timestamp = imuLog.timestamp;
                            writeLogBinary("IMU", sdIMULog, dataFile);
                            ++drainedThisCycle;
                            madeProgress = true;
                        }
                        break;
                    case 4:
                        if (barometer_logging_queue != nullptr &&
                            xQueueReceive(barometer_logging_queue, &baroLog, 0) == pdTRUE)
                        {
                            SD_Log_t<BaroData_t> sdBarometerLog(baroLog.data);
                            sdBarometerLog.timestamp = baroLog.timestamp;
                            writeLogBinary("BARO", sdBarometerLog, dataFile);
                            ++drainedThisCycle;
                            madeProgress = true;
                        }
                        break;
                    case 5:
                        if (gps_logging_queue != nullptr &&
                            xQueueReceive(gps_logging_queue, &gpsLog, 0) == pdTRUE)
                        {
                            SD_Log_t<GPSData_t> sdGPSLog(gpsLog.data);
                            sdGPSLog.timestamp = gpsLog.timestamp;
                            writeLogBinary("GPS", sdGPSLog, dataFile);
                            ++drainedThisCycle;
                            madeProgress = true;
                        }
                        break;
                    case 6:
                        if (pitotTube_logging_queue != nullptr &&
                            xQueueReceive(pitotTube_logging_queue, &pitotLog, 0) == pdTRUE)
                        {
                            SD_Log_t<PitotData_t> sdPitotLog(pitotLog.data);
                            sdPitotLog.timestamp = pitotLog.timestamp;
                            writeLogBinary("Pitot", sdPitotLog, dataFile);
                            heldPitot = sdPitotLog;
                            lastPitotSeen = xTaskGetTickCount();
                            ++drainedThisCycle;
                            madeProgress = true;
                        }
                        break;
                    default:
                        break;
                    }
                }
            }
            nextQueueStart = static_cast<uint8_t>((nextQueueStart + 1U) % 7U);

            const TickType_t now = xTaskGetTickCount();

            if ((now - lastControlSeen) >= holdWritePeriod)
            {
                heldControl.timestamp = now;
                writeLogBinary("CTRL_HOLD", heldControl, dataFile);
                lastControlSeen = now;
            }
            if ((now - lastStateSeen) >= holdWritePeriod)
            {
                heldState.timestamp = now;
                writeLogBinary("STATE_HOLD", heldState, dataFile);
                lastStateSeen = now;
            }
            if ((now - lastPitotSeen) >= holdWritePeriod)
            {
                heldPitot.timestamp = now;
                writeLogBinary("PITOT_HOLD", heldPitot, dataFile);
                lastPitotSeen = now;
            }

            if ((now - lastFlush) >= flushPeriod)
            {
                dataFile.flush();
                lastFlush = now;
            }
        }

        vTaskDelayUntil(&lastWake, consumePeriod);
    }
}
