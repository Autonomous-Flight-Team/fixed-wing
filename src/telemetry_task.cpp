/*
Authors:
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"
#include "hardware.h"
#include "string.h"
#include <arduino_freertos.h>
#include <queue.h>

void FillLoggingQueues(Log<StateVector_t> log)
{
    if (xQueueSend(stateVector_logging_queue, &log, 0) != pdTRUE) {
        ++stateVector_logging_drop_count;
    }
    if (stateVector_latest_queue != nullptr) {
        xQueueOverwrite(stateVector_latest_queue, &log);
    }
}

void FillLoggingQueues(Log<SensorData_t> log)
{
    if (xQueueSend(sensorData_logging_queue, &log, 0) != pdTRUE) {
        ++sensorData_logging_drop_count;
    }
    if (sensorData_latest_queue != nullptr) {
        xQueueOverwrite(sensorData_latest_queue, &log);
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

void SDCardTask(void *pvParameters){
    // IMPLEMENT
    (void)pvParameters;
    const TickType_t consumePeriod = pdMS_TO_TICKS(400);
    TickType_t lastWake = xTaskGetTickCount();

    Log<SensorData_t> sensorLog = {};
    Log<ControlOutput_t> controlLog = {};
    Log<StateVector_t> stateLog = {};
    Log<mavlink_manual_control_t> manualLog = {};

    for (;;)
    {
        // Simulate slow SD card writes by draining each queue once per cycle.
        if (sensorData_logging_queue != nullptr) {
            (void)xQueueReceive(sensorData_logging_queue, &sensorLog, 0);
        }
        if (controlOutput_logging_queue != nullptr) {
            (void)xQueueReceive(controlOutput_logging_queue, &controlLog, 0);
        }
        if (stateVector_logging_queue != nullptr) {
            (void)xQueueReceive(stateVector_logging_queue, &stateLog, 0);
        }
        if (manualControl_t_logging_queue != nullptr) {
            (void)xQueueReceive(manualControl_t_logging_queue, &manualLog, 0);
        }

        vTaskDelayUntil(&lastWake, consumePeriod);
    }
}
