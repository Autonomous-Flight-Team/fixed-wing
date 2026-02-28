#include "tasks.h"

// Basic smoke test:
// 1) Push one entry to each logging queue.
// 2) Pop one entry from each queue.
// 3) Print queue round-trip results to Serial.
void LoggingQueueSmokeTestTask(void *pvParameters) {
    (void)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (sensorData_logging_queue == nullptr ||
        controlOutput_logging_queue == nullptr ||
        stateVector_logging_queue == nullptr ||
        manualControl_t_logging_queue == nullptr) {
        Serial.println("[LOG-TEST] ERROR: one or more logging queues are null");
        vTaskDelete(NULL);
    }

    Log<SensorData_t> sensorLog = {};
    sensorLog.data.ax = 1.25f;
    sensorLog.timestamp = xTaskGetTickCount();

    Log<ControlOutput_t> controlLog = {};
    controlLog.data.somethingneedstobehere = 42.0;
    controlLog.timestamp = xTaskGetTickCount();

    Log<StateVector_t> stateLog = {};
    stateLog.data.x = 123.0;
    stateLog.timestamp = xTaskGetTickCount();

    Log<mavlink_manual_control_t> manualLog = {};
    manualLog.timestamp = xTaskGetTickCount();

    FillLoggingQueues(sensorLog);
    FillLoggingQueues(controlLog);
    FillLoggingQueues(stateLog);
    FillLoggingQueues(manualLog);

    Log<SensorData_t> sensorOut = {};
    Log<ControlOutput_t> controlOut = {};
    Log<StateVector_t> stateOut = {};
    Log<mavlink_manual_control_t> manualOut = {};

    const BaseType_t gotSensor =
        xQueueReceive(sensorData_logging_queue, &sensorOut, pdMS_TO_TICKS(100));
    const BaseType_t gotControl =
        xQueueReceive(controlOutput_logging_queue, &controlOut, pdMS_TO_TICKS(100));
    const BaseType_t gotState =
        xQueueReceive(stateVector_logging_queue, &stateOut, pdMS_TO_TICKS(100));
    const BaseType_t gotManual =
        xQueueReceive(manualControl_t_logging_queue, &manualOut, pdMS_TO_TICKS(100));

    Serial.println("[LOG-TEST] Queue smoke test results:");
    Serial.print("[LOG-TEST] sensor: ");
    Serial.print(gotSensor == pdTRUE ? "OK" : "FAIL");
    if (gotSensor == pdTRUE) {
        Serial.print(" ax=");
        Serial.print(sensorOut.data.ax, 3);
        Serial.print(" ts=");
        Serial.println(sensorOut.timestamp);
    } else {
        Serial.println();
    }

    Serial.print("[LOG-TEST] control: ");
    Serial.print(gotControl == pdTRUE ? "OK" : "FAIL");
    if (gotControl == pdTRUE) {
        Serial.print(" out=");
        Serial.print(controlOut.data.somethingneedstobehere, 3);
        Serial.print(" ts=");
        Serial.println(controlOut.timestamp);
    } else {
        Serial.println();
    }

    Serial.print("[LOG-TEST] state: ");
    Serial.print(gotState == pdTRUE ? "OK" : "FAIL");
    if (gotState == pdTRUE) {
        Serial.print(" x=");
        Serial.print(stateOut.data.x, 3);
        Serial.print(" ts=");
        Serial.println(stateOut.timestamp);
    } else {
        Serial.println();
    }

    Serial.print("[LOG-TEST] manual: ");
    Serial.print(gotManual == pdTRUE ? "OK" : "FAIL");
    if (gotManual == pdTRUE) {
        Serial.print(" ts=");
        Serial.println(manualOut.timestamp);
    } else {
        Serial.println();
    }

    vTaskDelete(NULL);
}
