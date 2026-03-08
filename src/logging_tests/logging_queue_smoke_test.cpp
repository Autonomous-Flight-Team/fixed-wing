#include "tasks.h"

// Continuous logging queue stress test:
// 1) Continuously push entries to all logging queues every 100 ms.
// 2) Keep queues full enough to exercise drop behavior.
// 3) Periodically print queue occupancy and drop counters.
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

    uint32_t i = 0;
    TickType_t lastReport = xTaskGetTickCount();
    //Serial.println("[LOG-TEST] Continuous queue fill started");

    for (;;) {
        Log<SensorData_t> sensorLog = {};
        sensorLog.data.ax = static_cast<float>(i);
        sensorLog.timestamp = xTaskGetTickCount();

        Log<ControlOutput_t> controlLog = {};
        controlLog.data.aileron = static_cast<float>((i % 200) / 100.0f - 1.0f);
        controlLog.data.elevator = static_cast<float>(((i + 25U) % 200) / 100.0f - 1.0f);
        controlLog.data.rudder = static_cast<float>(((i + 50U) % 200) / 100.0f - 1.0f);
        controlLog.data.throttle = static_cast<float>((i % 100) / 100.0f);
        controlLog.data.aileron_pwm = static_cast<uint16_t>(1000U + (i % 1000U));
        controlLog.data.elevator_pwm = static_cast<uint16_t>(1000U + ((i + 100U) % 1000U));
        controlLog.data.rudder_pwm = static_cast<uint16_t>(1000U + ((i + 200U) % 1000U));
        controlLog.data.throttle_pwm = static_cast<uint16_t>(1000U + ((i + 300U) % 1000U));
        controlLog.data.link_ok = ((i % 2U) == 0U);
        controlLog.timestamp = xTaskGetTickCount();

        Log<StateVector_t> stateLog = {};
        stateLog.data.x = static_cast<double>(i);
        stateLog.timestamp = xTaskGetTickCount();

        Log<mavlink_manual_control_t> manualLog = {};
        manualLog.data.x = static_cast<int16_t>(i & 0x7FFF);
        manualLog.timestamp = xTaskGetTickCount();

        FillLoggingQueues(sensorLog);
        FillLoggingQueues(controlLog);
        FillLoggingQueues(stateLog);
        FillLoggingQueues(manualLog);
        ++i;

        const TickType_t now = xTaskGetTickCount();
        if ((now - lastReport) >= pdMS_TO_TICKS(1000)) {
            lastReport = now;
            //Serial.println("[LOG-TEST] Queue stats:");

            //Serial.print("  sensor q=");
            //Serial.print(uxQueueMessagesWaiting(sensorData_logging_queue));
            //Serial.print(" dropped=");
            //Serial.println(sensorData_logging_drop_count);

            //Serial.print("  control q=");
            //Serial.print(uxQueueMessagesWaiting(controlOutput_logging_queue));
            //Serial.print(" dropped=");
            //Serial.println(controlOutput_logging_drop_count);

            //Serial.print("  state q=");
            //Serial.print(uxQueueMessagesWaiting(stateVector_logging_queue));
            //Serial.print(" dropped=");
            //Serial.println(stateVector_logging_drop_count);

            //Serial.print("  manual q=");
            //Serial.print(uxQueueMessagesWaiting(manualControl_t_logging_queue));
            //Serial.print(" dropped=");
            //Serial.println(manualControl_logging_drop_count);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
