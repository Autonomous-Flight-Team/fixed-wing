#include "tasks.h"
#include "hardware.h"

// Design note:
// Keep heartbeat/task-control logic isolated from simulator and latency code so this
// 50 Hz loop remains predictable and easy to reason about under load.
namespace {
struct ManualInputSnapshot {
    mavlink_manual_control_t mc = {};
    uint32_t lastInputMs = 0U;
    bool armed = false;
    bool manualInputEnabled = false;
};

void SendMavlinkMessage(const mavlink_message_t &msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = {};
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    MAVLINK_SERIAL_900.write(buffer, len);
}

void SendSysStatus() {
    constexpr uint32_t healthySensors =
        MAV_SYS_STATUS_SENSOR_3D_GYRO |
        MAV_SYS_STATUS_SENSOR_3D_ACCEL |
        MAV_SYS_STATUS_SENSOR_3D_MAG |
        MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
        MAV_SYS_STATUS_SENSOR_GPS |
        MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
        MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_PREARM_CHECK;

    mavlink_message_t msg = {};
    mavlink_sys_status_t status = {};
    status.onboard_control_sensors_present = healthySensors;
    status.onboard_control_sensors_enabled = healthySensors;
    status.onboard_control_sensors_health = healthySensors;
    status.load = 100U;                 // 1.0%
    status.voltage_battery = 12000U;    // 12.0V
    status.current_battery = -1;        // Not measured
    status.battery_remaining = 75;      // Placeholder

    mavlink_msg_sys_status_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &status);
    SendMavlinkMessage(msg);
}

float Clampf(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

float NormalizeStickAxis(int16_t axis) {
    return Clampf(static_cast<float>(axis) / 1000.0f, -1.0f, 1.0f);
}

float NormalizeThrottle(int16_t axis) {
    if (axis >= 0 && axis <= 1000) {
        return Clampf(static_cast<float>(axis) / 1000.0f, 0.0f, 1.0f);
    }
    return Clampf((static_cast<float>(axis) + 1000.0f) / 2000.0f, 0.0f, 1.0f);
}

uint16_t NormalizedToPwm(float value, uint16_t minPwm, uint16_t maxPwm) {
    const float clamped = Clampf(value, -1.0f, 1.0f);
    const float t = (clamped + 1.0f) * 0.5f;
    const float pwm = static_cast<float>(minPwm) + (static_cast<float>(maxPwm - minPwm) * t);
    return static_cast<uint16_t>(pwm);
}

uint16_t ThrottleToPwm(float value) {
    const float clamped = Clampf(value, 0.0f, 1.0f);
    const float pwm = 1000.0f + (clamped * 1000.0f);
    return static_cast<uint16_t>(pwm);
}

void LockMavlinkData() {
    if (mavlinkDataMutex != nullptr) {
        xSemaphoreTake(mavlinkDataMutex, portMAX_DELAY);
    }
}

void UnlockMavlinkData() {
    if (mavlinkDataMutex != nullptr) {
        xSemaphoreGive(mavlinkDataMutex);
    }
}

ManualInputSnapshot ReadManualInputSnapshot() {
    ManualInputSnapshot snapshot = {};
    LockMavlinkData();
    snapshot.mc = manual_control_data;
    snapshot.lastInputMs = mavlinkLastManualInputMs;
    snapshot.armed = mavlinkVehicleArmed;
    snapshot.manualInputEnabled = (mavlinkVehicleBaseMode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) != 0U;
    UnlockMavlinkData();
    return snapshot;
}

void UpdateControlOutput(const ManualInputSnapshot &snapshot) {
    const bool inputFresh = (millis() - snapshot.lastInputMs) < 300U;
    const bool linkOk = inputFresh && snapshot.manualInputEnabled;

    const float elevator = linkOk ? NormalizeStickAxis(snapshot.mc.x) : 0.0f;
    const float aileron = linkOk ? NormalizeStickAxis(snapshot.mc.y) : 0.0f;
    const float rudder = linkOk ? NormalizeStickAxis(snapshot.mc.r) : 0.0f;
    const float throttle =
        (snapshot.armed && linkOk) ? NormalizeThrottle(snapshot.mc.z) : 0.0f;

    // Publish a coherent snapshot for downstream telemetry/logging consumers.
    taskENTER_CRITICAL();
    controlOutput.elevator = elevator;
    controlOutput.aileron = aileron;
    controlOutput.rudder = rudder;
    controlOutput.throttle = throttle;
    controlOutput.elevator_pwm = NormalizedToPwm(elevator, 1000U, 2000U);
    controlOutput.aileron_pwm = NormalizedToPwm(aileron, 1000U, 2000U);
    controlOutput.rudder_pwm = NormalizedToPwm(rudder, 1000U, 2000U);
    controlOutput.throttle_pwm = ThrottleToPwm(throttle);
    controlOutput.link_ok = linkOk;
    taskEXIT_CRITICAL();
}
}  // namespace

void MavlinkHeartbeatTask(void *pvParameters) {
    (void)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(20);  // 50 Hz control loop
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t loopCounter = 0U;
    for (;;) {
        const ManualInputSnapshot inputSnapshot = ReadManualInputSnapshot();
        UpdateControlOutput(inputSnapshot);

        if ((loopCounter % 50U) == 0U) {
            mavlink_message_t heartbeatMsg = {};
            LockMavlinkData();
            const uint8_t baseMode = mavlinkVehicleBaseMode;
            const uint32_t customMode = mavlinkVehicleCustomMode;
            const uint8_t systemState = mavlinkVehicleSystemStatus;
            UnlockMavlinkData();
            mavlink_msg_heartbeat_pack(
                MAVLINK_SYSTEM_ID,
                MAVLINK_COMPONENT_ID,
                &heartbeatMsg,
                MAV_TYPE_FIXED_WING,
                MAV_AUTOPILOT_GENERIC,
                baseMode,
                customMode,
                systemState
            );
            SendMavlinkMessage(heartbeatMsg);
            SendSysStatus();
        }

        ++loopCounter;
        vTaskDelayUntil(&lastWake, period);
    }
}
