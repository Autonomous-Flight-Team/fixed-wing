/*
Authors:
Advik Sharma - github.com/jpyces
*/

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

struct ControlOutputSnapshot {
    uint16_t aileronPwm = 1500U;
    uint16_t elevatorPwm = 1500U;
    uint16_t rudderPwm = 1500U;
    uint16_t throttlePwm = 1000U;
};

void SendMavlinkMessage(const mavlink_message_t &msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = {};
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    MavlinkSerial900Write(buffer, len);
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

int16_t ClampManualAxis(int16_t value) {
    if (value > 1000) {
        return 1000;
    }
    if (value < -1000) {
        return -1000;
    }
    return value;
}

int16_t ClampManualThrottle(int16_t value) {
    if (value > 1000) {
        return 1000;
    }
    if (value < 0) {
        return 0;
    }
    return value;
}

uint16_t AxisToRcPwm(int16_t axis) {
    const int16_t clamped = ClampManualAxis(axis);
    return static_cast<uint16_t>(1500 + (clamped / 2));
}

uint16_t ThrottleToRcPwm(int16_t throttle) {
    const int16_t clamped = ClampManualThrottle(throttle);
    return static_cast<uint16_t>(1000 + clamped);
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

ControlOutputSnapshot ReadControlOutputSnapshot() {
    ControlOutputSnapshot snapshot = {};
    taskENTER_CRITICAL();
    snapshot.aileronPwm = controlOutput.aileron_pwm;
    snapshot.elevatorPwm = controlOutput.elevator_pwm;
    snapshot.rudderPwm = controlOutput.rudder_pwm;
    snapshot.throttlePwm = controlOutput.throttle_pwm;
    taskEXIT_CRITICAL();
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

void SendRcChannels(const ManualInputSnapshot &snapshot) {
    const bool inputFresh = ((millis() - snapshot.lastInputMs) < 300U) && snapshot.manualInputEnabled;
    const uint16_t roll = inputFresh ? AxisToRcPwm(snapshot.mc.y) : 1500U;
    const uint16_t pitch = inputFresh ? AxisToRcPwm(snapshot.mc.x) : 1500U;
    const uint16_t throttle = inputFresh ? ThrottleToRcPwm(snapshot.mc.z) : 1000U;
    const uint16_t yaw = inputFresh ? AxisToRcPwm(snapshot.mc.r) : 1500U;

    mavlink_message_t msg = {};
    mavlink_rc_channels_t rc = {};
    rc.time_boot_ms = millis();
    rc.chancount = 4U;
    rc.chan1_raw = roll;
    rc.chan2_raw = pitch;
    rc.chan3_raw = throttle;
    rc.chan4_raw = yaw;
    rc.rssi = inputFresh ? 100U : 0U;

    mavlink_msg_rc_channels_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &rc);
    SendMavlinkMessage(msg);
}

void SendServoOutputRaw(const ControlOutputSnapshot &snapshot) {
    mavlink_message_t msg = {};
    mavlink_servo_output_raw_t servo = {};
    servo.time_usec = micros();
    servo.port = 0U;
    servo.servo1_raw = snapshot.aileronPwm;
    servo.servo2_raw = snapshot.elevatorPwm;
    servo.servo3_raw = snapshot.throttlePwm;
    servo.servo4_raw = snapshot.rudderPwm;

    mavlink_msg_servo_output_raw_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &servo);
    SendMavlinkMessage(msg);
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
        const ControlOutputSnapshot controlSnapshot = ReadControlOutputSnapshot();

        if ((loopCounter % 5U) == 0U) {
            SendRcChannels(inputSnapshot);
            SendServoOutputRaw(controlSnapshot);
        }

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
