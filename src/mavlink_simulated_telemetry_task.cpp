#include "tasks.h"
#include "hardware.h"

#include <math.h>

// Design note:
// Simulator behavior is intentionally isolated to this task unit so simulated
// telemetry can evolve independently without increasing heartbeat/task complexity.
namespace {
constexpr int32_t kSeattleLatE7 = 476062000;     // 47.6062000 deg
constexpr int32_t kSeattleLonE7 = -1223321000;   // -122.3321000 deg
constexpr int32_t kSeattleAltMslMm = 52000;      // 52m AMSL
constexpr float kPi = 3.14159265359f;

struct SimFlightState {
    double latDeg = static_cast<double>(kSeattleLatE7) / 1.0e7;
    double lonDeg = static_cast<double>(kSeattleLonE7) / 1.0e7;
    float relAltM = 0.0f;
    float speedMps = 0.0f;
    float climbMps = 0.0f;
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float yawDeg = 90.0f;  // Start eastbound
    float throttleNorm = 0.0f;
    float vnMps = 0.0f;
    float veMps = 0.0f;
};

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

float Clampf(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

float WrapHeadingDeg(float headingDeg) {
    while (headingDeg < 0.0f) {
        headingDeg += 360.0f;
    }
    while (headingDeg >= 360.0f) {
        headingDeg -= 360.0f;
    }
    return headingDeg;
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

void UpdateSimFromManualControl(
    SimFlightState &sim,
    float dtSec,
    const ManualInputSnapshot &snapshot
) {
    const bool hasFreshInput =
        ((millis() - snapshot.lastInputMs) < 300U) && snapshot.manualInputEnabled;
    const bool armed = snapshot.armed;

    const float pitchInput = hasFreshInput ? NormalizeStickAxis(snapshot.mc.x) : 0.0f;
    const float rollInput = hasFreshInput ? NormalizeStickAxis(snapshot.mc.y) : 0.0f;
    const float yawInput = hasFreshInput ? NormalizeStickAxis(snapshot.mc.r) : 0.0f;
    const float throttleInput = (armed && hasFreshInput) ? NormalizeThrottle(snapshot.mc.z) : 0.0f;

    sim.throttleNorm = throttleInput;

    const float rollTargetDeg = rollInput * 45.0f;
    const float pitchTargetDeg = pitchInput * 20.0f;
    const float speedTargetMps = armed ? (30.0f * throttleInput) : 0.0f;

    sim.rollDeg += (rollTargetDeg - sim.rollDeg) * Clampf(dtSec * 3.5f, 0.0f, 1.0f);
    sim.pitchDeg += (pitchTargetDeg - sim.pitchDeg) * Clampf(dtSec * 2.8f, 0.0f, 1.0f);
    sim.speedMps += (speedTargetMps - sim.speedMps) * Clampf(dtSec * 1.6f, 0.0f, 1.0f);

    const float turnRateDegPerSec = (yawInput * 40.0f) + (sim.rollDeg * 0.45f);
    sim.yawDeg = WrapHeadingDeg(sim.yawDeg + turnRateDegPerSec * dtSec);

    float climbTargetMps = (sim.pitchDeg * 0.12f) + ((throttleInput - 0.5f) * 2.0f);
    if (!armed) {
        climbTargetMps = 0.0f;
    }
    sim.climbMps += (climbTargetMps - sim.climbMps) * Clampf(dtSec * 2.0f, 0.0f, 1.0f);
    sim.climbMps = Clampf(sim.climbMps, -6.0f, 6.0f);

    sim.relAltM = Clampf(sim.relAltM + (sim.climbMps * dtSec), 0.0f, 400.0f);

    const float yawRad = sim.yawDeg * (kPi / 180.0f);
    const float horizontalSpeedMps = sim.speedMps * cosf(sim.pitchDeg * (kPi / 180.0f));
    sim.vnMps = horizontalSpeedMps * cosf(yawRad);
    sim.veMps = horizontalSpeedMps * sinf(yawRad);

    const double metersPerDegLat = 111111.0;
    const double cosLat = cos(sim.latDeg * (kPi / 180.0f));
    const double metersPerDegLon = metersPerDegLat * ((fabs(cosLat) < 0.01) ? 0.01 : cosLat);

    sim.latDeg += static_cast<double>(sim.vnMps * dtSec) / metersPerDegLat;
    sim.lonDeg += static_cast<double>(sim.veMps * dtSec) / metersPerDegLon;
}

void SendGpsRawInt(const SimFlightState &sim) {
    mavlink_message_t msg = {};
    mavlink_gps_raw_int_t gps = {};
    gps.time_usec = static_cast<uint64_t>(millis()) * 1000ULL;
    gps.fix_type = 3U;              // 3D fix
    gps.lat = static_cast<int32_t>(sim.latDeg * 1.0e7);
    gps.lon = static_cast<int32_t>(sim.lonDeg * 1.0e7);
    gps.alt = kSeattleAltMslMm + static_cast<int32_t>(sim.relAltM * 1000.0f);
    gps.eph = 80U;                  // 0.8m HDOP-equivalent
    gps.epv = 120U;                 // 1.2m VDOP-equivalent
    gps.vel = static_cast<uint16_t>(Clampf(sim.speedMps * 100.0f, 0.0f, 65535.0f));
    gps.cog = static_cast<uint16_t>(WrapHeadingDeg(sim.yawDeg) * 100.0f);
    gps.satellites_visible = 12U;

    mavlink_msg_gps_raw_int_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &gps);
    SendMavlinkMessage(msg);
}

void SendGlobalPositionInt(const SimFlightState &sim) {
    mavlink_message_t msg = {};
    mavlink_global_position_int_t pos = {};
    pos.time_boot_ms = millis();
    pos.lat = static_cast<int32_t>(sim.latDeg * 1.0e7);
    pos.lon = static_cast<int32_t>(sim.lonDeg * 1.0e7);
    pos.alt = kSeattleAltMslMm + static_cast<int32_t>(sim.relAltM * 1000.0f);
    pos.relative_alt = static_cast<int32_t>(sim.relAltM * 1000.0f);
    pos.vx = static_cast<int16_t>(sim.vnMps * 100.0f);
    pos.vy = static_cast<int16_t>(sim.veMps * 100.0f);
    pos.vz = static_cast<int16_t>(-sim.climbMps * 100.0f);  // Positive down in MAVLink
    pos.hdg = static_cast<uint16_t>(WrapHeadingDeg(sim.yawDeg) * 100.0f);

    mavlink_msg_global_position_int_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &pos);
    SendMavlinkMessage(msg);
}

void SendAttitude(const SimFlightState &sim) {
    mavlink_message_t msg = {};
    mavlink_attitude_t attitude = {};
    attitude.time_boot_ms = millis();
    attitude.roll = sim.rollDeg * (kPi / 180.0f);
    attitude.pitch = sim.pitchDeg * (kPi / 180.0f);
    attitude.yaw = sim.yawDeg * (kPi / 180.0f);
    attitude.rollspeed = 0.0f;
    attitude.pitchspeed = 0.0f;
    attitude.yawspeed = 0.0f;

    mavlink_msg_attitude_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &attitude);
    SendMavlinkMessage(msg);
}

void SendVfrHud(const SimFlightState &sim) {
    mavlink_message_t msg = {};
    mavlink_vfr_hud_t hud = {};
    hud.airspeed = sim.speedMps;
    hud.groundspeed = sqrtf((sim.vnMps * sim.vnMps) + (sim.veMps * sim.veMps));
    hud.heading = static_cast<int16_t>(WrapHeadingDeg(sim.yawDeg));
    hud.throttle = static_cast<uint16_t>(sim.throttleNorm * 100.0f);
    hud.alt = sim.relAltM + (static_cast<float>(kSeattleAltMslMm) / 1000.0f);
    hud.climb = sim.climbMps;

    mavlink_msg_vfr_hud_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &hud);
    SendMavlinkMessage(msg);
}
}  // namespace

void MavlinkSimulatedTelemetryTask(void *pvParameters) {
    (void)pvParameters;

    // Decoupled cadence from heartbeat: telemetry realism can be tuned without
    // changing core heartbeat/control-output timing.
    const TickType_t period = pdMS_TO_TICKS(100);  // 10 Hz simulated telemetry
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t lastUpdateMs = millis();
    SimFlightState sim = {};

    for (;;) {
        const uint32_t nowMs = millis();
        float dtSec = static_cast<float>(nowMs - lastUpdateMs) / 1000.0f;
        lastUpdateMs = nowMs;
        dtSec = Clampf(dtSec, 0.01f, 0.2f);

        const ManualInputSnapshot inputSnapshot = ReadManualInputSnapshot();
        UpdateSimFromManualControl(sim, dtSec, inputSnapshot);

        SendGpsRawInt(sim);
        SendGlobalPositionInt(sim);
        SendAttitude(sim);
        SendVfrHud(sim);

        vTaskDelayUntil(&lastWake, period);
    }
}
