/*
Authors:
Advik Sharma - github.com/jpyces
Colin Faletto - github.com/faletto
*/

#include "tasks.h"
#include "hardware.h"

#include <math.h>

namespace {
constexpr float kPi = 3.14159265359f;
constexpr float kRadToDeg = 57.2957795131f;
constexpr float kDegToRad = 0.01745329252f;
constexpr float kGravityMps2 = 9.80665f;

struct SensorSnapshot {
    IMUData_t imu = {};
    BaroData_t baro = {};
    GPSData_t gps = {};
    PitotData_t pitot = {};
};

struct TelemetryState {
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float yawDeg = 0.0f;
    float climbMps = 0.0f;
    float lastBaroAltM = 0.0f;
    bool haveBaroAlt = false;
};

void SendMavlinkMessage(const mavlink_message_t &msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = {};
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    MavlinkSerial900Write(buffer, len);
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

float WrapSignedAngleDeg(float angleDeg) {
    while (angleDeg < -180.0f) {
        angleDeg += 360.0f;
    }
    while (angleDeg > 180.0f) {
        angleDeg -= 360.0f;
    }
    return angleDeg;
}

SensorSnapshot ReadSensorSnapshot() {
    SensorSnapshot snapshot = {};
    if (dataMutex != nullptr && xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        snapshot.imu = imuData;
        snapshot.baro = baroData;
        snapshot.gps = gpsData;
        snapshot.pitot = pitotData;
        xSemaphoreGive(dataMutex);
    }
    return snapshot;
}

void UpdateTelemetryState(TelemetryState &state, const SensorSnapshot &snapshot, float dtSec) {
    state.rollDeg = WrapSignedAngleDeg(state.rollDeg + (snapshot.imu.gx * dtSec * kRadToDeg));
    state.pitchDeg = WrapSignedAngleDeg(state.pitchDeg + (snapshot.imu.gy * dtSec * kRadToDeg));
    state.yawDeg = WrapHeadingDeg(state.yawDeg + (snapshot.imu.gz * dtSec * kRadToDeg));

    const float accelMag = sqrtf(
        (snapshot.imu.ax * snapshot.imu.ax) +
        (snapshot.imu.ay * snapshot.imu.ay) +
        (snapshot.imu.az * snapshot.imu.az)
    );
    if (accelMag > 2.0f && accelMag < 15.0f) {
        const float accelRollDeg = atan2f(snapshot.imu.ay, snapshot.imu.az) * kRadToDeg;
        const float accelPitchDeg = atan2f(-snapshot.imu.ax, sqrtf((snapshot.imu.ay * snapshot.imu.ay) + (snapshot.imu.az * snapshot.imu.az))) * kRadToDeg;
        state.rollDeg = (state.rollDeg * 0.98f) + (accelRollDeg * 0.02f);
        state.pitchDeg = (state.pitchDeg * 0.98f) + (accelPitchDeg * 0.02f);
    }

    if (!state.haveBaroAlt) {
        state.lastBaroAltM = snapshot.baro.altitude;
        state.haveBaroAlt = true;
        state.climbMps = 0.0f;
    } else if (dtSec > 0.0f) {
        state.climbMps = (snapshot.baro.altitude - state.lastBaroAltM) / dtSec;
        state.lastBaroAltM = snapshot.baro.altitude;
    }
}

int16_t ToMilliG(float accelMps2) {
    return static_cast<int16_t>(Clampf((accelMps2 / kGravityMps2) * 1000.0f, -32768.0f, 32767.0f));
}

int16_t ToMilliRad(float gyroRadPerSec) {
    return static_cast<int16_t>(Clampf(gyroRadPerSec * 1000.0f, -32768.0f, 32767.0f));
}

void SendRawImu(const SensorSnapshot &snapshot) {
    mavlink_message_t msg = {};
    mavlink_raw_imu_t imu = {};
    imu.time_usec = static_cast<uint64_t>(millis()) * 1000ULL;
    imu.xacc = ToMilliG(snapshot.imu.ax);
    imu.yacc = ToMilliG(snapshot.imu.ay);
    imu.zacc = ToMilliG(snapshot.imu.az);
    imu.xgyro = ToMilliRad(snapshot.imu.gx);
    imu.ygyro = ToMilliRad(snapshot.imu.gy);
    imu.zgyro = ToMilliRad(snapshot.imu.gz);
    imu.xmag = 0;
    imu.ymag = 0;
    imu.zmag = 0;

    mavlink_msg_raw_imu_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &imu);
    SendMavlinkMessage(msg);
}

void SendScaledPressure(const SensorSnapshot &snapshot) {
    mavlink_message_t msg = {};
    mavlink_scaled_pressure_t pressure = {};
    pressure.time_boot_ms = millis();
    pressure.press_abs = snapshot.baro.pressure / 100.0f;
    pressure.press_diff = 0.0f;
    pressure.temperature = static_cast<int16_t>(snapshot.baro.temp * 100.0f);
    pressure.temperature_press_diff = static_cast<int16_t>(snapshot.baro.temp * 100.0f);

    mavlink_msg_scaled_pressure_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &pressure);
    SendMavlinkMessage(msg);
}

void SendGpsRawInt(const SensorSnapshot &snapshot, const TelemetryState &state) {
    mavlink_message_t msg = {};
    mavlink_gps_raw_int_t gps = {};
    gps.time_usec = static_cast<uint64_t>(millis()) * 1000ULL;
    gps.fix_type = 3U;
    gps.lat = static_cast<int32_t>(snapshot.gps.lat * 1.0e7);
    gps.lon = static_cast<int32_t>(snapshot.gps.lon * 1.0e7);
    gps.alt = static_cast<int32_t>(snapshot.gps.gps_altitude * 1000.0);
    gps.eph = 80U;
    gps.epv = 120U;
    gps.vel = static_cast<uint16_t>(Clampf(snapshot.gps.vs * 100.0f, 0.0f, 65535.0f));
    gps.cog = static_cast<uint16_t>(WrapHeadingDeg(state.yawDeg) * 100.0f);
    gps.satellites_visible = 12U;

    mavlink_msg_gps_raw_int_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &gps);
    SendMavlinkMessage(msg);
}

void SendGlobalPositionInt(const SensorSnapshot &snapshot, const TelemetryState &state) {
    mavlink_message_t msg = {};
    mavlink_global_position_int_t pos = {};
    const float yawRad = state.yawDeg * kDegToRad;
    const float groundSpeedMps = snapshot.gps.vs;
    pos.time_boot_ms = millis();
    pos.lat = static_cast<int32_t>(snapshot.gps.lat * 1.0e7);
    pos.lon = static_cast<int32_t>(snapshot.gps.lon * 1.0e7);
    pos.alt = static_cast<int32_t>(snapshot.gps.gps_altitude * 1000.0f);
    pos.relative_alt = static_cast<int32_t>(snapshot.baro.altitude * 1000.0f);
    pos.vx = static_cast<int16_t>(Clampf(cosf(yawRad) * groundSpeedMps * 100.0f, -32768.0f, 32767.0f));
    pos.vy = static_cast<int16_t>(Clampf(sinf(yawRad) * groundSpeedMps * 100.0f, -32768.0f, 32767.0f));
    pos.vz = static_cast<int16_t>(Clampf(-state.climbMps * 100.0f, -32768.0f, 32767.0f));
    pos.hdg = static_cast<uint16_t>(WrapHeadingDeg(state.yawDeg) * 100.0f);

    mavlink_msg_global_position_int_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &pos);
    SendMavlinkMessage(msg);
}

void SendAttitude(const SensorSnapshot &snapshot, const TelemetryState &state) {
    mavlink_message_t msg = {};
    mavlink_attitude_t attitude = {};
    attitude.time_boot_ms = millis();
    attitude.roll = state.rollDeg * kDegToRad;
    attitude.pitch = state.pitchDeg * kDegToRad;
    attitude.yaw = state.yawDeg * kDegToRad;
    attitude.rollspeed = snapshot.imu.gx;
    attitude.pitchspeed = snapshot.imu.gy;
    attitude.yawspeed = snapshot.imu.gz;

    mavlink_msg_attitude_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &attitude);
    SendMavlinkMessage(msg);
}

void SendVfrHud(const SensorSnapshot &snapshot, const TelemetryState &state) {
    mavlink_message_t msg = {};
    mavlink_vfr_hud_t hud = {};
    hud.airspeed = snapshot.pitot.ias_mps;
    hud.groundspeed = snapshot.gps.vs;
    hud.heading = static_cast<int16_t>(WrapHeadingDeg(state.yawDeg));
    hud.throttle = static_cast<uint16_t>(Clampf((snapshot.pitot.ias_mps / 30.0f) * 100.0f, 0.0f, 100.0f));
    hud.alt = snapshot.baro.altitude;
    hud.climb = state.climbMps;

    mavlink_msg_vfr_hud_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &hud);
    SendMavlinkMessage(msg);
}
}  // namespace

void MavlinkTelemetryTask(void *pvParameters) {
    (void)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(100);
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t lastUpdateMs = millis();
    TelemetryState telemetryState = {};

    for (;;) {
        const uint32_t nowMs = millis();
        float dtSec = static_cast<float>(nowMs - lastUpdateMs) / 1000.0f;
        lastUpdateMs = nowMs;
        dtSec = Clampf(dtSec, 0.01f, 0.2f);

        const SensorSnapshot snapshot = ReadSensorSnapshot();
        UpdateTelemetryState(telemetryState, snapshot, dtSec);

        SendRawImu(snapshot);
        SendScaledPressure(snapshot);
        SendGpsRawInt(snapshot, telemetryState);
        SendGlobalPositionInt(snapshot, telemetryState);
        SendAttitude(snapshot, telemetryState);
        SendVfrHud(snapshot, telemetryState);

        vTaskDelayUntil(&lastWake, period);
    }
}