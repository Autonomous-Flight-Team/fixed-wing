/*
Authors:
Colin Faletto github.com/faletto
*/

#include "tasks.h"
#include "hardware.h"

namespace {
void SendMavlinkMessage(const mavlink_message_t &msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = {};
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    MAVLINK_SERIAL_900.write(buffer, len);
}

float ReadAltitudeMeters() {
    float altitudeM = 0.0f;
    if (dataMutex != nullptr) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            altitudeM = sensorData.altitude;
            xSemaphoreGive(dataMutex);
        }
    } else {
        altitudeM = sensorData.altitude;
    }
    return altitudeM;
}

void SendAltitudeVfrHud(float altitudeM) {
    mavlink_message_t msg = {};
    mavlink_vfr_hud_t hud = {};

    // Only altitude is sourced here; remaining fields are intentionally neutral.
    hud.airspeed = 0.0f;
    hud.groundspeed = 0.0f;
    hud.heading = 0;
    hud.throttle = 0U;
    hud.alt = altitudeM;
    hud.climb = 0.0f;

    mavlink_msg_vfr_hud_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &hud);
    SendMavlinkMessage(msg);
}
}  // namespace

void MavlinkAltitudeTask(void *pvParameters) {
    (void)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(100);  // 10 Hz altitude telemetry
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        Serial.println("Sending altitude packet");
        const float altitudeM = ReadAltitudeMeters();
        SendAltitudeVfrHud(altitudeM);
        vTaskDelayUntil(&lastWake, period);
    }
}
