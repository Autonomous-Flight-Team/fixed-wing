#include "tasks.h"
#include "hardware.h"

void MavlinkHeartbeatTask(void *pvParameters) {
    (void)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(1000);  // 1 Hz heartbeat
    TickType_t lastWake = xTaskGetTickCount();
    const uint8_t baseMode = MAV_MODE_GUIDED_ARMED;
    const uint32_t customMode = 10U;
    const uint8_t systemState = MAV_STATE_ACTIVE;

    for (;;) {
        mavlink_message_t msg = {};
        mavlink_msg_heartbeat_pack(
            MAVLINK_SYSTEM_ID,
            MAVLINK_COMPONENT_ID,
            &msg,
            MAV_TYPE_FIXED_WING,
            MAV_AUTOPILOT_GENERIC,
            baseMode,
            customMode,
            systemState
        );

        uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = {};
        const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
        MAVLINK_SERIAL_900.write(buffer, len);
        Serial.println("[MAVLINK] Heartbeat sent (AUTO)");

        vTaskDelayUntil(&lastWake, period);
    }
}
