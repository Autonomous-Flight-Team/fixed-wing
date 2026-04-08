#include "tasks.h"
#include "hardware.h"

#include <Arduino.h>

void Serial3LoopbackSelfTestTask(void *pvParameters) {
    (void)pvParameters;

    uint32_t seq = 0U;
    uint32_t lastParsedCount = mavlinkRxParsed24Count;

    const TickType_t period = pdMS_TO_TICKS(1000);
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        mavlink_message_t msg = {};
        mavlink_msg_ping_pack(
            MAVLINK_SYSTEM_ID,
            MAVLINK_COMPONENT_ID,
            &msg,
            static_cast<uint64_t>(micros()),
            seq,
            MAVLINK_SYSTEM_ID,
            MAVLINK_COMPONENT_ID
        );

        uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = {};
        const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
        MAVLINK_SERIAL_24.write(buffer, len);

        // Give RX24 task a short window to parse echoed loopback bytes.
        vTaskDelay(pdMS_TO_TICKS(50));

        const uint32_t parsedCount = mavlinkRxParsed24Count;
        const uint32_t delta = parsedCount - lastParsedCount;

        Serial.print("[SER3][LOOPBACK] tx_seq=");
        Serial.print(seq);
        Serial.print(" parsed_delta=");
        Serial.print(delta);
        Serial.print(" parsed_total=");
        Serial.print(parsedCount);

        if (delta > 0U) {
            Serial.println(" PASS");
        } else {
            Serial.println(" WAITING");
        }

        lastParsedCount = parsedCount;
        ++seq;
        vTaskDelayUntil(&lastWake, period);
    }
}
