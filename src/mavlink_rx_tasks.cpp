/*
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"
#include "hardware.h"

#include <arduino_freertos.h>
#include <queue.h>

#define MAVLINK_COMM_900 MAVLINK_COMM_0
#define MAVLINK_COMM_24  MAVLINK_COMM_1

// TODO: Update these UARTs to match our wiring
#define MAVLINK_SERIAL_900 Serial2
#define MAVLINK_SERIAL_24  Serial3

QueueHandle_t mavlinkRxQueue900 = nullptr;
QueueHandle_t mavlinkRxQueue24  = nullptr;
volatile uint32_t mavlinkRxDrop900 = 0;
volatile uint32_t mavlinkRxDrop24  = 0;
volatile mavlink_message_t mavlinkLastTelemetry = {};
volatile uint32_t mavlinkTelemetryCount = 0;

const int SLOW_MS_PER_TICK = 2; // 500 Hz poll
const int FAST_MS_PER_TICK = 1; // 1000 Hz poll

void MavlinkRx900Task(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(SLOW_MS_PER_TICK); 
    mavlink_message_t msg;
    mavlink_status_t status;

    for (;;) {
        while (MAVLINK_SERIAL_900.available() > 0) {
            uint8_t c = static_cast<uint8_t>(MAVLINK_SERIAL_900.read());
            if (mavlink_parse_char(MAVLINK_COMM_900, c, &msg, &status)) {
                MavlinkRxPacket_t pkt;
                pkt.link = LINK_900MHZ;
                pkt.msg = msg;
                if (xQueueSend(mavlinkRxQueue900, &pkt, 0) != pdTRUE) {
                    ++mavlinkRxDrop900;
                }
            }
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}

void MavlinkRx24Task(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(SLOW_MS_PER_TICK);
    mavlink_message_t msg;
    mavlink_status_t status;

    for (;;) {
        while (MAVLINK_SERIAL_24.available() > 0) {
            uint8_t c = static_cast<uint8_t>(MAVLINK_SERIAL_24.read());
            if (mavlink_parse_char(MAVLINK_COMM_24, c, &msg, &status)) {
                MavlinkRxPacket_t pkt;
                pkt.link = LINK_24GHZ;
                pkt.msg = msg;
                if (xQueueSend(mavlinkRxQueue24, &pkt, 0) != pdTRUE) {
                    ++mavlinkRxDrop24;
                }
            }
        }
        vTaskDelayUntil(&lastWake, freq);
    }
}

void MavlinkControlDispatchTask(void *pvParameters) {
    MavlinkRxPacket_t pkt;
    for (;;) {
        if (xQueueReceive(mavlinkRxQueue900, &pkt, pdMS_TO_TICKS(SLOW_MS_PER_TICK)) == pdTRUE) {
            switch (pkt.msg.msgid) {
                case MAVLINK_MSG_ID_MANUAL_CONTROL: {
                    mavlink_manual_control_t mc;
                    mavlink_msg_manual_control_decode(&pkt.msg, &mc);
                    // TODO: Apply manual control input.
                    break;
                }
                default:
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(FAST_MS_PER_TICK));
    }
}

void MavlinkTelemetryDispatchTask(void *pvParameters) {
    MavlinkRxPacket_t pkt;
    for (;;) {
        if (xQueueReceive(mavlinkRxQueue24, &pkt, pdMS_TO_TICKS(SLOW_MS_PER_TICK)) == pdTRUE) {
            // Store last raw telemetry message for flexible handling.
            mavlinkLastTelemetry = pkt.msg;
            ++mavlinkTelemetryCount;
        }

        vTaskDelay(pdMS_TO_TICKS(FAST_MS_PER_TICK));
    }
}
