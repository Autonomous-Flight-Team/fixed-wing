/*
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"

#include <arduino_freertos.h>
#include <Arduino.h>
#include <queue.h>

void QuadcopterOriginPacketForwardTask (void *pvParameters){
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(RX_SLOW_MS_PER_TICK);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    MavlinkRxPacket_t pkt;
    for (;;){
        if (xQueueReceive(mavlinkRx24_QuadcopterOrigin_ForwardQueue, &pkt, 0) == pdTRUE){
            uint16_t len = mavlink_msg_to_send_buffer(buf, &pkt.msg);
            MavlinkSerial900Write(buf, len);
        }
        vTaskDelayUntil(&lastWake, freq);
    }
    
}