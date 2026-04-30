/*
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"

#include <arduino_freertos.h>
#include <Arduino.h>
#include <queue.h>

namespace {
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

int16_t PwmToAxis(uint16_t pwm) {
    if (pwm == 0U || pwm == 65535U) {
        return 0;
    }
    const int32_t centered = static_cast<int32_t>(pwm) - 1500;
    const int32_t scaled = centered * 2;
    if (scaled > 1000) {
        return 1000;
    }
    if (scaled < -1000) {
        return -1000;
    }
    return static_cast<int16_t>(scaled);
}

int16_t PwmToThrottle(uint16_t pwm) {
    if (pwm == 0U || pwm == 65535U) {
        return 500;
    }
    int32_t scaled = static_cast<int32_t>(pwm) - 1000;
    if (scaled > 1000) {
        scaled = 1000;
    }
    if (scaled < 0) {
        scaled = 0;
    }
    return static_cast<int16_t>(scaled);
}

void UpdateManualInputMetadataLocked() {
    mavlinkLastManualInputMs = millis();
    mavlinkLastManualInputUs = micros();
    ++mavlinkManualInputFrameCount;
}

void ProcessPacket(const MavlinkRxPacket_t &pkt) {
    switch (pkt.msg.msgid) {
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {
            LockMavlinkData();
            mavlink_msg_set_position_target_global_int_decode(&pkt.msg, &set_global_position);
            UnlockMavlinkData();
            break;
        }

        case MAVLINK_MSG_ID_MANUAL_CONTROL: {
            LockMavlinkData();
            mavlink_msg_manual_control_decode(&pkt.msg, &manual_control_data);
            ConstructLogAndFillQueue(manual_control_data);
            // Serial.print("1: ");

            // Serial.println(manual_control_data.aux1);
            // Serial.print("2: ");
            
            // Serial.println(manual_control_data.aux2);
            // Serial.print("3: ");

            // Serial.println(manual_control_data.aux3);
            // Serial.print("4: ");

            // Serial.println(manual_control_data.aux4);
            // Serial.print("5: ");

            // Serial.println(manual_control_data.aux5);
            // Serial.print("6: ");

            // Serial.println(manual_control_data.aux6);
            Serial.print("buttons: ");

            Serial.println(manual_control_data.buttons, 2);
            // Serial.print("buttons2: ");

            // Serial.println(manual_control_data.buttons2);

            UpdateManualInputMetadataLocked();
            UnlockMavlinkData();
            break;
        }

        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
            mavlink_rc_channels_override_t rc = {};
            mavlink_msg_rc_channels_override_decode(&pkt.msg, &rc);
            mavlink_manual_control_t mc = {};
            mc.x = PwmToAxis(rc.chan2_raw);      // pitch
            mc.y = PwmToAxis(rc.chan1_raw);      // roll
            mc.r = PwmToAxis(rc.chan4_raw);      // yaw
            mc.z = PwmToThrottle(rc.chan3_raw);  // throttle [0..1000]
            Serial.println("Updates from RC Channel Override");
            Serial.print("Channel 5: ");
            Serial.println(rc.chan5_raw);
            Serial.print("Channel 6: ");
            Serial.println(rc.chan6_raw);
            mc.aux1 = rc.chan5_raw;
            mc.aux2 = rc.chan6_raw;
            mc.buttons = 0U;
            LockMavlinkData();
            manual_control_data = mc;
            UpdateManualInputMetadataLocked();
            UnlockMavlinkData();
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_LONG: {
            LockMavlinkData();
            mavlink_msg_command_long_decode(&pkt.msg, &specific_cmds);
            UnlockMavlinkData();
            break;
        }

        case MAVLINK_MSG_ID_SET_MODE: {
            LockMavlinkData();
            mavlink_msg_set_mode_decode(&pkt.msg, &mode);
            UnlockMavlinkData();
            break;
        }

        default:
            break;
    }
}
}  // namespace

void RxMavlinkProcess900PacketTask(void *pvParameters) {
    (void)pvParameters;

    MavlinkRxPacket_t pkt = {};
    for (;;) {
        if (mavlinkRxQueue900 == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(RX_FAST_MS_PER_TICK));
            continue;
        }

        if (xQueueReceive(mavlinkRxQueue900, &pkt, 0) == pdTRUE) {
            if (mavlinkQgcHandshakeQueue != nullptr) {
                (void)xQueueSend(mavlinkQgcHandshakeQueue, &pkt, 0);
            }
            ProcessPacket(pkt);
        } else {
            vTaskDelay(pdMS_TO_TICKS(RX_FAST_MS_PER_TICK));
        }
    }
}

void RxMavlinkProcess24PacketTask(void *pvParameters){
    (void) pvParameters;

    MavlinkRxPacket_t pkt = {};
    for (;;) {
        if (mavlinkRxQueue24 == nullptr){
            vTaskDelay(RX_FAST_MS_PER_TICK);
            continue;
        }

        if (xQueueReceive(mavlinkRxQueue24, &pkt, 0) == pdTRUE){
            xQueueSend(mavlinkRx24_QuadcopterOrigin_ForwardQueue, &pkt, 0);
        } else {
            vTaskDelay(pdMS_TO_TICKS(RX_FAST_MS_PER_TICK));
        }
    }
}
