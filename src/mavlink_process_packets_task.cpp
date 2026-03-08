/*
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"
#include "hardware.h"

#include <arduino_freertos.h>
#include <Arduino.h>
#include <queue.h>

// Stolen from mavlink_rx_tasks.cpp - perhaps refactor to not repeat
// RX loops are paced to avoid starving other tasks; dispatch is slightly faster.

namespace {
uint32_t g_lastManualPrintMs = 0U;
constexpr uint8_t kControlPrintArmedOnly = 1U;
constexpr uint8_t kControlPrintAlways = 2U;

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

void UpdateManualInputMetadata() {
    taskENTER_CRITICAL();
    mavlinkLastManualInputMs = millis();
    ++mavlinkManualInputFrameCount;
    taskEXIT_CRITICAL();
}

void PrintManualInputsIfDue(const char *sourceTag) {
    const uint32_t nowMs = millis();
    if ((nowMs - g_lastManualPrintMs) < 200U) {
        return;
    }

    mavlink_manual_control_t mcSnapshot = {};
    bool armed = false;
    uint8_t printMode = kControlPrintArmedOnly;
    taskENTER_CRITICAL();
    mcSnapshot = manual_control_data;
    armed = mavlinkVehicleArmed;
    printMode = mavlinkControlPrintMode;
    taskEXIT_CRITICAL();

    if (printMode == kControlPrintArmedOnly && !armed) {
        return;
    }
    if (printMode != kControlPrintArmedOnly && printMode != kControlPrintAlways) {
        return;
    }

    g_lastManualPrintMs = nowMs;
    Serial.print("[MAVLINK][CTRL][");
    Serial.print(sourceTag);
    Serial.print("] x=");
    Serial.print(mcSnapshot.x);
    Serial.print(" y=");
    Serial.print(mcSnapshot.y);
    Serial.print(" z=");
    Serial.print(mcSnapshot.z);
    Serial.print(" r=");
    Serial.print(mcSnapshot.r);
    Serial.print(" buttons=");
    Serial.println(static_cast<unsigned long>(mcSnapshot.buttons));
}
}  // namespace

void RxMavlinkProcess900PacketTask(void *pvParameters) {
    //TickType_t lastWake = xTaskGetTickCount();
    //const TickType_t freq = pdMS_TO_TICKS(SLOW_MS_PER_TICK);

    MavlinkRxPacket_t pkt = {};
    for (;;) {
        bool handledPacket = false;
        for (uint8_t queueIndex = 0U; queueIndex < 2U; ++queueIndex) {
            QueueHandle_t queueHandle = (queueIndex == 0U) ? mavlinkRxQueue900 : mavlinkRxQueue24;
            if (queueHandle == nullptr) {
                continue;
            }

            if (xQueueReceive(queueHandle, &pkt, 0) != pdTRUE) {
                continue;
            }

            handledPacket = true;
            HandleQgcHandshakePacket(pkt);
            switch (pkt.msg.msgid) {
                // Parsing and storing pkt message based on their msgID into globally accessible variables
                case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {
                    mavlink_msg_set_position_target_global_int_decode(&pkt.msg, &set_global_position);
                    break;
                }

                // This case may be irrelevant - ATTTIUDE calculated locally by flight controller
                case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: {
                    break;
                }

                case MAVLINK_MSG_ID_MANUAL_CONTROL: {
                    taskENTER_CRITICAL();
                    mavlink_msg_manual_control_decode(&pkt.msg, &manual_control_data);
                    taskEXIT_CRITICAL();
                    UpdateManualInputMetadata();
                    PrintManualInputsIfDue("MAN");
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
                    mc.buttons = 0U;
                    taskENTER_CRITICAL();
                    manual_control_data = mc;
                    taskEXIT_CRITICAL();
                    UpdateManualInputMetadata();
                    PrintManualInputsIfDue("RCOVR");
                    break;
                }

                case MAVLINK_MSG_ID_COMMAND_LONG: {
                    mavlink_msg_command_long_decode(&pkt.msg, &specific_cmds);
                    break;
                }

                case MAVLINK_MSG_ID_SET_MODE: {
                    mavlink_msg_set_mode_decode(&pkt.msg, &mode);
                    break;
                }

                default:
                    break;
            }
        }

        if (!handledPacket) {
            vTaskDelay(pdMS_TO_TICKS(RX_FAST_MS_PER_TICK));
        }
    }
}
