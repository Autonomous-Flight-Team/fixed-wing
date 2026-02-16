/*
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"
#include "hardware.h"

#include <arduino_freertos.h>
#include <queue.h>

// Stolen from mavlink_rx_tasks.cpp - perhaps refactor to not repeat
// RX loops are paced to avoid starving other tasks; dispatch is slightly faster.
const int SLOW_MS_PER_TICK = 2; // 500 Hz poll
const int FAST_MS_PER_TICK = 1; // 1000 Hz poll

void RxMavlinkProcess900PacketTask(void *pvParameters) {
    //TickType_t lastWake = xTaskGetTickCount();
    //const TickType_t freq = pdMS_TO_TICKS(SLOW_MS_PER_TICK);

    MavlinkRxPacket_t pkt;
    for (;;) {
        if (xQueueReceive(mavlinkRxQueue900, &pkt, pdMS_TO_TICKS(SLOW_MS_PER_TICK)) == pdTRUE) {
            switch (pkt.msg.msgid) {
                // Parsing and storing pkt message based on their msgID into globally accessible variables
                case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {
                    mavlink_msg_set_position_target_global_int_decode(&pkt.msg, &set_global_position);
                    break;

                }
                // This case may be irrelevant - ATTTIUDE calculated locallly by flight controller
                // Not included globally
                case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: {
                    mavlink_set_attitude_target_t attitude;
                    mavlink_msg_set_attitude_target_decode(&pkt.msg, &attitude);
                    // TODO: Store attitude setpoint (q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, type_mask).
                    break;
                }

                case MAVLINK_MSG_ID_MANUAL_CONTROL: {
                    mavlink_msg_manual_control_decode(&pkt.msg, &manual_control_data);
                    // TODO: Store manual control inputs (x, y, z, r, buttons).
                    break;
                }
                case MAVLINK_MSG_ID_COMMAND_LONG: {
                    mavlink_msg_command_long_decode(&pkt.msg, &specific_cmds);
                    // TODO: Handle arm/disarm, takeoff, land, etc. from QGC.
                    break;
                }

                // Perhaps change for custom command
                // Not global as of now
                case MAVLINK_MSG_ID_SET_MODE: {
                    mavlink_msg_set_mode_decode(&pkt.msg, &mode);
                    // TODO: Handle mode change from QGC.
                    break;
                }
                
                default:
                    break;
            }
                
        }
        vTaskDelay(pdMS_TO_TICKS(FAST_MS_PER_TICK));
    }
}
