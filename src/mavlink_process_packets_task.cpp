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

void MavlinkProcess900PacketTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(SLOW_MS_PER_TICK);

    MavlinkRxPacket_t pkt;
    for (;;) {
        if (xQueueReceive(mavlinkRxQueue900, &pkt, pdMS_TO_TICKS(SLOW_MS_PER_TICK)) == pdTRUE) {
            switch (pkt.msg.msgid) {
                case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {
                    mavlink_set_position_target_global_int_t sp;
                    mavlink_msg_set_position_target_global_int_decode(&pkt.msg, &sp);
                    // TODO: Store position target setpoint (lat_int, lon_int, alt, type_mask, coordinate_frame).
                    break;
                }
                case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: {
                    mavlink_set_attitude_target_t att;
                    mavlink_msg_set_attitude_target_decode(&pkt.msg, &att);
                    // TODO: Store attitude setpoint (q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, type_mask).
                    break;
                }
                case MAVLINK_MSG_ID_MANUAL_CONTROL: {
                    mavlink_manual_control_t mc;
                    mavlink_msg_manual_control_decode(&pkt.msg, &mc);
                    // TODO: Store manual control inputs (x, y, z, r, buttons).
                    break;
                }
                case MAVLINK_MSG_ID_COMMAND_LONG: {
                    mavlink_command_long_t cmd;
                    mavlink_msg_command_long_decode(&pkt.msg, &cmd);
                    // TODO: Handle arm/disarm, takeoff, land, etc. from QGC.
                    break;
                }
                case MAVLINK_MSG_ID_SET_MODE: {
                    mavlink_set_mode_t mode;
                    mavlink_msg_set_mode_decode(&pkt.msg, &mode);
                    // TODO: Handle mode change from QGC.
                    break;
                }
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    // Optional: track link health or system type.
                    break;
                }
                default:
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(FAST_MS_PER_TICK));
    }
}
