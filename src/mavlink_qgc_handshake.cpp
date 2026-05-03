/*
Authors:
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"
#include "hardware.h"
#include <Arduino.h>
#include <string.h>

// Bare-bones QGC handshake task.
// Handles: heartbeat detection, arm/disarm, autopilot version,
// param list (with aux channel mapping), mission ack, timesync.
// Advertises as a custom autopilot with joystick/aux support
// so QGC exposes full controller configuration UI.

namespace
{

    // ── Parameter table ──────────────────────────────────────────────────────────
    // Advertise enough for QGC to show joystick config with aux/switch channels.
    struct ParamDef
    {
        const char *name;
        float value;
        uint8_t type;
    };

    static ParamDef gParams[] = {
        // Identity
        {"SYSID_THISMAV", static_cast<float>(MAVLINK_SYSTEM_ID), MAV_PARAM_TYPE_UINT8},
        {"MAV_TYPE", static_cast<float>(MAV_TYPE_FIXED_WING), MAV_PARAM_TYPE_UINT8},

        // Disables pre-arm RC checks so QGC doesn't block joystick use
        {"ARMING_CHECK", 0.0f, MAV_PARAM_TYPE_INT32},

        // 0 = RC transmitter, 1 = joystick/aux, 2 = both
        // Set 1 so QGC activates the joystick tab and aux channel config
        {"COM_RC_IN_MODE", 1.0f, MAV_PARAM_TYPE_INT32},

        // Primary axis mappings (QGC joystick config reads these)
        {"RCMAP_ROLL", 1.0f, MAV_PARAM_TYPE_INT32},
        {"RCMAP_PITCH", 2.0f, MAV_PARAM_TYPE_INT32},
        {"RCMAP_THROTTLE", 3.0f, MAV_PARAM_TYPE_INT32},
        {"RCMAP_YAW", 4.0f, MAV_PARAM_TYPE_INT32},

        // Aux channels — these unlock the "Buttons" and switch assignment UI in QGC
        {"RC_MAP_AUX1", 5.0f, MAV_PARAM_TYPE_INT32},
        {"RC_MAP_AUX2", 6.0f, MAV_PARAM_TYPE_INT32},
        {"RC_MAP_AUX3", 7.0f, MAV_PARAM_TYPE_INT32},
        {"RC_MAP_AUX4", 8.0f, MAV_PARAM_TYPE_INT32},
        {"RC_MAP_AUX5", 9.0f, MAV_PARAM_TYPE_INT32},
        {"RC_MAP_AUX6", 10.0f, MAV_PARAM_TYPE_INT32},

        // RC channel calibration — QGC requires these to show the channel sliders
        {"RC1_MIN", 1000.0f, MAV_PARAM_TYPE_INT32},
        {"RC1_TRIM", 1500.0f, MAV_PARAM_TYPE_INT32},
        {"RC1_MAX", 2000.0f, MAV_PARAM_TYPE_INT32},
        {"RC1_REV", 1.0f, MAV_PARAM_TYPE_INT32},
        {"RC2_MIN", 1000.0f, MAV_PARAM_TYPE_INT32},
        {"RC2_TRIM", 1500.0f, MAV_PARAM_TYPE_INT32},
        {"RC2_MAX", 2000.0f, MAV_PARAM_TYPE_INT32},
        {"RC2_REV", 1.0f, MAV_PARAM_TYPE_INT32},
        {"RC3_MIN", 1000.0f, MAV_PARAM_TYPE_INT32},
        {"RC3_TRIM", 1000.0f, MAV_PARAM_TYPE_INT32},
        {"RC3_MAX", 2000.0f, MAV_PARAM_TYPE_INT32},
        {"RC3_REV", 1.0f, MAV_PARAM_TYPE_INT32},
        {"RC4_MIN", 1000.0f, MAV_PARAM_TYPE_INT32},
        {"RC4_TRIM", 1500.0f, MAV_PARAM_TYPE_INT32},
        {"RC4_MAX", 2000.0f, MAV_PARAM_TYPE_INT32},
        {"RC4_REV", 1.0f, MAV_PARAM_TYPE_INT32},
        // Aux channels 5-10
        {"RC5_MIN", 1000.0f, MAV_PARAM_TYPE_INT32},
        {"RC5_TRIM", 1500.0f, MAV_PARAM_TYPE_INT32},
        {"RC5_MAX", 2000.0f, MAV_PARAM_TYPE_INT32},
        {"RC5_REV", 1.0f, MAV_PARAM_TYPE_INT32},
        {"RC6_MIN", 1000.0f, MAV_PARAM_TYPE_INT32},
        {"RC6_TRIM", 1500.0f, MAV_PARAM_TYPE_INT32},
        {"RC6_MAX", 2000.0f, MAV_PARAM_TYPE_INT32},
        {"RC6_REV", 1.0f, MAV_PARAM_TYPE_INT32},
        {"RC7_MIN", 1000.0f, MAV_PARAM_TYPE_INT32},
        {"RC7_TRIM", 1500.0f, MAV_PARAM_TYPE_INT32},
        {"RC7_MAX", 2000.0f, MAV_PARAM_TYPE_INT32},
        {"RC7_REV", 1.0f, MAV_PARAM_TYPE_INT32},
        {"RC8_MIN", 1000.0f, MAV_PARAM_TYPE_INT32},
        {"RC8_TRIM", 1500.0f, MAV_PARAM_TYPE_INT32},
        {"RC8_MAX", 2000.0f, MAV_PARAM_TYPE_INT32},
        {"RC8_REV", 1.0f, MAV_PARAM_TYPE_INT32},
        {"RC9_MIN", 1000.0f, MAV_PARAM_TYPE_INT32},
        {"RC9_TRIM", 1500.0f, MAV_PARAM_TYPE_INT32},
        {"RC9_MAX", 2000.0f, MAV_PARAM_TYPE_INT32},
        {"RC9_REV", 1.0f, MAV_PARAM_TYPE_INT32},
        {"RC10_MIN", 1000.0f, MAV_PARAM_TYPE_INT32},
        {"RC10_TRIM", 1500.0f, MAV_PARAM_TYPE_INT32},
        {"RC10_MAX", 2000.0f, MAV_PARAM_TYPE_INT32},
        {"RC10_REV", 1.0f, MAV_PARAM_TYPE_INT32},
    };
    static constexpr int16_t kParamCount = static_cast<int16_t>(sizeof(gParams) / sizeof(gParams[0]));

    // ── Helpers ───────────────────────────────────────────────────────────────────
    void Send(const mavlink_message_t &msg)
    {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {};
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        MavlinkSerial900Write(buf, len);
    }

    void SendAck(uint16_t cmd, uint8_t result, uint8_t sysid, uint8_t compid)
    {
        mavlink_message_t msg = {};
        mavlink_command_ack_t ack = {};
        ack.command = cmd;
        ack.result = result;
        ack.target_system = sysid;
        ack.target_component = compid;
        mavlink_msg_command_ack_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &ack);
        Send(msg);
    }

    void SendAutopilotVersion(uint8_t sysid, uint8_t compid)
    {
        mavlink_message_t msg = {};
        mavlink_autopilot_version_t v = {};
        // Advertise joystick/manual-input capability explicitly.
        // MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT is required for QGC param UI.
        // MAV_PROTOCOL_CAPABILITY_COMMAND_INT unlocks COMMAND_INT arm path.
        v.capabilities =
            MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
            MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
        mavlink_msg_autopilot_version_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &v);
        Send(msg);
        SendAck(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, MAV_RESULT_ACCEPTED, sysid, compid);
    }

    void SendParam(int16_t idx)
    {
        if (idx < 0 || idx >= kParamCount)
            return;
        mavlink_message_t msg = {};
        mavlink_param_value_t p = {};
        p.param_value = gParams[idx].value;
        p.param_count = static_cast<uint16_t>(kParamCount);
        p.param_index = static_cast<uint16_t>(idx);
        p.param_type = gParams[idx].type;
        memset(p.param_id, 0, sizeof(p.param_id));
        strncpy(p.param_id, gParams[idx].name, sizeof(p.param_id) - 1U);
        mavlink_msg_param_value_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &p);
        Send(msg);
    }

    int16_t FindParam(const char *name)
    {
        for (int16_t i = 0; i < kParamCount; ++i)
        {
            if (strncmp(gParams[i].name, name, 16) == 0)
                return i;
        }
        return -1;
    }

    void SetArmed(bool armed)
    {
        if (mavlinkDataMutex)
            xSemaphoreTake(mavlinkDataMutex, portMAX_DELAY);
        mavlinkVehicleArmed = armed;
        mavlinkVehicleBaseMode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        if (armed)
        {
            mavlinkVehicleBaseMode |= MAV_MODE_FLAG_SAFETY_ARMED;
            mavlinkVehicleSystemStatus = MAV_STATE_ACTIVE;
        }
        else
        {
            mavlinkVehicleBaseMode &= static_cast<uint8_t>(~MAV_MODE_FLAG_SAFETY_ARMED);
            mavlinkVehicleSystemStatus = MAV_STATE_STANDBY;
        }
        if (mavlinkDataMutex)
            xSemaphoreGive(mavlinkDataMutex);
        Serial.println(armed ? "[QGC] Armed" : "[QGC] Disarmed");
    }

    // ── Packet dispatcher ─────────────────────────────────────────────────────────
    void HandlePacket(const MavlinkRxPacket_t &pkt)
    {
        switch (pkt.msg.msgid)
        {

        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_heartbeat_t hb = {};
            mavlink_msg_heartbeat_decode(&pkt.msg, &hb);
            if (hb.type == MAV_TYPE_GCS)
            {
                if (mavlinkDataMutex)
                    xSemaphoreTake(mavlinkDataMutex, portMAX_DELAY);
                mavlinkGcsPresent = true;
                if (mavlinkDataMutex)
                    xSemaphoreGive(mavlinkDataMutex);
            }
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            mavlink_command_long_t cmd = {};
            mavlink_msg_command_long_decode(&pkt.msg, &cmd);

            switch (cmd.command)
            {
            case MAV_CMD_COMPONENT_ARM_DISARM:
                SetArmed(cmd.param1 > 0.5f);
                SendAck(cmd.command, MAV_RESULT_ACCEPTED, pkt.msg.sysid, pkt.msg.compid);
                break;

            case MAV_CMD_DO_SET_MODE:
                // Accept and echo back — no flight mode switching needed for manual control
                SendAck(cmd.command, MAV_RESULT_ACCEPTED, pkt.msg.sysid, pkt.msg.compid);
                break;

            case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            case MAV_CMD_REQUEST_MESSAGE:
                SendAutopilotVersion(pkt.msg.sysid, pkt.msg.compid);
                break;

            default:
                SendAck(cmd.command, MAV_RESULT_UNSUPPORTED, pkt.msg.sysid, pkt.msg.compid);
                break;
            }
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_INT:
        {
            mavlink_command_int_t cmd = {};
            mavlink_msg_command_int_decode(&pkt.msg, &cmd);
            if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM)
            {
                SetArmed(cmd.param1 > 0.5f);
                SendAck(cmd.command, MAV_RESULT_ACCEPTED, pkt.msg.sysid, pkt.msg.compid);
            }
            else
            {
                SendAck(cmd.command, MAV_RESULT_UNSUPPORTED, pkt.msg.sysid, pkt.msg.compid);
            }
            break;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
            for (int16_t i = 0; i < kParamCount; ++i)
            {
                SendParam(i);
            }
            break;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        {
            mavlink_param_request_read_t req = {};
            mavlink_msg_param_request_read_decode(&pkt.msg, &req);
            if (req.param_index >= 0 && req.param_index < kParamCount)
            {
                SendParam(req.param_index);
            }
            else
            {
                req.param_id[15] = '\0';
                const int16_t idx = FindParam(req.param_id);
                if (idx >= 0)
                    SendParam(idx);
            }
            break;
        }

        case MAVLINK_MSG_ID_PARAM_SET:
        {
            mavlink_param_set_t req = {};
            mavlink_msg_param_set_decode(&pkt.msg, &req);
            req.param_id[15] = '\0';
            const int16_t idx = FindParam(req.param_id);
            if (idx >= 0)
            {
                gParams[idx].value = req.param_value;
                gParams[idx].type = req.param_type;
                SendParam(idx);
            }
            break;
        }

        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        {
            mavlink_mission_request_list_t req = {};
            mavlink_msg_mission_request_list_decode(&pkt.msg, &req);
            mavlink_message_t msg = {};
            mavlink_mission_count_t cnt = {};
            cnt.target_system = pkt.msg.sysid;
            cnt.target_component = pkt.msg.compid;
            cnt.count = 0U;
            cnt.mission_type = req.mission_type;
            mavlink_msg_mission_count_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &cnt);
            Send(msg);
            break;
        }

        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        {
            mavlink_mission_clear_all_t req = {};
            mavlink_msg_mission_clear_all_decode(&pkt.msg, &req);
            mavlink_message_t msg = {};
            mavlink_mission_ack_t ack = {};
            ack.target_system = pkt.msg.sysid;
            ack.target_component = pkt.msg.compid;
            ack.type = MAV_MISSION_ACCEPTED;
            ack.mission_type = req.mission_type;
            mavlink_msg_mission_ack_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &ack);
            Send(msg);
            break;
        }

        case MAVLINK_MSG_ID_TIMESYNC:
        {
            mavlink_timesync_t ts = {};
            mavlink_msg_timesync_decode(&pkt.msg, &ts);
            if (ts.tc1 == 0)
            {
                mavlink_timesync_t resp = {};
                resp.tc1 = ts.ts1;
                resp.ts1 = static_cast<int64_t>(micros()) * 1000LL;
                mavlink_message_t msg = {};
                mavlink_msg_timesync_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &resp);
                Send(msg);
            }
            break;
        }

        case MAVLINK_MSG_ID_PING:
        {
            //(void)MavlinkHandlePingForLatency(pkt);
            break;
        }

        default:
            break;
        }
    }

} // namespace

void MavlinkQgcHandshakeTask(void *pvParameters)
{
    (void)pvParameters;

    MavlinkRxPacket_t pkt = {};
    for (;;)
    {
        if (mavlinkQgcHandshakeQueue == nullptr)
        {
            vTaskDelay(pdMS_TO_TICKS(RX_SLOW_MS_PER_TICK));
            continue;
        }
        if (xQueueReceive(mavlinkQgcHandshakeQueue, &pkt, pdMS_TO_TICKS(RX_SLOW_MS_PER_TICK)) == pdTRUE)
        {
            HandlePacket(pkt);
        }
    }
}