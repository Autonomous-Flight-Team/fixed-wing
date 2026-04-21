/*
Authors:
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"
#include "hardware.h"
#include "mavlink_latency.h"
#include <Arduino.h>
#include <string.h>

// Design note:
// This file intentionally handles QGC handshake/protocol management only.
// Latency probing and PING RTT state live in mavlink_latency_probe_task.cpp.
namespace {
struct QgcParamDef {
    const char *name;
    float value;
    uint8_t type;
};

QgcParamDef gQgcParamTable[] = {
    {"SYSID_THISMAV", static_cast<float>(MAVLINK_SYSTEM_ID), MAV_PARAM_TYPE_UINT8},
    {"MAV_TYPE", static_cast<float>(MAV_TYPE_FIXED_WING), MAV_PARAM_TYPE_UINT8},
    {"ARMING_CHECK", 0.0f, MAV_PARAM_TYPE_INT32},
    {"QGC_SPOOF", 1.0f, MAV_PARAM_TYPE_INT32},
};
constexpr int16_t kQgcParamCount = static_cast<int16_t>(sizeof(gQgcParamTable) / sizeof(gQgcParamTable[0]));

void SendMavlinkMessage(const mavlink_message_t &msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = {};
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    MavlinkSerial900Write(buffer, len);
}

void SendCommandAck(uint16_t command, uint8_t result, uint8_t targetSystem, uint8_t targetComponent) {
    mavlink_message_t msg = {};
    mavlink_command_ack_t ack = {};
    ack.command = command;
    ack.result = result;
    ack.target_system = targetSystem;
    ack.target_component = targetComponent;

    mavlink_msg_command_ack_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &ack);
    SendMavlinkMessage(msg);
}

void SendMissionAck(uint8_t targetSystem, uint8_t targetComponent, uint8_t result, uint8_t missionType) {
    mavlink_message_t msg = {};
    mavlink_mission_ack_t ack = {};
    ack.target_system = targetSystem;
    ack.target_component = targetComponent;
    ack.type = result;
    ack.mission_type = missionType;

    mavlink_msg_mission_ack_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &ack);
    SendMavlinkMessage(msg);
}

void SendAutopilotVersion(uint8_t targetSystem, uint8_t targetComponent) {
    (void)targetSystem;
    (void)targetComponent;
    mavlink_message_t msg = {};
    mavlink_autopilot_version_t version = {};

    // Keep handshake metadata zeroed for QGC-facing test mode.
    version.capabilities = 0U;
    version.flight_sw_version = 0U;
    version.middleware_sw_version = 0U;
    version.os_sw_version = 0U;
    version.board_version = 0U;
    version.vendor_id = 0U;
    version.product_id = 0U;
    version.uid = 0U;

    mavlink_msg_autopilot_version_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &version);
    SendMavlinkMessage(msg);
}

void SendMissionCountZero(uint8_t targetSystem, uint8_t targetComponent, uint8_t missionType) {
    mavlink_message_t msg = {};
    mavlink_mission_count_t count = {};
    count.target_system = targetSystem;
    count.target_component = targetComponent;
    count.count = 0U;
    count.mission_type = missionType;

    mavlink_msg_mission_count_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &count);
    SendMavlinkMessage(msg);
}

void SendSingleParam(const QgcParamDef &def, int16_t index) {
    mavlink_message_t msg = {};
    mavlink_param_value_t param = {};
    param.param_value = def.value;
    param.param_count = static_cast<uint16_t>(kQgcParamCount);
    param.param_index = index;
    param.param_type = def.type;
    memset(param.param_id, 0, sizeof(param.param_id));
    strncpy(param.param_id, def.name, sizeof(param.param_id) - 1U);

    mavlink_msg_param_value_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &param);
    SendMavlinkMessage(msg);
}

void SendParamList() {
    for (int16_t idx = 0; idx < kQgcParamCount; ++idx) {
        SendSingleParam(gQgcParamTable[idx], idx);
    }
}

QgcParamDef *FindParamByName(const char *name) {
    for (int16_t idx = 0; idx < kQgcParamCount; ++idx) {
        if (strncmp(gQgcParamTable[idx].name, name, 16) == 0) {
            return &gQgcParamTable[idx];
        }
    }
    return nullptr;
}

void SetArmedState(bool armed) {
    if (mavlinkDataMutex != nullptr) {
        xSemaphoreTake(mavlinkDataMutex, portMAX_DELAY);
    }
    mavlinkVehicleArmed = armed;
    mavlinkVehicleBaseMode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (armed) {
        mavlinkVehicleBaseMode |= MAV_MODE_FLAG_SAFETY_ARMED;
        mavlinkVehicleSystemStatus = MAV_STATE_ACTIVE;
    } else {
        mavlinkVehicleBaseMode &= static_cast<uint8_t>(~MAV_MODE_FLAG_SAFETY_ARMED);
        mavlinkVehicleSystemStatus = MAV_STATE_ACTIVE;
    }
    if (mavlinkDataMutex != nullptr) {
        xSemaphoreGive(mavlinkDataMutex);
    }
}

void UpdateVehicleMode(uint8_t baseMode, uint32_t customMode) {
    if (mavlinkDataMutex != nullptr) {
        xSemaphoreTake(mavlinkDataMutex, portMAX_DELAY);
    }
    mavlinkVehicleBaseMode = static_cast<uint8_t>(
        baseMode | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
    );
    mavlinkVehicleCustomMode = customMode;
    if (mavlinkVehicleArmed) {
        mavlinkVehicleBaseMode |= MAV_MODE_FLAG_SAFETY_ARMED;
    } else {
        mavlinkVehicleBaseMode &= static_cast<uint8_t>(~MAV_MODE_FLAG_SAFETY_ARMED);
    }
    if (mavlinkDataMutex != nullptr) {
        xSemaphoreGive(mavlinkDataMutex);
    }
}
}  // namespace

void ProcessQgcHandshakePacket(const MavlinkRxPacket_t &pkt) {
    switch (pkt.msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cmd = {};
            mavlink_msg_command_long_decode(&pkt.msg, &cmd);

            if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                const bool shouldArm = cmd.param1 > 0.5f;
                SetArmedState(shouldArm);
                SendCommandAck(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED, pkt.msg.sysid, pkt.msg.compid);
                Serial.println(shouldArm ? "[MAVLINK][QGC] ARM accepted" : "[MAVLINK][QGC] DISARM accepted");
                return;
            }

            if (cmd.command == MAV_CMD_DO_SET_MODE) {
                UpdateVehicleMode(static_cast<uint8_t>(cmd.param1), static_cast<uint32_t>(cmd.param2));
                SendCommandAck(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED, pkt.msg.sysid, pkt.msg.compid);
                Serial.println("[MAVLINK][QGC] DO_SET_MODE accepted");
                return;
            }

            if (cmd.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
                SendAutopilotVersion(pkt.msg.sysid, pkt.msg.compid);
                SendCommandAck(
                    MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                    MAV_RESULT_ACCEPTED,
                    pkt.msg.sysid,
                    pkt.msg.compid
                );
                Serial.println("[MAVLINK][QGC] AUTOPILOT_VERSION sent");
                return;
            }

            if (cmd.command == MAV_CMD_REQUEST_MESSAGE &&
                static_cast<uint32_t>(cmd.param1) == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
                SendAutopilotVersion(pkt.msg.sysid, pkt.msg.compid);
                SendCommandAck(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, pkt.msg.sysid, pkt.msg.compid);
                Serial.println("[MAVLINK][QGC] REQUEST_MESSAGE(AUTOPILOT_VERSION) handled");
                return;
            }

            SendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED, pkt.msg.sysid, pkt.msg.compid);
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_INT: {
            mavlink_command_int_t cmd = {};
            mavlink_msg_command_int_decode(&pkt.msg, &cmd);

            if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                const bool shouldArm = cmd.param1 > 0.5f;
                SetArmedState(shouldArm);
                SendCommandAck(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED, pkt.msg.sysid, pkt.msg.compid);
                return;
            }

            SendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED, pkt.msg.sysid, pkt.msg.compid);
            break;
        }

        case MAVLINK_MSG_ID_SET_MODE: {
            mavlink_set_mode_t requestedMode = {};
            mavlink_msg_set_mode_decode(&pkt.msg, &requestedMode);
            UpdateVehicleMode(requestedMode.base_mode, requestedMode.custom_mode);
            Serial.println("[MAVLINK][QGC] SET_MODE received");
            break;
        }

        case MAVLINK_MSG_ID_TIMESYNC: {
            mavlink_timesync_t ts = {};
            mavlink_msg_timesync_decode(&pkt.msg, &ts);

            // Per MAVLink timesync: respond when tc1 == 0 with our local timestamp.
            if (ts.tc1 == 0) {
                mavlink_timesync_t response = {};
                response.tc1 = ts.ts1;
                response.ts1 = static_cast<int64_t>(micros()) * 1000LL;  // ns estimate

                mavlink_message_t msg = {};
                mavlink_msg_timesync_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &response);
                SendMavlinkMessage(msg);
            }
            break;
        }

        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
            mavlink_mission_request_list_t req = {};
            mavlink_msg_mission_request_list_decode(&pkt.msg, &req);
            SendMissionCountZero(pkt.msg.sysid, pkt.msg.compid, req.mission_type);
            Serial.println("[MAVLINK][QGC] MISSION_COUNT=0 sent");
            break;
        }

        case MAVLINK_MSG_ID_MISSION_COUNT: {
            mavlink_mission_count_t req = {};
            mavlink_msg_mission_count_decode(&pkt.msg, &req);
            const uint8_t result = (req.count == 0U) ? MAV_MISSION_ACCEPTED : MAV_MISSION_UNSUPPORTED;
            SendMissionAck(pkt.msg.sysid, pkt.msg.compid, result, req.mission_type);
            break;
        }

        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
            mavlink_mission_clear_all_t req = {};
            mavlink_msg_mission_clear_all_decode(&pkt.msg, &req);
            SendMissionAck(pkt.msg.sysid, pkt.msg.compid, MAV_MISSION_ACCEPTED, req.mission_type);
            break;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            SendParamList();
            Serial.println("[MAVLINK][QGC] PARAM_VALUE list sent");
            break;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            mavlink_param_request_read_t req = {};
            mavlink_msg_param_request_read_decode(&pkt.msg, &req);

            if (req.param_index >= 0 &&
                req.param_index < kQgcParamCount) {
                SendSingleParam(gQgcParamTable[req.param_index], req.param_index);
                break;
            }

            req.param_id[15] = '\0';
            QgcParamDef *def = FindParamByName(req.param_id);
            if (def != nullptr) {
                for (int16_t idx = 0; idx < kQgcParamCount; ++idx) {
                    if (def == &gQgcParamTable[idx]) {
                        SendSingleParam(*def, idx);
                        break;
                    }
                }
            }
            break;
        }

        case MAVLINK_MSG_ID_PARAM_SET: {
            mavlink_param_set_t req = {};
            mavlink_msg_param_set_decode(&pkt.msg, &req);
            req.param_id[15] = '\0';
            QgcParamDef *def = FindParamByName(req.param_id);
            if (def != nullptr) {
                def->value = req.param_value;
                def->type = req.param_type;
                for (int16_t idx = 0; idx < kQgcParamCount; ++idx) {
                    if (def == &gQgcParamTable[idx]) {
                        SendSingleParam(*def, idx);
                        break;
                    }
                }
            }
            break;
        }

        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t gcsHeartbeat = {};
            mavlink_msg_heartbeat_decode(&pkt.msg, &gcsHeartbeat);
            if (gcsHeartbeat.type == MAV_TYPE_GCS) {
                if (mavlinkDataMutex != nullptr) {
                    xSemaphoreTake(mavlinkDataMutex, portMAX_DELAY);
                }
                mavlinkGcsPresent = true;
                if (mavlinkDataMutex != nullptr) {
                    xSemaphoreGive(mavlinkDataMutex);
                }
            }
            break;
        }

        case MAVLINK_MSG_ID_PING: {
            // Delegate PING handling so RTT bookkeeping stays in one place.
            (void)MavlinkHandlePingForLatency(pkt);
            break;
        }

        default:
            break;
    }
}

void MavlinkQgcHandshakeTask(void *pvParameters) {
    (void)pvParameters;

    MavlinkRxPacket_t pkt = {};
    for (;;) {
        if (mavlinkQgcHandshakeQueue == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(RX_SLOW_MS_PER_TICK));
            continue;
        }

        if (xQueueReceive(mavlinkQgcHandshakeQueue, &pkt, pdMS_TO_TICKS(RX_SLOW_MS_PER_TICK)) == pdTRUE) {
            ProcessQgcHandshakePacket(pkt);
        }
    }
}
