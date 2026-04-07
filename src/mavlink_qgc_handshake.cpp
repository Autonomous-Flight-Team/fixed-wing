#include "tasks.h"
#include "hardware.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>

namespace {
constexpr uint32_t LATENCY_PING_SEQ_MAGIC = 0xC0DE0000UL;

volatile uint8_t g_peerSystemId = 0U;
volatile uint8_t g_peerComponentId = 0U;
volatile bool g_havePeerTarget = false;
volatile uint32_t g_lastProbeSeq = 0U;
volatile uint32_t g_lastProbeSentUs = 0U;
volatile bool g_probeAwaitingResponse = false;
uint16_t g_probeCounter = 0U;
uint32_t g_probePacketCount = 0U;
uint32_t g_rttSampleCount = 0U;
uint64_t g_rttSumUs = 0U;
uint32_t g_rttMinMs = 0xFFFFFFFFUL;
uint32_t g_rttMaxMs = 0U;
uint32_t g_rttTimeoutCount = 0U;
uint8_t g_latencyResponderSystemId = 0U;

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
    MAVLINK_SERIAL_900.write(buffer, len);
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

void SendStatusTextInfo(const char *text) {
    mavlink_statustext_t status = {};
    status.severity = MAV_SEVERITY_INFO;
    snprintf(reinterpret_cast<char *>(status.text), sizeof(status.text), "%s", text);
    status.id = 0U;
    status.chunk_seq = 0U;

    mavlink_message_t msg = {};
    mavlink_msg_statustext_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &status);
    SendMavlinkMessage(msg);
}

void SendPing(uint64_t timeUsec, uint32_t seq, uint8_t targetSystem, uint8_t targetComponent) {
    mavlink_message_t msg = {};
    mavlink_msg_ping_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        timeUsec,
        seq,
        targetSystem,
        targetComponent
    );
    SendMavlinkMessage(msg);
}

void UpdatePeerFromPacket(const MavlinkRxPacket_t &pkt) {
    if (pkt.msg.sysid == 0U || pkt.msg.sysid == MAVLINK_SYSTEM_ID) {
        return;
    }
    g_peerSystemId = pkt.msg.sysid;
    g_peerComponentId = pkt.msg.compid;
    g_havePeerTarget = true;
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

void MavlinkLatencyProbeTask(void *pvParameters) {
    (void)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(1000);  // 1 Hz
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        if (g_probeAwaitingResponse) {
            ++g_rttTimeoutCount;
            Serial.print("[MAVLINK][LATENCY] timeout packet#");
            Serial.print(g_probePacketCount);
            Serial.print(" seq=");
            Serial.print(g_lastProbeSeq);
            Serial.print(" totalTimeouts=");
            Serial.println(g_rttTimeoutCount);
        }

        const uint32_t seq = LATENCY_PING_SEQ_MAGIC | static_cast<uint32_t>(g_probeCounter++);
        const uint32_t nowUs = micros();
        // QGC is most reliable replying to broadcast pings for RTT testing.
        const uint8_t targetSystem = 0U;
        const uint8_t targetComponent = 0U;

        g_lastProbeSeq = seq;
        g_lastProbeSentUs = nowUs;
        g_probeAwaitingResponse = true;
        ++g_probePacketCount;

        SendPing(static_cast<uint64_t>(nowUs), seq, targetSystem, targetComponent);
        Serial.print("[MAVLINK][LATENCY] packet#");
        Serial.print(g_probePacketCount);
        Serial.print(" sent to ");
        Serial.print(targetSystem);
        Serial.print("/");
        Serial.println(targetComponent);
        vTaskDelayUntil(&lastWake, period);
    }
}

void ProcessQgcHandshakePacket(const MavlinkRxPacket_t &pkt) {
    UpdatePeerFromPacket(pkt);

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
            mavlink_ping_t ping = {};
            mavlink_msg_ping_decode(&pkt.msg, &ping);

            if (ping.target_system == 0U && ping.target_component == 0U) {
                // Reply to broadcast pings so peers can measure RTT to us.
                SendPing(ping.time_usec, ping.seq, pkt.msg.sysid, pkt.msg.compid);
                break;
            }

            const bool targetedToUs =
                (ping.target_system == MAVLINK_SYSTEM_ID || ping.target_system == 0U) &&
                (ping.target_component == MAVLINK_COMPONENT_ID || ping.target_component == 0U);
            const bool probeMatch =
                ((ping.seq & 0xFFFF0000UL) == LATENCY_PING_SEQ_MAGIC) &&
                (g_probeAwaitingResponse && ping.seq == g_lastProbeSeq);

            if (targetedToUs && probeMatch) {
                if (g_latencyResponderSystemId == 0U) {
                    g_latencyResponderSystemId = pkt.msg.sysid;
                    Serial.print("[MAVLINK][LATENCY] Lock responder sysid=");
                    Serial.println(g_latencyResponderSystemId);
                }
                if (pkt.msg.sysid != g_latencyResponderSystemId) {
                    break;
                }

                const uint32_t rttUs = micros() - g_lastProbeSentUs;
                const uint32_t rttMs = rttUs / 1000U;
                g_probeAwaitingResponse = false;
                ++g_rttSampleCount;
                g_rttSumUs += rttUs;
                mavlinkLatencyRttLastUs = rttUs;
                if (mavlinkLatencyRttAvgUs == 0U) {
                    mavlinkLatencyRttAvgUs = rttUs;
                } else {
                    // EWMA smoothing (80% previous, 20% new) for pilot-facing stability.
                    mavlinkLatencyRttAvgUs = (mavlinkLatencyRttAvgUs * 4U + rttUs) / 5U;
                }
                mavlinkLatencyOneWayAvgUs = mavlinkLatencyRttAvgUs / 2U;
                if (rttMs < g_rttMinMs) {
                    g_rttMinMs = rttMs;
                }
                if (rttMs > g_rttMaxMs) {
                    g_rttMaxMs = rttMs;
                }
                const uint32_t avgUs = (g_rttSampleCount == 0U)
                    ? 0U
                    : static_cast<uint32_t>(g_rttSumUs / g_rttSampleCount);
                const uint32_t avgMsWhole = avgUs / 1000U;
                const uint32_t avgMsFrac = avgUs % 1000U;

                Serial.print("[MAVLINK][LATENCY] packet#");
                Serial.print(g_probePacketCount);
                Serial.print(" seq=");
                Serial.print(g_lastProbeSeq);
                Serial.print(" RTT ms: ");
                Serial.print(rttMs);
                Serial.print(" avg/min/max: ");
                Serial.print(avgMsWhole);
                Serial.print(".");
                if (avgMsFrac < 100U) {
                    Serial.print("0");
                }
                if (avgMsFrac < 10U) {
                    Serial.print("0");
                }
                Serial.print(avgMsFrac);
                Serial.print("/");
                Serial.print(g_rttMinMs);
                Serial.print("/");
                Serial.println(g_rttMaxMs);

                char msg[80] = {};
                snprintf(
                    msg,
                    sizeof(msg),
                    "pkt%lu RTT %lu avg %lu.%03lu",
                    static_cast<unsigned long>(g_probePacketCount),
                    static_cast<unsigned long>(rttMs),
                    static_cast<unsigned long>(avgMsWhole),
                    static_cast<unsigned long>(avgMsFrac)
                );
                SendStatusTextInfo(msg);
            }
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
