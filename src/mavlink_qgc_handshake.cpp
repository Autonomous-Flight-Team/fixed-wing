#include "tasks.h"
#include "hardware.h"
#include <Arduino.h>
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
uint32_t g_rttSampleCount = 0U;
uint32_t g_rttSumMs = 0U;
uint32_t g_rttMinMs = 0xFFFFFFFFUL;
uint32_t g_rttMaxMs = 0U;
uint8_t g_latencyResponderSystemId = 0U;

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

    SendCommandAck(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, MAV_RESULT_ACCEPTED, targetSystem, targetComponent);
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

void SendParamListZero() {
    mavlink_message_t msg = {};
    mavlink_param_value_t param = {};
    param.param_value = 0.0f;
    param.param_count = 0U;
    param.param_index = -1;
    param.param_type = MAV_PARAM_TYPE_REAL32;
    param.param_id[0] = '\0';

    mavlink_msg_param_value_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &param);
    SendMavlinkMessage(msg);
}
}  // namespace

void MavlinkLatencyProbeTask(void *pvParameters) {
    (void)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(1000);  // 1 Hz
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        if (g_probeAwaitingResponse) {
            Serial.print("[MAVLINK][LATENCY] No response for seq ");
            Serial.println(g_lastProbeSeq);
        }

        const uint32_t seq = LATENCY_PING_SEQ_MAGIC | static_cast<uint32_t>(g_probeCounter++);
        const uint32_t nowUs = micros();
        // QGC is most reliable replying to broadcast pings for RTT testing.
        const uint8_t targetSystem = 0U;
        const uint8_t targetComponent = 0U;

        g_lastProbeSeq = seq;
        g_lastProbeSentUs = nowUs;
        g_probeAwaitingResponse = true;

        SendPing(static_cast<uint64_t>(nowUs), seq, targetSystem, targetComponent);
        Serial.print("[MAVLINK][LATENCY] Probe sent to ");
        Serial.print(targetSystem);
        Serial.print("/");
        Serial.println(targetComponent);
        vTaskDelayUntil(&lastWake, period);
    }
}

void HandleQgcHandshakePacket(const MavlinkRxPacket_t &pkt) {
    UpdatePeerFromPacket(pkt);

    switch (pkt.msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cmd = {};
            mavlink_msg_command_long_decode(&pkt.msg, &cmd);

            if (cmd.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
                SendAutopilotVersion(pkt.msg.sysid, pkt.msg.compid);
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

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            SendParamListZero();
            Serial.println("[MAVLINK][QGC] PARAM_VALUE sent (param_count=0)");
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
                g_rttSumMs += rttMs;
                if (rttMs < g_rttMinMs) {
                    g_rttMinMs = rttMs;
                }
                if (rttMs > g_rttMaxMs) {
                    g_rttMaxMs = rttMs;
                }
                const uint32_t avgMs = (g_rttSampleCount == 0U) ? 0U : (g_rttSumMs / g_rttSampleCount);

                Serial.print("[MAVLINK][LATENCY] RTT ms: ");
                Serial.print(rttMs);
                Serial.print(" avg/min/max: ");
                Serial.print(avgMs);
                Serial.print("/");
                Serial.print(g_rttMinMs);
                Serial.print("/");
                Serial.println(g_rttMaxMs);

                char msg[50] = {};
                snprintf(
                    msg,
                    sizeof(msg),
                    "RTT %lu avg %lu",
                    static_cast<unsigned long>(rttMs),
                    static_cast<unsigned long>(avgMs)
                );
                SendStatusTextInfo(msg);
            }
            break;
        }

        default:
            break;
    }
}
