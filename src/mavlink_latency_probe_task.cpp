/*
Authors:
Advik Sharma - github.com/jpyces
*/

#include "mavlink_latency.h"

#include "tasks.h"
#include "hardware.h"

#include <Arduino.h>
#include <stdio.h>

// Design note:
// Keep latency probe state and PING RTT accounting in one module so handshake
// remains focused on capability/mission/parameter negotiation.
namespace {
constexpr uint32_t LATENCY_PING_SEQ_MAGIC = 0xC0DE0000UL;
constexpr bool kEnableLatencySerialLogs = false;

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

void SendMavlinkMessage(const mavlink_message_t &msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = {};
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    MavlinkSerial900Write(buffer, len);
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
}  // namespace

void MavlinkLatencyProbeTask(void *pvParameters) {
    (void)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(1000);  // 1 Hz
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        if (g_probeAwaitingResponse) {
            ++g_rttTimeoutCount;
            if (kEnableLatencySerialLogs) {
                Serial.print("[MAVLINK][LATENCY] timeout packet#");
                Serial.print(g_probePacketCount);
                Serial.print(" seq=");
                Serial.print(g_lastProbeSeq);
                Serial.print(" totalTimeouts=");
                Serial.println(g_rttTimeoutCount);
            }
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
        if (kEnableLatencySerialLogs) {
            Serial.print("[MAVLINK][LATENCY] packet#");
            Serial.print(g_probePacketCount);
            Serial.print(" sent to ");
            Serial.print(targetSystem);
            Serial.print("/");
            Serial.println(targetComponent);
        }
        vTaskDelayUntil(&lastWake, period);
    }
}

bool MavlinkHandlePingForLatency(const MavlinkRxPacket_t &pkt) {
    // Called by the handshake task for MAVLINK_MSG_ID_PING frames.
    // Returning true means this module fully handled the packet.
    if (pkt.msg.msgid != MAVLINK_MSG_ID_PING) {
        return false;
    }

    mavlink_ping_t ping = {};
    mavlink_msg_ping_decode(&pkt.msg, &ping);

    if (ping.target_system == 0U && ping.target_component == 0U) {
        // Reply to broadcast pings so peers can measure RTT to us.
        SendPing(ping.time_usec, ping.seq, pkt.msg.sysid, pkt.msg.compid);
        return true;
    }

    const bool targetedToUs =
        (ping.target_system == MAVLINK_SYSTEM_ID || ping.target_system == 0U) &&
        (ping.target_component == MAVLINK_COMPONENT_ID || ping.target_component == 0U);
    const bool probeMatch =
        ((ping.seq & 0xFFFF0000UL) == LATENCY_PING_SEQ_MAGIC) &&
        (g_probeAwaitingResponse && ping.seq == g_lastProbeSeq);

    if (!(targetedToUs && probeMatch)) {
        return false;
    }

    if (g_latencyResponderSystemId == 0U) {
        g_latencyResponderSystemId = pkt.msg.sysid;
        if (kEnableLatencySerialLogs) {
            Serial.print("[MAVLINK][LATENCY] Lock responder sysid=");
            Serial.println(g_latencyResponderSystemId);
        }
    }
    if (pkt.msg.sysid != g_latencyResponderSystemId) {
        return false;
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

    if (kEnableLatencySerialLogs) {
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
    }

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
    return true;
}
