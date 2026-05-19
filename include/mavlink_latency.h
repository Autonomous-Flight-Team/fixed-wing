#ifndef MAVLINK_LATENCY_H
#define MAVLINK_LATENCY_H

#include "types.h"

// Processes incoming PING packets for RTT probing.
// Returns true when the packet was handled by the latency module.
bool MavlinkHandlePingForLatency(const MavlinkRxPacket_t &pkt);

#endif
