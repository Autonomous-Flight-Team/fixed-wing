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

    
}