/*
Authors:
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"
#include "hardware.h"

void MavlinkSerial900Write(const uint8_t *data, uint16_t len) {
    if (data == nullptr || len == 0U) {
        return;
    }

    if (mavlinkTxMutex != nullptr) {
        if (xSemaphoreTake(mavlinkTxMutex, portMAX_DELAY) == pdTRUE) {
            MAVLINK_SERIAL_900.write(data, len);
            xSemaphoreGive(mavlinkTxMutex);
            return;
        }
    }

    // Fallback path used before mutex init or if lock acquisition fails.
    MAVLINK_SERIAL_900.write(data, len);
}
