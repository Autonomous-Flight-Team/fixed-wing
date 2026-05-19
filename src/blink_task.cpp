#include "tasks.h"

int MS_PER_TICK = 1000;

void BlinkTask(void *pvParameters) {
  
  for (;;) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);
    digitalWrite(arduino::LED_BUILTIN, (blinkState.on) ? arduino::LOW : arduino::HIGH);
    blinkState.on = !blinkState.on;
    vTaskDelayUntil(&lastWake, freq);
  }
}