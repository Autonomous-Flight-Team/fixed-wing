#include "tasks.h"

int MS_PER_TICK = 1000;

void BlinkTask(void *pvParameters) {
  for (;;) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(MS_PER_TICK);
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        digitalWrite(arduino::LED_BUILTIN, (blinkState.on) ? arduino::LOW : arduino::HIGH);
        if (blinkState.on) {
          Serial.println("off");
        } else {
          Serial.println("on");
        }
        blinkState.on = !blinkState.on;
        xSemaphoreGive(dataMutex);
    }
    vTaskDelayUntil(&lastWake, freq);
  }
}