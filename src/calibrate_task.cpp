// Authors
// - Colin Faletto github.com/faletto

#include "tasks.h"
#include "hardware.h"

void CalibrateTask(void *pvParameters) {
  String input = "";

  while (1) {
    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n') {
         
      }
    }

    vTaskDelay(10);
  }
}