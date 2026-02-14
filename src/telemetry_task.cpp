/*
Authors:
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"
#include "hardware.h"

#include <arduino_freertos.h>
#include <queue.h>

QueueHandle_t global_logging_queue = nullptr;

void GlobalLoggingTask(void *pvParameters){

    for (;;)
    {
        
    }
}