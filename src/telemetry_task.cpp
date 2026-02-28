/*
Authors:
Advik Sharma - github.com/jpyces
*/

#include "tasks.h"
#include "hardware.h"
#include "string.h"
#include <arduino_freertos.h>
#include <queue.h>

void FillLoggingQueues(Log<StateVector_t> log)
{
    xQueueSend(stateVector_logging_queue, &log, 0);
}

void FillLoggingQueues(Log<SensorData_t> log)
{
    xQueueSend(sensorData_logging_queue, &log, 0);
}

void FillLoggingQueues(Log<ControlOutput_t> log)
{
    xQueueSend(controlOutput_logging_queue, &log, 0);
}

void FillLoggingQueues(Log<mavlink_manual_control_t> log)
{
    xQueueSend(manualControl_t_logging_queue, &log, 0);
}

void SDCardTask(void *pvParameters){

    for (;;)
    {
        
    }
}
