#ifndef TASKS_H
#define TASKS_H

#include "types.h"
// #include "libraries/freertos-teensy/src/arduino_freertos.h"
// #include "libraries/freertos-teensy/src/task.h"
// #include "libraries/freertos-teensy/src/semphr.h"
#include <arduino_freertos.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

// Makes the variables declared in the respective files global
extern SensorData_t sensorData;
extern ControlOutput_t controlOutput;
extern StateVector_t stateVector;
extern SemaphoreHandle_t dataMutex;
extern BlinkState_t blinkState;

// Mavlink shit
extern QueueHandle_t mavlinkRxQueue900;
extern QueueHandle_t mavlinkRxQueue24;
extern volatile uint32_t mavlinkRxDrop900;
extern volatile uint32_t mavlinkRxDrop24;
extern volatile mavlink_message_t mavlinkLastTelemetry;
extern volatile uint32_t mavlinkTelemetryCount;
extern mavlink_set_position_target_global_int_t set_global_position;
extern mavlink_manual_control_t manual_control_data;
extern mavlink_command_long_t specific_cmds;
extern mavlink_set_mode_t mode;

// Logging
extern QueueHandle_t sensorData_logging_queue;
extern QueueHandle_t controlOutput_logging_queue;
extern QueueHandle_t stateVector_logging_queue;

// Task Declarations
// General
void ImuBaroTask(void *pvParameters);
void GPSTask(void *pvParameters);
void StateTask(void *pvParameters);
void PIDTask(void *pvParameters);
void GSATxTask(void *pvParameters);
void GSARxTask(void *pvParameters);
void BlinkTask(void *pvParameters);

// Mavlink Tasks
void MavlinkRx900Task(void *pvParameters);
void MavlinkRx24Task(void *pvParameters);
void MavlinkControlDispatchTask(void *pvParameters);
void MavlinkTelemetryDispatchTask(void *pvParameters);
void RxMavlinkProcess900PacketTask(void *pvParameters);

// Logging task?
void GlobalLoggingTask(void *pvParameters);

#endif
