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

extern SensorData_t sensorData;
extern ControlOutput_t controlOutput;
extern StateVector_t stateVector;
extern SemaphoreHandle_t dataMutex;
extern BlinkState_t blinkState;

extern QueueHandle_t mavlinkRxQueue900;
extern QueueHandle_t mavlinkRxQueue24;
extern volatile uint32_t mavlinkRxDrop900;
extern volatile uint32_t mavlinkRxDrop24;
extern volatile mavlink_message_t mavlinkLastTelemetry;
extern volatile uint32_t mavlinkTelemetryCount;


// Task Declarations
void ImuBaroTask(void *pvParameters);
void GPSTask(void *pvParameters);
void StateTask(void *pvParameters);
void PIDTask(void *pvParameters);
void GSATxTask(void *pvParameters);
void GSARxTask(void *pvParameters);
void BlinkTask(void *pvParameters);
void MavlinkRx900Task(void *pvParameters);
void MavlinkRx24Task(void *pvParameters);
void MavlinkControlDispatchTask(void *pvParameters);
void MavlinkTelemetryDispatchTask(void *pvParameters);


#endif
