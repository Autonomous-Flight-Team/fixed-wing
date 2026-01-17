#ifndef TASKS_H
#define TASKS_H




#include "structs.h"
// #include "libraries/freertos-teensy/src/arduino_freertos.h"
// #include "libraries/freertos-teensy/src/task.h"
// #include "libraries/freertos-teensy/src/semphr.h"
#include <arduino_freertos.h>
#include <semphr.h>
#include <task.h>

extern SensorData_t sensorData;
extern ControlOutput_t controlOutput;
extern StateVector_t stateVector;
extern SemaphoreHandle_t dataMutex;
extern BlinkState_t blinkState;


// Task Declarations
void ImuBaroTask(void *pvParameters);
void GPSTask(void *pvParameters);
void StateTask(void *pvParameters);
void PIDTask(void *pvParameters);
void GSATxTask(void *pvParameters);
void GSARxTask(void *pvParameters);
void BlinkTask(void *pvParameters);

#endif