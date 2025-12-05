#ifndef TASKS_H
#define TASKS_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


extern SensorData_t sensorData;
extern ControlOutput_t controlOutput;
extern StateVector_t stateVector;
extern SemaphoreHandle_t dataMutex;


// Task Declarations
void ImuBaroTask(void *pvParameters);
void GPSTask(void *pvParameters);
void StateTask(void *pvParameters);
void PIDTask(void *pvParameters);
void GSATxTask(void *pvParameters);
void GSARxTask(void *pvParameters);

#endif