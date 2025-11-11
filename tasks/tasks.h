#ifndef TASKS_H
#define TASKS_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


extern SensorData_t sensorData;
extern ControlOutput_t controlOutput;
extern SemaphoreHandle_t dataMutex;


// Task Declarations
void SensorTask(void *pvParameters);
void StateTask(void *pvParameters);
void PIDTask(void *pvParameters);
void DSHOTTask(void *pvParameters);
void GSATxTask(void *pvParameters);
void GSARxTask(void *pvParameters);

#endif