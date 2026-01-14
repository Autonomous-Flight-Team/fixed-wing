

// Authors
// - Colin Faletto github.com/faletto



#include "tasks.h"
#include "hardware.h"
#include "structs.h"
#include <arduino_freertos.h>
#include <semphr.h>
#include <task.h>

SemaphoreHandle_t dataMutex;


int STACK_DEPTH = 512;
int priority[] = {1,2,3,4};


SensorData_t sensorData = {0};
ControlOutput_t controlOutput = {0};
StateVector_t stateVector = {0};
DroneMode mode = MANUAL;

// Program Entry Point
int main(void) {
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);

    HardwareInit();
    dataMutex = xSemaphoreCreateMutex();
    

    // xTaskCreate Paramenters:
    // pvTaskCode - Pointer to task
    // pcName - Debugging Label
    // usStackDepth - Stack size in words
    // pvParameters - Parameters passed into task
    // uxPriority - Priority level (lower is more priority)
    // pxCreatedTask - Pointer to task handle
    xTaskCreate(ImuBaroTask, "ImuBaro", STACK_DEPTH, NULL, *priority + 1, NULL);
    xTaskCreate(GPSTask, "GPS", STACK_DEPTH, NULL, *priority + 2, NULL);
    xTaskCreate(StateTask, "State", STACK_DEPTH, NULL, *priority, NULL);
    xTaskCreate(PIDTask, "PID", STACK_DEPTH, NULL, *priority, NULL);
    xTaskCreate(GSARxTask, "GSARx", STACK_DEPTH, NULL, *priority+3, NULL);
    xTaskCreate(GSATxTask, "GSATx", STACK_DEPTH, NULL, *priority+3, NULL);

    vTaskStartScheduler();

    return 0;

}


