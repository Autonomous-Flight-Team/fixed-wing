

// Authors
// - Colin Faletto github.com/faletto



#include "tasks.h"
#include "hardware.h"
#include "types.h"
#include <arduino_freertos.h>
#include <semphr.h>
#include <task.h>

SemaphoreHandle_t dataMutex;


int STACK_DEPTH = 512;
int priority[] = {1,2,3,4};


SensorData_t sensorData = {0};
ControlOutput_t controlOutput = {0};
StateVector_t stateVector = {0};
BlinkState_t blinkState = {false};
mavlink_set_position_target_global_int_t set_global_position = {};
mavlink_manual_control_t manual_control_data = {};
mavlink_command_long_t specific_cmds = {};
mavlink_set_mode_t mode = {};


DroneMode droneMode = MANUAL;

// Program Entry Point

void setup() {
    Serial.begin(115200);
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    // digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);

    HardwareInit();
    dataMutex = xSemaphoreCreateMutex();

    // MAVLink RX queues (ensure Serial2/Serial3 are initialized in HardwareInit).
    mavlinkRxQueue900 = xQueueCreate(16, sizeof(MavlinkRxPacket_t));
    mavlinkRxQueue24  = xQueueCreate(16, sizeof(MavlinkRxPacket_t));
    

    // xTaskCreate Paramenters:
    // pvTaskCode - Pointer to task
    // pcName - Debugging Label
    // usStackDepth - Stack size in words
    // pvParameters - Parameters passed into task
    // uxPriority - Priority level (lower is more priority)
    // pxCreatedTask - Pointer to task handle
    xTaskCreate(BlinkTask, "Blink", STACK_DEPTH, NULL, *priority, NULL);
    // xTaskCreate(ImuBaroTask, "ImuBaro", STACK_DEPTH, NULL, *priority + 1, NULL);
    // xTaskCreate(GPSTask, "GPS", STACK_DEPTH, NULL, *priority + 2, NULL);
    // xTaskCreate(StateTask, "State", STACK_DEPTH, NULL, *priority, NULL);
    // xTaskCreate(PIDTask, "PID", STACK_DEPTH, NULL, *priority, NULL);
    // xTaskCreate(GSARxTask, "GSARx", STACK_DEPTH, NULL, *priority+3, NULL);
    // xTaskCreate(GSATxTask, "GSATx", STACK_DEPTH, NULL, *priority+3, NULL);
    xTaskCreate(MavlinkRx900Task, "Rx900", STACK_DEPTH, NULL, *priority + 3, NULL);
    xTaskCreate(MavlinkRx24Task, "Rx24", STACK_DEPTH, NULL, *priority + 2, NULL);
    xTaskCreate(MavlinkRx24Task, "Rx24", STACK_DEPTH, NULL, *priority + 2, NULL);
    xTaskCreate(RxMavlinkProcess900PacketTask, "900MhzProces", STACK_DEPTH, NULL, *priority + 2, NULL);
    vTaskStartScheduler();

}

void loop() {}
