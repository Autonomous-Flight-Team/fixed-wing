// Authors
// - Colin Faletto github.com/faletto

#include "tasks.h"
#include "hardware.h"
#include "types.h"
#include "map_pins.h"
#include <arduino_freertos.h>
#include <semphr.h>
#include <task.h>

#define QUEUE_SIZE 100
SemaphoreHandle_t dataMutex, controllerMutex, stateMutex, mavlinkDataMutex;

int STACK_DEPTH = 512;
int RX_PROCESS_STACK_DEPTH = 1536;
int priority[] = {1, 2, 3, 4};

SensorData_t sensorData = {0};
ControlOutput_t controlOutput = {0};
StateVector_t stateVector = {0};
BlinkState_t blinkState = {false};

// Mavlink Manual Control - 900 Rx variables
mavlink_set_position_target_global_int_t set_global_position = {};
mavlink_manual_control_t manual_control_data = {};
mavlink_command_long_t specific_cmds = {};
mavlink_set_mode_t mode = {};
volatile uint32_t mavlinkLastManualInputMs = 0U;
volatile uint32_t mavlinkManualInputFrameCount = 0U;
volatile uint8_t mavlinkControlPrintMode = 1U; // 1=armed-only, 2=always
volatile uint8_t mavlinkVehicleBaseMode =
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
volatile uint32_t mavlinkVehicleCustomMode = 0U;
volatile uint8_t mavlinkVehicleSystemStatus = MAV_STATE_ACTIVE;
volatile bool mavlinkVehicleArmed = false;
volatile bool mavlinkGcsPresent = false;
QueueHandle_t mavlinkRxQueue900 = nullptr;
QueueHandle_t mavlinkRxQueue24 = nullptr;

// manual

Controller_t controllerData = {};
SetServoStates_t servoStateData = {
    aileron_neutral,  // set_aileron  = 16
    elevator_neutral, // set_elevator = 20
    rudder_neutral,   // set_rudder   = 30
    0,                // set_throttle
    false,            // flaps
    false             // release_drone
};

// Logging Queues
QueueHandle_t sensorData_logging_queue = nullptr;
QueueHandle_t controlOutput_logging_queue = nullptr;
QueueHandle_t stateVector_logging_queue = nullptr;
QueueHandle_t manualControl_t_logging_queue = nullptr;
QueueHandle_t sensorData_latest_queue = nullptr;
QueueHandle_t stateVector_latest_queue = nullptr;
volatile uint32_t sensorData_logging_drop_count = 0;
volatile uint32_t controlOutput_logging_drop_count = 0;
volatile uint32_t stateVector_logging_drop_count = 0;
volatile uint32_t manualControl_logging_drop_count = 0;

DroneMode droneMode = MANUAL; // Perhaps use mavlinks version, or update custom mavlink cmd

static void InitMavlinkRx()
{
    // MAVLink RX queues (ensure Serial2/Serial3 are initialized in HardwareInit).
    mavlinkRxQueue900 = xQueueCreate(QUEUE_SIZE, sizeof(MavlinkRxPacket_t));
    mavlinkRxQueue24 = xQueueCreate(QUEUE_SIZE, sizeof(MavlinkRxPacket_t));

    xTaskCreate(MavlinkRx900Task, "Rx900", STACK_DEPTH, NULL, *priority + 3, NULL);
    xTaskCreate(MavlinkRx24Task, "Rx24", STACK_DEPTH, NULL, *priority + 2, NULL);
    // xTaskCreate(MavlinkRx24Task, "Rx24", STACK_DEPTH, NULL, *priority + 2, NULL);
    xTaskCreate(RxMavlinkProcess900PacketTask, "900MhzProces", RX_PROCESS_STACK_DEPTH, NULL, *priority + 2, NULL);
}

static void InitLogging()
{
    sensorData_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<SensorData_t>));
    controlOutput_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<ControlOutput_t>));
    stateVector_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<StateVector_t>));
    manualControl_t_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<mavlink_manual_control_t>));
    sensorData_latest_queue = xQueueCreate(1, sizeof(Log<SensorData_t>));
    stateVector_latest_queue = xQueueCreate(1, sizeof(Log<StateVector_t>));
    xTaskCreate(SDCardTask, "Logger", STACK_DEPTH, NULL, *priority, NULL);
}

static void InitTx()
{
    // Keep Serial2 MAVLink-only for QGC. GSATxTask writes a raw custom packet and
    // can corrupt MAVLink framing on the same UART.
    // xTaskCreate(GSATxTask, "GSATx", STACK_DEPTH, NULL, *priority + 2, NULL);
    xTaskCreate(MavlinkHeartbeatTask, "MavHb", STACK_DEPTH, NULL, *priority + 2, NULL);
    // Latency probe disabled to keep serial output focused on control and heartbeat.
    // xTaskCreate(MavlinkLatencyProbeTask, "MavLat", STACK_DEPTH, NULL, *priority + 2, NULL);
}

// Program Entry Point

void setup()
{
    Serial.begin(115200);
    MAVLINK_SERIAL_900.begin(MAVLINK_BAUD);
    MAVLINK_SERIAL_24.begin(MAVLINK_BAUD);

    // Give PlatformIO monitor time to attach to USB CDC before tasks start logging.
    const uint32_t serialWaitStartMs = millis();
    while (!Serial && (millis() - serialWaitStartMs) < 5000U)
    {
        delay(10);
    }
    Serial.println("[BOOT] setup start");
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    // digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);

    HardwareInit();
    intialize_manual_control();
    controllerMutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();

    // Manual controls initialized
    intialize_manual_control();
    controllerMutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    mavlinkDataMutex = xSemaphoreCreateMutex();
    

    // xTaskCreate Paramenters:
    // pvTaskCode - Pointer to task
    // pcName - Debugging Label
    // usStackDepth - Stack size in words
    // pvParameters - Parameters passed into task
    // uxPriority - Priority level (lower is more priority)
    // pxCreatedTask - Pointer to task handle
    xTaskCreate(BlinkTask, "Blink", STACK_DEPTH, NULL, *priority, NULL);
    // xTaskCreate(ImuBaroTask, "ImuBaro", STACK_DEPTH, NULL, *priority + 1, NULL);
    //  xTaskCreate(GPSTask, "GPS", STACK_DEPTH, NULL, *priority + 2, NULL);
    // xTaskCreate(StateTask, "State", STACK_DEPTH, NULL, *priority, NULL);
    // xTaskCreate(PIDTask, "PID", STACK_DEPTH, NULL, *priority, NULL);
    //  xTaskCreate(GSARxTask, "GSARx", STACK_DEPTH, NULL, *priority+3, NULL);
    //  xTaskCreate(GSATxTask, "GSATx", STACK_DEPTH, NULL, *priority+3, NULL);
    InitLogging();
    xTaskCreate(LoggingQueueSmokeTestTask, "LogQSmoke", STACK_DEPTH, NULL, *priority, NULL);
    InitMavlinkRx();
    InitTx();
    Serial.println("[BOOT] starting scheduler");

    // manual
    xTaskCreate(writeServoTask, "ServoWrite", 1024, NULL, 1, NULL);
    xTaskCreate(updateStatesTask, "States", 1024, NULL, 2, NULL);
    // xTaskCreate(radioTask, "ReadRadio(RX)", 1024, NULL, 3, NULL); TODO: REPLACE WITH MAVLink tasks

    vTaskStartScheduler();
}

void loop() {}
