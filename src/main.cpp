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
static constexpr bool kEnableSimulatedLocationSensorTask = false;
static constexpr bool kEnableSerial3LoopbackSelfTestTask = false;

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
volatile uint32_t mavlinkLastManualInputUs = 0U;
volatile uint32_t mavlinkManualInputFrameCount = 0U;
volatile uint8_t mavlinkControlPrintMode = 1U; // 1=armed-only, 2=always
volatile uint8_t mavlinkVehicleBaseMode =
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
volatile uint32_t mavlinkVehicleCustomMode = 0U;
volatile uint8_t mavlinkVehicleSystemStatus = MAV_STATE_ACTIVE;
volatile bool mavlinkVehicleArmed = false;
volatile bool mavlinkGcsPresent = false;
volatile uint32_t mavlinkLatencyRttLastUs = 0U;
volatile uint32_t mavlinkLatencyRttAvgUs = 0U;
volatile uint32_t mavlinkLatencyOneWayAvgUs = 0U;
volatile uint32_t mavlinkPilotLatencyEstimateUs = 0U;
QueueHandle_t mavlinkRxQueue900 = nullptr;
QueueHandle_t mavlinkRxQueue24 = nullptr;
QueueHandle_t mavlinkQgcHandshakeQueue = nullptr;

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

namespace {
void FailStartup(const char *reason) {
    Serial.print("[BOOT][FAIL] ");
    Serial.println(reason);
    for (;;) {
        delay(1000);
    }
}

bool CreateTaskChecked(TaskFunction_t fn, const char *name, uint16_t stackDepth, UBaseType_t priorityValue) {
    if (xTaskCreate(fn, name, stackDepth, NULL, priorityValue, NULL) != pdPASS) {
        Serial.print("[BOOT][FAIL] xTaskCreate failed: ");
        Serial.println(name);
        return false;
    }
    return true;
}
}  // namespace

static bool InitMavlinkRx()
{
    // MAVLink RX queues (ensure Serial2/Serial3 are initialized in HardwareInit).
    mavlinkRxQueue900 = xQueueCreate(QUEUE_SIZE, sizeof(MavlinkRxPacket_t));
    mavlinkRxQueue24 = xQueueCreate(QUEUE_SIZE, sizeof(MavlinkRxPacket_t));
    mavlinkQgcHandshakeQueue = xQueueCreate(QUEUE_SIZE, sizeof(MavlinkRxPacket_t));
    if (mavlinkRxQueue900 == nullptr || mavlinkRxQueue24 == nullptr || mavlinkQgcHandshakeQueue == nullptr) {
        Serial.println("[BOOT][FAIL] MAVLink queue creation failed");
        return false;
    }

    if (!CreateTaskChecked(MavlinkRx900Task, "Rx900", STACK_DEPTH, *priority + 3)) {
        return false;
    }
    if (!CreateTaskChecked(MavlinkRx24Task, "Rx24", STACK_DEPTH, *priority + 2)) {
        return false;
    }
    if (!CreateTaskChecked(RxMavlinkProcess900PacketTask, "900MhzProces", RX_PROCESS_STACK_DEPTH, *priority + 2)) {
        return false;
    }
    if (!CreateTaskChecked(MavlinkQgcHandshakeTask, "QgcHandshake", RX_PROCESS_STACK_DEPTH, *priority + 2)) {
        return false;
    }
    if (kEnableSerial3LoopbackSelfTestTask) {
        if (!CreateTaskChecked(Serial3LoopbackSelfTestTask, "Ser3Loop", STACK_DEPTH, *priority + 1)) {
            return false;
        }
    }
    return true;
}

static bool InitLogging()
{
    sensorData_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<SensorData_t>));
    controlOutput_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<ControlOutput_t>));
    stateVector_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<StateVector_t>));
    manualControl_t_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<mavlink_manual_control_t>));
    sensorData_latest_queue = xQueueCreate(1, sizeof(Log<SensorData_t>));
    stateVector_latest_queue = xQueueCreate(1, sizeof(Log<StateVector_t>));
    if (sensorData_logging_queue == nullptr ||
        controlOutput_logging_queue == nullptr ||
        stateVector_logging_queue == nullptr ||
        manualControl_t_logging_queue == nullptr ||
        sensorData_latest_queue == nullptr ||
        stateVector_latest_queue == nullptr) {
        Serial.println("[BOOT][FAIL] logging queue creation failed");
        return false;
    }
    return CreateTaskChecked(SDCardTask, "Logger", STACK_DEPTH, *priority);
}

static bool InitTx()
{
    // Keep Serial2 MAVLink-only for QGC. GSATxTask writes a raw custom packet and
    // can corrupt MAVLink framing on the same UART.
    // xTaskCreate(GSATxTask, "GSATx", STACK_DEPTH, NULL, *priority + 2, NULL);
    if (!CreateTaskChecked(MavlinkHeartbeatTask, "MavHb", STACK_DEPTH, *priority + 2)) {
        return false;
    }
    if (!CreateTaskChecked(MavlinkLatencyProbeTask, "MavLat", STACK_DEPTH, *priority + 2)) {
        return false;
    }
    if (kEnableSimulatedLocationSensorTask) {
        if (!CreateTaskChecked(MavlinkSimulatedTelemetryTask, "MavSim", RX_PROCESS_STACK_DEPTH, *priority + 2)) {
            return false;
        }
    }
    return true;
}

// Program Entry Point

void setup()
{
    Serial.begin(115200);
    MAVLINK_SERIAL_900.begin(MAVLINK_BAUD_900);
    MAVLINK_SERIAL_24.begin(MAVLINK_BAUD_24);

    // Give PlatformIO monitor time to attach to USB CDC before tasks start logging.
    const uint32_t serialWaitStartMs = millis();
    while (!Serial && (millis() - serialWaitStartMs) < 5000U)
    {
        delay(10);
    }
    Serial.println("[BOOT] setup start");
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    // digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);

    if (!HardwareInit()) {
        FailStartup("HardwareInit failed");
    }
    if (!initialize_manual_control()) {
        FailStartup("initialize_manual_control failed");
    }

    controllerMutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
    mavlinkDataMutex = xSemaphoreCreateMutex();
    if (controllerMutex == nullptr || stateMutex == nullptr || dataMutex == nullptr || mavlinkDataMutex == nullptr) {
        FailStartup("mutex creation failed");
    }

    // xTaskCreate Paramenters:
    // pvTaskCode - Pointer to task
    // pcName - Debugging Label
    // usStackDepth - Stack size in words
    // pvParameters - Parameters passed into task
    // uxPriority - Priority level (lower is more priority)
    // pxCreatedTask - Pointer to task handle
    if (!CreateTaskChecked(BlinkTask, "Blink", STACK_DEPTH, *priority)) {
        FailStartup("Blink task creation failed");
    }
    // xTaskCreate(ImuBaroTask, "ImuBaro", STACK_DEPTH, NULL, *priority + 1, NULL);
    //  xTaskCreate(GPSTask, "GPS", STACK_DEPTH, NULL, *priority + 2, NULL);
    // xTaskCreate(StateTask, "State", STACK_DEPTH, NULL, *priority, NULL);
    // xTaskCreate(PIDTask, "PID", STACK_DEPTH, NULL, *priority, NULL);
    //  xTaskCreate(GSARxTask, "GSARx", STACK_DEPTH, NULL, *priority+3, NULL);
    //  xTaskCreate(GSATxTask, "GSATx", STACK_DEPTH, NULL, *priority+3, NULL);
    if (!InitLogging()) {
        FailStartup("InitLogging failed");
    }
    if (!CreateTaskChecked(LoggingQueueSmokeTestTask, "LogQSmoke", STACK_DEPTH, *priority)) {
        FailStartup("LogQSmoke task creation failed");
    }
    if (!InitMavlinkRx()) {
        FailStartup("InitMavlinkRx failed");
    }
    if (!InitTx()) {
        FailStartup("InitTx failed");
    }
    Serial.println("[BOOT] starting scheduler");

    // manual
    if (!CreateTaskChecked(writeServoTask, "ServoWrite", 1024, 1)) {
        FailStartup("ServoWrite task creation failed");
    }
    if (!CreateTaskChecked(updateStatesTask, "States", 1024, 2)) {
        FailStartup("States task creation failed");
    }
    //xTaskCreate(radioTask, "ReadRadio(RX)", 1024, NULL, 3, NULL); TODO: REPLACE WITH MAVLink tasks

    vTaskStartScheduler();
}

void loop() {}
