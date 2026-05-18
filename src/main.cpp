// Authors
// - Colin Faletto github.com/faletto

#include "tasks.h"
#include "hardware.h"
#include "types.h"
#include "map_pins.h"
#include <arduino_freertos.h>
#include <semphr.h>
#include <task.h>

constexpr int QUEUE_SIZE = 100;

// Priority Variables
constexpr int kPriorityOne = 1;  // Constant storing L1 Priority Indicator
constexpr int kPriorityTwo = 2;  // Constant storing L2 Priority Indicator
constexpr int kPriorityThree = 3;  // Constant storing L3 Priority Indicator
constexpr int kPriorityFour = 4;  // Constant storing L4 Priority Indicator
constexpr int kPriorityFive = 5;   // Constant storing L5 Priority Indicator
constexpr int kPrioritySix = 6;   // Constant storing L6 Priority Indicator
constexpr int kPrioritySeven = 7; // Constant storing L7 Priority Indicator
constexpr int kPriorityEight = 8;  // Constant storing L8 Priority Indicator

SemaphoreHandle_t dataMutex, controllerMutex, stateMutex, mavlinkDataMutex, mavlinkTxMutex;
// mavlinkTxMutex protects the actual UART line to prevent multiple tasks from writing at the same time
//      and forces single frame writes at a time

int STACK_DEPTH = 512;
int RX_PROCESS_STACK_DEPTH = 1536;
//int priority[] = {1, 2, 3, 4};
static constexpr bool kEnableSimulatedLocationSensorTask = true;
static constexpr bool kEnableSerial3LoopbackSelfTestTask = false;

SensorData_t sensorData = {0};
IMUData_t imuData = {0};
BaroData_t baroData = {0};
ControlOutput_t controlOutput = {0};
StateVector_t stateVector = {0};
BlinkState_t blinkState = {false};
FlapsPosition mavlinkFlapsPosition = FLAPS_DOWN;

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
QueueHandle_t mavlinkRx24_QuadcopterOrigin_ForwardQueue = nullptr;

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
QueueHandle_t controlOutput_logging_queue = nullptr;
QueueHandle_t stateVector_logging_queue = nullptr;
QueueHandle_t manualControl_t_logging_queue = nullptr;

// Sensor Logging Queues
QueueHandle_t imu_logging_queue;
QueueHandle_t barometer_logging_queue;
QueueHandle_t gps_logging_queue;
QueueHandle_t pitotTube_logging_queue;

// Queue Drop Trackers
volatile uint32_t controlOutput_logging_drop_count = 0;
volatile uint32_t stateVector_logging_drop_count = 0;
volatile uint32_t manualControl_logging_drop_count = 0;
volatile uint32_t imu_logging_drop_count = 0;
volatile uint32_t barometer_logging_drop_count = 0;
volatile uint32_t gps_logging_drop_count = 0;
volatile uint32_t pitotTube_logging_drop_count = 0;

DroneMode droneMode = MANUAL; // Perhaps use mavlinks version, or update custom mavlink cmd

namespace
{
    void FailStartup(const char *reason)
    {
        Serial.print("[BOOT][FAIL] ");
        Serial.println(reason);
        for (;;)
        {
            delay(1000);
        }
    }

    bool CreateTaskChecked(TaskFunction_t fn, const char *name, uint16_t stackDepth, UBaseType_t priorityValue)
    {
        if (xTaskCreate(fn, name, stackDepth, NULL, priorityValue, NULL) != pdPASS)
        {
            Serial.print("[BOOT][FAIL] xTaskCreate failed: ");
            Serial.println(name);
            return false;
        }
        return true;
    }
} // namespace

static bool InitMavlinkRx()
{
    // MAVLink RX queues (ensure Serial2/Serial3 are initialized in HardwareInit).
    mavlinkRxQueue900 = xQueueCreate(QUEUE_SIZE, sizeof(MavlinkRxPacket_t));
    mavlinkRxQueue24 = xQueueCreate(QUEUE_SIZE, sizeof(MavlinkRxPacket_t));
    mavlinkQgcHandshakeQueue = xQueueCreate(QUEUE_SIZE, sizeof(MavlinkRxPacket_t));
    mavlinkRx24_QuadcopterOrigin_ForwardQueue = xQueueCreate(QUEUE_SIZE, sizeof(MavlinkRxPacket_t));

    if (mavlinkRxQueue900 == nullptr ||
        mavlinkRxQueue24 == nullptr ||
        mavlinkQgcHandshakeQueue == nullptr ||
        mavlinkRx24_QuadcopterOrigin_ForwardQueue == nullptr)
    {
        Serial.println("[BOOT][FAIL] MAVLink queue creation failed");
        return false;
    }

    if (!CreateTaskChecked(MavlinkRx900Task, "Rx900", STACK_DEPTH, kPriorityFour))
    {
        return false;
    }
    if (!CreateTaskChecked(MavlinkRx24Task, "Rx24", STACK_DEPTH, kPriorityFour))
    {
        return false;
    }
    if (!CreateTaskChecked(RxMavlinkProcess900PacketTask, "900MhzProces", RX_PROCESS_STACK_DEPTH, kPriorityThree))
    {
        return false;
    }
    if (!CreateTaskChecked(RxMavlinkProcess24PacketTask, "24GhzProces", RX_PROCESS_STACK_DEPTH, kPriorityThree))
    {
        return false;
    }
    if (!CreateTaskChecked(MavlinkQgcHandshakeTask, "QgcHandshake", RX_PROCESS_STACK_DEPTH, kPriorityThree))
    {
        return false;
    }
    if (kEnableSerial3LoopbackSelfTestTask)
    {
        if (!CreateTaskChecked(Serial3LoopbackSelfTestTask, "Ser3Loop", STACK_DEPTH, kPriorityTwo))
        {
            return false;
        }
    }
    return true;
}

static bool InitLogging()
{
    controlOutput_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<ControlOutput_t>));
    stateVector_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<StateVector_t>));
    manualControl_t_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<mavlink_manual_control_t>));

    // Sensor logging queues
    imu_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<IMUData_t>));
    barometer_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<BaroData_t>));
    gps_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<GPSData_t>));
    pitotTube_logging_queue = xQueueCreate(QUEUE_SIZE, sizeof(Log<PitotData_t>));

    if (controlOutput_logging_queue == nullptr ||
        stateVector_logging_queue == nullptr ||
        manualControl_t_logging_queue == nullptr ||
        imu_logging_queue == nullptr || 
        barometer_logging_queue == nullptr || 
        gps_logging_queue == nullptr || 
        pitotTube_logging_queue == nullptr)
    {
        Serial.println("[BOOT][FAIL] logging queue creation failed");
        return false;
    }

    return CreateTaskChecked(SDCardTask, "Logger", 2048, kPriorityOne);
}

static bool InitTx()
{
    // Keep Serial2 MAVLink-only for QGC. GSATxTask writes a raw custom packet and
    // can corrupt MAVLink framing on the same UART.
    if (!CreateTaskChecked(MavlinkHeartbeatTask, "MavHb", STACK_DEPTH, kPriorityThree))
    {
        return false;
    }
    if (!CreateTaskChecked(QuadcopterOriginPacketForwardTask, "QuadOriginPacketForward", STACK_DEPTH, kPriorityTwo))
    {
        return false;
    }
    if (!CreateTaskChecked(MavlinkLatencyProbeTask, "MavLat", STACK_DEPTH, kPriorityTwo))
    {
        return false;
    }
    if (!CreateTaskChecked(MavlinkAltitudeTask, "MavAlt", STACK_DEPTH, kPriorityTwo))
    {
        return false;
    }
    if (kEnableSimulatedLocationSensorTask)
    {
        if (!CreateTaskChecked(MavlinkSimulatedTelemetryTask, "MavSim", RX_PROCESS_STACK_DEPTH, kPriorityOne))
        {
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

    // if (!HardwareInit()) {
    //     FailStartup("HardwareInit failed");
    // }
    HardwareInit();
    if (!initialize_manual_control()) {
        FailStartup("initialize_manual_control failed");
    }

    controllerMutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
    mavlinkDataMutex = xSemaphoreCreateMutex();
    mavlinkTxMutex = xSemaphoreCreateMutex();

    if (controllerMutex == nullptr ||
        stateMutex == nullptr ||
        dataMutex == nullptr ||
        mavlinkDataMutex == nullptr ||
        mavlinkTxMutex == nullptr)
    {
        FailStartup("mutex creation failed");
    }

    // xTaskCreate Paramenters:
    // pvTaskCode - Pointer to task
    // pcName - Debugging Label
    // usStackDepth - Stack size in words
    // pvParameters - Parameters passed into task
    // uxPriority - Priority level (lower is more priority)
    // pxCreatedTask - Pointer to task handle
    if (!CreateTaskChecked(BlinkTask, "Blink", STACK_DEPTH, kPriorityOne))
    {
        FailStartup("Blink task creation failed");
    }
    if (!InitLogging())
    {
        FailStartup("InitLogging failed");
    }
    xTaskCreate(ImuBaroTask, "ImuBaro", STACK_DEPTH, NULL, kPriorityTwo, NULL);

    //  xTaskCreate(GPSTask, "GPS", STACK_DEPTH, NULL, kPriorityThree, NULL);
    // xTaskCreate(StateTask, "State", STACK_DEPTH, NULL, kPriorityOne, NULL);
    // xTaskCreate(PIDTask, "PID", STACK_DEPTH, NULL, kPriorityOne, NULL);
    // if (!CreateTaskChecked(LoggingQueueSmokeTestTask, "LogQSmoke", STACK_DEPTH, kPriorityOne))
    // {
    //     FailStartup("LogQSmoke task creation failed");
    // }
    if (!InitMavlinkRx())
    {
        FailStartup("InitMavlinkRx failed");
    }
    if (!InitTx())
    {
        FailStartup("InitTx failed");
    }
    Serial.println("[BOOT] starting scheduler");

    // manual
    if (!CreateTaskChecked(writeServoTask, "ServoWrite", 1024, kPriorityOne))
    {
        FailStartup("ServoWrite task creation failed");
    }
    if (!CreateTaskChecked(updateStatesTask, "States", 1024, kPriorityThree))
    {
        FailStartup("States task creation failed");
    }
    // xTaskCreate(radioTask, "ReadRadio(RX)", 1024, NULL, 3, NULL); TODO: REPLACE WITH MAVLink tasks
    //xTaskCreate(printControllerTask, "PrintController", 2048, NULL, 1, NULL);
    vTaskStartScheduler();
}

void loop() {}
