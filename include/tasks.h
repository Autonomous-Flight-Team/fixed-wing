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
extern SemaphoreHandle_t mavlinkDataMutex;

// Mavlink recieve shit
extern QueueHandle_t mavlinkRxQueue900;
extern QueueHandle_t mavlinkRxQueue24;
extern QueueHandle_t mavlinkQgcHandshakeQueue;
extern volatile uint32_t mavlinkRxDrop900;
extern volatile uint32_t mavlinkRxDrop24;
extern volatile uint32_t mavlinkRxParsed24Count;
extern QueueHandle_t maxlinkTxQueue900;
extern volatile mavlink_message_t mavlinkLastTelemetry;
extern volatile uint32_t mavlinkTelemetryCount;
extern mavlink_set_position_target_global_int_t set_global_position;
extern mavlink_manual_control_t manual_control_data;
extern mavlink_command_long_t specific_cmds;
extern mavlink_set_mode_t mode;
extern volatile uint32_t mavlinkLastManualInputMs;
extern volatile uint32_t mavlinkLastManualInputUs;
extern volatile uint32_t mavlinkManualInputFrameCount;
extern volatile uint8_t mavlinkControlPrintMode;
extern volatile uint8_t mavlinkVehicleBaseMode;
extern volatile uint32_t mavlinkVehicleCustomMode;
extern volatile uint8_t mavlinkVehicleSystemStatus;
extern volatile bool mavlinkVehicleArmed;
extern volatile bool mavlinkGcsPresent;
extern volatile uint32_t mavlinkLatencyRttLastUs;
extern volatile uint32_t mavlinkLatencyRttAvgUs;
extern volatile uint32_t mavlinkLatencyOneWayAvgUs;
extern volatile uint32_t mavlinkPilotLatencyEstimateUs;

// RX loops are paced to avoid starving other tasks; dispatch is slightly faster.
const int RX_SLOW_MS_PER_TICK = 2; // 500 Hz poll
const int RX_FAST_MS_PER_TICK = 1; // 1000 Hz poll
inline constexpr uint8_t MAVLINK_SYSTEM_ID = 10U;
inline constexpr uint8_t MAVLINK_COMPONENT_ID = MAV_COMP_ID_AUTOPILOT1;

// Logging
extern QueueHandle_t sensorData_logging_queue;
extern QueueHandle_t controlOutput_logging_queue;
extern QueueHandle_t stateVector_logging_queue;
extern QueueHandle_t manualControl_t_logging_queue;
extern QueueHandle_t sensorData_latest_queue;
extern QueueHandle_t stateVector_latest_queue;
extern volatile uint32_t sensorData_logging_drop_count;
extern volatile uint32_t controlOutput_logging_drop_count;
extern volatile uint32_t stateVector_logging_drop_count;
extern volatile uint32_t manualControl_logging_drop_count;

// Manual
extern Controller_t controllerData;
extern SemaphoreHandle_t controllerMutex;

extern SetServoStates_t servoStateData;
extern SemaphoreHandle_t stateMutex;

// Task Declarations
// General
void ImuBaroTask(void *pvParameters);
void GPSTask(void *pvParameters);
void StateTask(void *pvParameters);
void PIDTask(void *pvParameters);
void GSATxTask(void *pvParameters);
void GSARxTask(void *pvParameters);
void BlinkTask(void *pvParameters);
void MavlinkHeartbeatTask(void *pvParameters);
void MavlinkLatencyProbeTask(void *pvParameters);

// Mavlink Tasks
void MavlinkRx900Task(void *pvParameters);
void MavlinkRx24Task(void *pvParameters);
void MavlinkControlDispatchTask(void *pvParameters);
void MavlinkTelemetryDispatchTask(void *pvParameters);
void RxMavlinkProcess900PacketTask(void *pvParameters);
void MavlinkQgcHandshakeTask(void *pvParameters);
void MavlinkSimulatedTelemetryTask(void *pvParameters);
void Serial3LoopbackSelfTestTask(void *pvParameters);

// Manual tasks
void writeServoTask(void *pvParameters);
void updateStatesTask(void *pvParameters);
void radioTask(void *pvParameters);

// Logging task Declarations
void SDCardTask(void *pvParameters);
void LoggingQueueSmokeTestTask(void *pvParameters);
template <typename T>
void FillLoggingQueues(Log<T> log) = delete;
void FillLoggingQueues(Log<StateVector_t> log);
void FillLoggingQueues(Log<SensorData_t> log);
void FillLoggingQueues(Log<ControlOutput_t> log);
void FillLoggingQueues(Log<mavlink_manual_control_t> log);

// Templated functions need to be in header files in order to be accessible and
// proper linking
template <typename T>
inline void ConstructLog(const T &data)
{
    Log<T> log(data);
    log.timestamp = xTaskGetTickCount();
    FillLoggingQueues(log);
}

#endif
