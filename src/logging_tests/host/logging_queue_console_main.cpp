#include <cstdint>
#include <iomanip>
#include <iostream>
#include <queue>
#include <type_traits>

struct SensorData_t {
    float ax = 0.0f;
};

struct StateVector_t {
    double x = 0.0;
};

struct ControlOutput_t {
    double somethingneedstobehere = 0.0;
};

struct mavlink_manual_control_t {
    int16_t x = 0;
};

template <typename T>
struct Log {
    T data{};
    uint32_t timestamp = 0;
};

std::queue<Log<SensorData_t>> sensorData_logging_queue;
std::queue<Log<ControlOutput_t>> controlOutput_logging_queue;
std::queue<Log<StateVector_t>> stateVector_logging_queue;
std::queue<Log<mavlink_manual_control_t>> manualControl_t_logging_queue;

template <typename T>
void FillLoggingQueues(Log<T>) = delete;

void FillLoggingQueues(Log<SensorData_t> log) { sensorData_logging_queue.push(log); }
void FillLoggingQueues(Log<ControlOutput_t> log) { controlOutput_logging_queue.push(log); }
void FillLoggingQueues(Log<StateVector_t> log) { stateVector_logging_queue.push(log); }
void FillLoggingQueues(Log<mavlink_manual_control_t> log) { manualControl_t_logging_queue.push(log); }

template <typename T>
void ConstructLog(const T &data, uint32_t ts) {
    Log<T> log{};
    log.data = data;
    log.timestamp = ts;
    FillLoggingQueues(log);
}

template <typename T>
bool Pop(std::queue<Log<T>> &q, Log<T> &out) {
    if (q.empty()) return false;
    out = q.front();
    q.pop();
    return true;
}

int main() {
    std::cout << "[LOG-TEST] Host queue smoke test\n";

    ConstructLog(SensorData_t{1.25f}, 1001U);
    ConstructLog(ControlOutput_t{42.0}, 1002U);
    ConstructLog(StateVector_t{123.0}, 1003U);
    ConstructLog(mavlink_manual_control_t{77}, 1004U);

    Log<SensorData_t> sensorOut{};
    Log<ControlOutput_t> controlOut{};
    Log<StateVector_t> stateOut{};
    Log<mavlink_manual_control_t> manualOut{};

    const bool gotSensor = Pop(sensorData_logging_queue, sensorOut);
    const bool gotControl = Pop(controlOutput_logging_queue, controlOut);
    const bool gotState = Pop(stateVector_logging_queue, stateOut);
    const bool gotManual = Pop(manualControl_t_logging_queue, manualOut);

    std::cout << "[LOG-TEST] sensor: " << (gotSensor ? "OK" : "FAIL");
    if (gotSensor) std::cout << " ax=" << std::fixed << std::setprecision(3) << sensorOut.data.ax
                             << " ts=" << sensorOut.timestamp;
    std::cout << "\n";

    std::cout << "[LOG-TEST] control: " << (gotControl ? "OK" : "FAIL");
    if (gotControl) std::cout << " out=" << std::fixed << std::setprecision(3)
                              << controlOut.data.somethingneedstobehere
                              << " ts=" << controlOut.timestamp;
    std::cout << "\n";

    std::cout << "[LOG-TEST] state: " << (gotState ? "OK" : "FAIL");
    if (gotState) std::cout << " x=" << std::fixed << std::setprecision(3) << stateOut.data.x
                            << " ts=" << stateOut.timestamp;
    std::cout << "\n";

    std::cout << "[LOG-TEST] manual: " << (gotManual ? "OK" : "FAIL");
    if (gotManual) std::cout << " x=" << manualOut.data.x << " ts=" << manualOut.timestamp;
    std::cout << "\n";

    return (gotSensor && gotControl && gotState && gotManual) ? 0 : 1;
}
