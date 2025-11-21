// Authors
// - Colin Faletto github.com/faletto

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "tasks/tasks.h"
#include "hardware.h"



SemaphoreHandle_t dataMutex;

int STACK_DEPTH = 512;
int[] priority = {1,2,3,4};

typedef struct {
    // IMU
        // Linear Acceleration
        float ax, ay, az;
        // Rotational Velocity
        float gx, gy, gz;
    // Barometer
        float altitude;
        float pressure;
        float temp;
    // GPS
        // Latitude
        double lat;
        // Longitude
        double lon;
        float gps_altitude;
        // Linear Velocity
        float vs; 

} SensorData_t;

typedef struct {
    // Linear Position
    double x, double y, double z,
    // Linear Velocity
    double u, double v, double w,
    // Angular Velocity
    double phi, double theta, double psi,
    // Angular Acceleration
    double p, double q, double r
} StateVector_t;

typedef struct {
    //TODO
} ControlOutput_t;

SensorData_t sensorData = {0};
ControlOutput_t controlOutput = {0};
StateVector_t stateVector = {0};

// Program Entry Point
int main(void) {
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
    xTaskCreate(DSHOTTask, "DShot", STACK_DEPTH, NULL, *priority, NULL);
    xTaskCreate(GSARxTask, "GSARx", STACK_DEPTH, NULL, *priority+3, NULL);
    xTaskCreate(GSATxTask, "GSATx", STACK_DEPTH, NULL, *priority+3, NULL);

    vTaskStartScheduler();

    return 0;

}


