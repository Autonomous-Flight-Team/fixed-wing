// Authors
// - Colin Faletto github.com/faletto


#include "hardware.h"

// Initializes all Hardware
bool HardwareInit() {
    return ImuInit() && BaroInit() && GPSInit();
}

// Initializes the IMU
bool ImuInit() {
    Wire.begin();
    if (imu.begin() != ICM_20948_Stat_Ok) return false;
    imu.setAccelRange(ICM_20948_ACCEL_RANGE_4G);
    imu.setGyroRange(ICM_20948_GYRO_RANGE_500DPS);
    return true;
}

int BARO_OVERSAMPLING_RATE = 8;

// Initializes the Barometer
bool BaroInit() {
    if (!bmp.begin_I2C()) return false;
    bmp.setOversampling(BARO_OVERSAMPLING_RATE);
    return true;
}

int GPS_PORT = 1; // TODO
int GPS_BAUD = 9600;

HardwareSerial gpsSerial(GPS_PORT);

// Initializes the GPS
bool GPSInit() {
    gpsSerial.begin(GPS_BAUD);
    return true;
}