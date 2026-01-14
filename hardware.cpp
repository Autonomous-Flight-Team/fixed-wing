// Authors
// - Colin Faletto github.com/faletto


#include "hardware.h"

ICM_20948_I2C imu;
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;



// Initializes the IMU
bool ImuInit() {
    Wire.begin();
    if (imu.begin() != ICM_20948_Stat_Ok) return false;
    return true;
}

// Initializes the Barometer
bool BaroInit() {
    if (!bmp.begin_I2C()) return false;
    return true;
}

int GPS_BAUD = 9600;

// Initializes the GPS
bool GPSInit() {
    gpsSerial.begin(GPS_BAUD);
    return true;
}

// Initializes all Hardware
bool HardwareInit() {
    return ImuInit() && BaroInit() && GPSInit();
}
