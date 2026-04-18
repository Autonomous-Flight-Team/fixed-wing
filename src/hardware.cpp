// Authors
// - Colin Faletto github.com/faletto


#include "hardware.h"

ICM_20948_I2C imu;
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;

#define IMU_SDA 18
#define IMU_SCL 19

// Initializes the IMU
bool ImuInit() {
    Wire.begin();
    if (imu.begin(Wire) != ICM_20948_Stat_Ok) {
        Serial.println("IMU failed :(");
        return false;
    }
    return true;
}

// Initializes the Barometer
bool BaroInit() {
    if (!bmp.begin_I2C()) {
        Serial.println("Barometer failed :(");
        return false;
    }
    return true;
}

int GPS_BAUD = 9600;

// Initializes the GPS
bool GPSInit() {
    gpsSerial.begin(GPS_BAUD);
    return true;
}

bool SDInit() {
    if (SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD Card failed :(");
        return false;
    }
    return true;
}

// Initializes all Hardware
bool HardwareInit() {
    return ImuInit() && BaroInit() && GPSInit() && SDInit();
}
