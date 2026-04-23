// Authors
// - Colin Faletto github.com/faletto

#include "hardware.h"

ICM_20948_I2C imu;
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;
Adafruit_LSM9DS1 lsm;

// Initializes the IMU
bool ImuInit()
{
    Wire.begin();
    if (!lsm.begin())
        return false;
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    return true;
}

// Initializes the Barometer
bool BaroInit()
{
    if (!bmp.begin_I2C())
        return false;
    return true;
}

int GPS_BAUD = 9600;

// Initializes the GPS
bool GPSInit()
{
    gpsSerial.begin(GPS_BAUD);
    return true;
}

// Initializes all Hardware
bool HardwareInit()
{
    return ImuInit() && BaroInit(); //&& GPSInit();
}
