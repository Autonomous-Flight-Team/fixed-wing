// Authors
// - Colin Faletto github.com/faletto


#ifndef HARDWARE_H
#define HARDWARE_H


#include "tasks.h"

// Dependant on final hardware
#include <ICM_20948.h>
#include <Adafruit_BMP3XX.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

bool HardwareInit(void);

extern ICM_20948_I2C imu;
extern Adafruit_BMP3XX bmp;
extern TinyGPSPlus gps;

#define gpsSerial Serial1


#endif