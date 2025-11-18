// Authors
// - Colin Faletto github.com/faletto


#ifndef HARDWARE_H
#define HARDWARE_H

#include "FreeRTOS.h"
#include "tasks/tasks.h"

// Dependant on final hardware
#include <ICM_20948.h>
#include <Adafruit_BMP3XX.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

void HardwareInit(void);

ICM_20948_I2C imu;
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;

HardwareSerial gpsSerial;


#endif