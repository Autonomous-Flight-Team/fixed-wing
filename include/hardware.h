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
#include <Adafruit_LSM9DS1.h>

bool HardwareInit(void);
bool intialize_manual_control();

extern ICM_20948_I2C imu;
extern Adafruit_BMP3XX bmp;
extern TinyGPSPlus gps;
extern Adafruit_LSM9DS1 lsm;

#define gpsSerial Serial1

#define MAVLINK_COMM_900 MAVLINK_COMM_0
#define MAVLINK_COMM_24 MAVLINK_COMM_1
inline constexpr uint32_t MAVLINK_BAUD = 460800;

// TODO: Update these UARTs to match our wiring
#define MAVLINK_SERIAL_900 Serial2
#define MAVLINK_SERIAL_24 Serial3

#endif