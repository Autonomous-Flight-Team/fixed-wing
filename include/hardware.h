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
#include <SD.h>

bool HardwareInit(void);
bool initialize_manual_control();

extern ICM_20948_I2C imu;
extern Adafruit_BMP3XX bmp;
extern TinyGPSPlus gps;
extern Adafruit_LSM9DS1 lsm;

#define gpsSerial Serial1

// Separates the UART Data Streams into their own buffers to prevent overlap and corruption
#define MAVLINK_COMM_900 MAVLINK_COMM_0
#define MAVLINK_COMM_24 MAVLINK_COMM_1

// Both have their own Buads
inline constexpr uint32_t MAVLINK_BAUD_900 = 460800;
inline constexpr uint32_t MAVLINK_BAUD_24 = 115200;

// TODO: Update these UARTs to match our wiring
// Actual Hardware RX-TX references
#define MAVLINK_SERIAL_900 Serial2
#define MAVLINK_SERIAL_24 Serial3 

#endif