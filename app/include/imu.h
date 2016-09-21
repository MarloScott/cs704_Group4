#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#define IMU_SPI_RW_BIT 7
#define IMU_SPI_READ 1
#define IMU_SPI_WRITE 0

#define IMU_INT_PIN 55
#define IMU_INT_EN  56
    #define IMU_RAW_RDY_EN 0
#define IMU_INT_STATUS 58
    #define IMU_RAW_DATA_RDY_INT 0


// Accelerometer Registers
#define IMU_ACCEL_XOUT_H 59
#define IMU_ACCEL_XOUT_L 60
#define IMU_ACCEL_YOUT_H 61
#define IMU_ACCEL_YOUT_L 62
#define IMU_ACCEL_ZOUT_H 63
#define IMU_ACCEL_ZOUT_L 64

// Gyroscope Registers
#define IMU_GYRO_XOUT_H 67
#define IMU_GYRO_XOUT_L 68
#define IMU_GYRO_YOUT_H 69
#define IMU_GYRO_YOUT_L 70
#define IMU_GYRO_ZOUT_H 71
#define IMU_GYRO_ZOUT_L 72

void IMU_init();
void IMU_get_reading();

#endif
