#ifndef IMU_H
#define IMU_H

#include "mpu9250/mpu9250.h"
#include "mpu9250/mpu9250_regs.h"

#include <stdint.h>

#define IMU_SPI_RW_BIT 7
#define IMU_SPI_READ 1
#define IMU_SPI_WRITE 0

#define IMU_INT_SET 55
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


struct VECTOR{
  float X;
  float Y;
  float Z;
};

struct MATRIX{
  struct VECTOR row1;
  struct VECTOR row2;
  struct VECTOR row3;
};

struct IMU_DEVICE_POSE{
  struct MATRIX rotation;
  struct VECTOR angle;
  struct VECTOR acceleration;
  struct VECTOR velocity;
  struct VECTOR distance;
  struct VECTOR accelOffset;
  struct VECTOR gyroOffset;
  struct mpu9250_s device;
};

struct IMU_SAMPLE{
  struct VECTOR accel;
  struct VECTOR gyro;
};


// TODO: write IMU gyro calibrating function
//int IMU_GYRO_CALIBRATE(struct mpu9250_s *device);

// Blocking function to pause untill next IMU data is avaliable.
// TODO: Should be replaced with interrupt.
void IMU_POLL_DATA_RDY(struct mpu9250_s *device);

void IMU_DEVICE_INIT(struct IMU_DEVICE_POSE *devicePose, struct mpu9250_s device);

void IMU_CALIBRATE(struct IMU_DEVICE_POSE *devicePose);

void IMU_READ(struct IMU_SAMPLE *sample, struct mpu9250_s device);

struct VECTOR GET_COLOMN(struct MATRIX *mA, int i);



#endif
