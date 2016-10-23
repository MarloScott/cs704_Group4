
#include "imu.h"

#include <stdint.h>

#include "../../modules/libmpu9250/lib/mpu9250/mpu9250.h"
#include "imu_adaptor.h"


void IMU_DEVICE_INIT(struct IMU_DEVICE_POSE *devicePose, struct mpu9250_s device)
{
  devicePose->device = device;
  // Initialise angles
  devicePose->angle.X = 0;
  devicePose->angle.Y = 0;
  devicePose->angle.Z = 0;
  //TODO: Calibrate off-sets
  IMU_CALIBRATE(devicePose);

  devicePose->rotation.row1.X =1;
  devicePose->rotation.row1.Y =0;
  devicePose->rotation.row1.Z =0;

  devicePose->rotation.row2.X =0;
  devicePose->rotation.row2.Y =1;
  devicePose->rotation.row2.Z =0;

  devicePose->rotation.row3.X =0;
  devicePose->rotation.row3.Y =0;
  devicePose->rotation.row3.Z =1;

  devicePose->velocity.X = 0;
  devicePose->velocity.Y = 0;
  devicePose->velocity.Z = 0;

  devicePose->distance.X = 0;
  devicePose->distance.Y = 0;
  devicePose->distance.Z = 0;

  devicePose->acceleration.X = 0;
  devicePose->acceleration.Y = 0;
  devicePose->acceleration.Z = 0;

}

void IMU_RESET(struct IMU_DEVICE_POSE *devicePose)
{

}

void IMU_UPDATE(struct IMU_DEVICE_POSE *devicePose)
{
  struct IMU_SAMPLE sample;
  struct MATRIX skew, new, errorCorrect, norm;
  struct VECTOR correctedAccel;
  IMU_READ(&sample, devicePose->device);

  sample.gyro.X = (sample.gyro.X - devicePose->gyroOffset.X)*0.01;
  sample.gyro.Y = (sample.gyro.Y - devicePose->gyroOffset.Y)*0.01;
  sample.gyro.Z = (sample.gyro.Z - devicePose->gyroOffset.Z)*0.01;

  devicePose->angle.X += sample.gyro.X;
  devicePose->angle.Y += sample.gyro.Y;
  devicePose->angle.Z += sample.gyro.Z;

  devicePose->acceleration.X = sample.accel.X;
  devicePose->acceleration.Y = sample.accel.Y;
  devicePose->acceleration.Z = sample.accel.Z;


  SKEW_VECTOR(&skew, &(sample.gyro));
  ERROR_CORRECT(&skew, &errorCorrect);
  NORMALIZE_MATRIX(&errorCorrect, &norm);
  MATRIX_MULTIPLY(&(devicePose->rotation),&norm,&new);
  ERROR_CORRECT(&new, &errorCorrect);
  NORMALIZE_MATRIX(&errorCorrect, &norm);
  devicePose->rotation = norm;

  VECTOR_MULTIPLY(&norm, &(devicePose->acceleration), &correctedAccel);


  devicePose->acceleration.X = correctedAccel.X - devicePose->accelOffset.X;
  devicePose->acceleration.Y = correctedAccel.Y - devicePose->accelOffset.Y;
  devicePose->acceleration.Z = correctedAccel.Z - devicePose->accelOffset.Z;

  devicePose->velocity.X += devicePose->acceleration.X*0.01;
  devicePose->velocity.Y += devicePose->acceleration.Y*0.01;
  devicePose->velocity.Z += devicePose->acceleration.Z*0.01;

  devicePose->distance.X += devicePose->velocity.X*0.01;
  devicePose->distance.Y += devicePose->velocity.Y*0.01;
  devicePose->distance.Z += devicePose->velocity.Z*0.01;

}


void SKEW_VECTOR(struct MATRIX *skew, struct VECTOR *vector)
{
  skew->row1.X = 1;
  skew->row1.Y = vector->Z*-1;
  skew->row1.Z = vector->Y;

  skew->row2.X = vector->Z;
  skew->row2.Y = 1;
  skew->row2.Z = vector->X*-1;

  skew->row3.X = vector->Y*-1;
  skew->row3.Y = vector->X;
  skew->row3.Z = 1;
}

void DOT_PRODUCT(struct VECTOR mA, struct VECTOR mB, float *mOutput)
{
  *mOutput = ((mA.X*mB.X)+(mA.Y*mB.Y)+(mA.Z*mB.Z));
}

void ERROR_CORRECT(struct MATRIX *mA, struct MATRIX *mOutput)
{
  float error;
  struct VECTOR col1 = GET_COLOMN(mA,1);
  struct VECTOR col2 = GET_COLOMN(mA,2);
  struct VECTOR col3 = GET_COLOMN(mA,3);
  struct VECTOR temp;
  DOT_PRODUCT(col1,col2,&error);

  mOutput->row1.X = col1.X - ((error/2)*col2.X);
  mOutput->row2.X = col1.Y - ((error/2)*col2.Y);
  mOutput->row3.X = col1.Z - ((error/2)*col2.Z);

  mOutput->row1.Y = col2.X - ((error/2)*col1.X);
  mOutput->row2.Y = col2.Y - ((error/2)*col1.Y);
  mOutput->row3.Y = col2.Z - ((error/2)*col1.Z);


  col1 = GET_COLOMN(mOutput,1);
  col2 = GET_COLOMN(mOutput,2);
  CROSS_PRODUCT(&col1, &col2, &temp);

  mOutput->row1.Z = temp.X;
  mOutput->row2.Z = temp.Y;
  mOutput->row3.Z = temp.Z;


}

void NORMALIZE_MATRIX(struct MATRIX *mA, struct MATRIX *mOutput)
{
  struct VECTOR col1 = GET_COLOMN(mA,1);
  struct VECTOR col2 = GET_COLOMN(mA,2);
  struct VECTOR col3 = GET_COLOMN(mA,3);

  float dot1, dot2, dot3;

  DOT_PRODUCT(col1,col1,&dot1);
  DOT_PRODUCT(col2,col2,&dot2);
  DOT_PRODUCT(col3,col3,&dot3);

  mOutput->row1.X = 0.5*((3-dot1)*col1.X);
  mOutput->row2.X = 0.5*((3-dot1)*col1.Y);
  mOutput->row3.X = 0.5*((3-dot1)*col1.Z);

  mOutput->row1.Y = 0.5*((3-dot2)*col2.X);
  mOutput->row2.Y = 0.5*((3-dot2)*col2.Y);
  mOutput->row3.Y = 0.5*((3-dot2)*col2.Z);

  mOutput->row1.Z = 0.5*((3-dot3)*col3.X);
  mOutput->row2.Z = 0.5*((3-dot3)*col3.Y);
  mOutput->row3.Z = 0.5*((3-dot3)*col3.Z);
}

struct VECTOR GET_COLOMN(struct MATRIX *mA, int i)
{
  struct VECTOR col;
  if(i == 1){
    col.X = mA->row1.X;
    col.Y = mA->row2.X;
    col.Z = mA->row3.X;
  }else if(i == 2){
    col.X = mA->row1.Y;
    col.Y = mA->row2.Y;
    col.Z = mA->row3.Y;
  }else{
    col.X = mA->row1.Z;
    col.Y = mA->row2.Z;
    col.Z = mA->row3.Z;
  }
  return col;
}

void CROSS_PRODUCT(struct VECTOR *mA, struct VECTOR *mB, struct VECTOR *mOutput)
{
  mOutput->X = ((mA->Y*mB->Z)-(mA->Z*mB->Y));
  mOutput->Y = ((mA->Z*mB->X)-(mA->X*mB->Z));
  mOutput->Z = ((mA->X*mB->Y)-(mA->Y*mB->X));
}

void MATRIX_MULTIPLY(struct MATRIX *mA, struct MATRIX *mB, struct MATRIX *mOutput)
{
  mOutput->row1.X = ((mA->row1.X*mB->row1.X)+(mA->row1.Y*mB->row2.X)+(mA->row1.Z*mB->row3.X));
  mOutput->row1.Y = ((mA->row1.X*mB->row1.Y)+(mA->row1.Y*mB->row2.Y)+(mA->row1.Z*mB->row3.Y));
  mOutput->row1.Z = ((mA->row1.X*mB->row1.Z)+(mA->row1.Y*mB->row2.Z)+(mA->row1.Z*mB->row3.Z));

  mOutput->row2.X = ((mA->row2.X*mB->row1.X)+(mA->row2.Y*mB->row2.X)+(mA->row2.Z*mB->row3.X));
  mOutput->row2.Y = ((mA->row2.X*mB->row1.Y)+(mA->row2.Y*mB->row2.Y)+(mA->row2.Z*mB->row3.Y));
  mOutput->row2.Z = ((mA->row2.X*mB->row1.Z)+(mA->row2.Y*mB->row2.Z)+(mA->row2.Z*mB->row3.Z));

  mOutput->row3.X = ((mA->row3.X*mB->row1.X)+(mA->row3.Y*mB->row2.X)+(mA->row3.Z*mB->row3.X));
  mOutput->row3.Y = ((mA->row3.X*mB->row1.Y)+(mA->row3.Y*mB->row2.Y)+(mA->row3.Z*mB->row3.Y));
  mOutput->row3.Z = ((mA->row3.X*mB->row1.Z)+(mA->row3.Y*mB->row2.Z)+(mA->row3.Z*mB->row3.Z));
}

void VECTOR_MULTIPLY(struct MATRIX *mA, struct VECTOR *mB, struct VECTOR *mOutput)
{
  mOutput->X = ((mA->row1.X*mB->X)+(mA->row1.Y*mB->Y)+(mA->row1.Z*mB->Z));
  mOutput->Y = ((mA->row2.X*mB->X)+(mA->row2.Y*mB->Y)+(mA->row2.Z*mB->Z));
  mOutput->Z = ((mA->row3.X*mB->X)+(mA->row3.Y*mB->Y)+(mA->row3.Z*mB->Z));
}

void IMU_CALIBRATE(struct IMU_DEVICE_POSE *devicePose)
{
  //TODO: Implement for loop and average samples
  struct IMU_SAMPLE sample;
  IMU_READ(&sample, devicePose->device);

  devicePose->accelOffset.X = sample.accel.X;
  devicePose->accelOffset.Y = sample.accel.Y;
  devicePose->accelOffset.Z = sample.accel.Z;

  devicePose->gyroOffset.X = sample.gyro.X;
  devicePose->gyroOffset.Y = sample.gyro.Y;
  devicePose->gyroOffset.Z = sample.gyro.Z;
}

void IMU_READ(struct IMU_SAMPLE *sample, struct mpu9250_s device)
{
  IMU_POLL_DATA_RDY(&device);
  mpu9250_read_gyro(&device, &(sample->gyro.X),&(sample->gyro.Y),&(sample->gyro.Z));
  mpu9250_read_accel(&device, &(sample->accel.X),&(sample->accel.Y),&(sample->accel.Z));
}

void IMU_Handler()
{

}

void IMU_POLL_DATA_RDY(struct mpu9250_s *device)
{

  int IMU_FLAG = 0;
  uint8_t status_data;
  int res;

  while(IMU_FLAG==0){
    res = mpu9250_read_reg(device, MPU9250_REG_INT_STATUS,&status_data);
    if (res < 0) {
        error_flash(1, -res);
    }
    if(status_data == 1){
      IMU_FLAG = 1;
      return;
    }
  }
}

// Read a single register from the device
static int mpu9250_read_reg(struct mpu9250_s *device, uint8_t reg, uint8_t* val)
{
    uint8_t data_out[2] = {0xFF, 0xFF};
    uint8_t data_in[2] = {0xFF, 0xFF};
    int res;

    data_out[0] = reg | MPU9250_REG_READ_FLAG;
    data_out[1] = 0x00;

    res = device->driver->spi_transfer(device->driver_ctx, 2, data_out, data_in);

    if (res >= 0) {
        *val = data_in[1];
    }

    return res;
}
