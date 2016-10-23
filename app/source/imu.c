
#include "imu.h"

#include <stdint.h>

#include "../../modules/libmpu9250/lib/mpu9250/mpu9250.h"
#include "imu_adaptor.h"



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
