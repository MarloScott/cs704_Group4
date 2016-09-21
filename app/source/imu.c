
#include "imu.h"

#include <stdint.h>

#include "imu_adaptor.h"

void IMU_init(){
    // TODO
}

void IMU_get_reading(void* ctx){

    uint8_t send_data, save_data[6], i;

    // Read Accel X
    for (i=0;i<6;i++){
        send_data = (IMU_SPI_READ<<IMU_SPI_RW_BIT) | (i + IMU_ACCEL_XOUT_H);
        imu_spi_transfer(ctx, 2, &send_data, &save_data[i]);
    }

}
