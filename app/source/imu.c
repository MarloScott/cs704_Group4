
#include "imu.h"

#include <stdint.h>

#include "../../modules/libmpu9250/lib/mpu9250/mpu9250.h"
#include "imu_adaptor.h"

int8_t IMU_init(void* ctx){
    struct mpu9250_s mpu9250;

    struct mpu9250_driver_s mpu9250_driver;
    mpu9250_driver.spi_transfer = &imu_spi_transfer;

    return mpu9250_init(&mpu9250, &mpu9250_driver, ctx);
}

void IMU_get_readings(void* ctx, uint8_t* save_data){

    uint8_t send_data, i;

    // Read Accel X
    for (i=0;i<1;i++){
        send_data = (IMU_SPI_READ << IMU_SPI_RW_BIT) | (IMU_ACCEL_XOUT_H + i);
        imu_spi_transfer(ctx, 2, &send_data, &save_data[i]);
    }

}
