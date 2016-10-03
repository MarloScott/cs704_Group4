
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "stm32l1xx.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

#include "cs704_version.h"

#include "board.h"
#include "gpio.h"
#include "spi.h"
#include "radio_adaptor.h"
#include "imu_adaptor.h"

#include "fifteenfour.h"
#include "beacon.h"

#include "mpu9250/mpu9250.h"
#include "at86rf212/at86rf212.h"

#include "imu.h"

#define GROUP_N       4

// Network configuration
#define PAN_ID        (0x0010|GROUP_N)  //!< PAN ID for beacon network
#define ADDRESS       0x0001            //!< Master address is 0x0001, other beacons must increment this
#define CHANNEL       (2+GROUP_N)       //!< Channel for beacon use

//#define BEACON_MODE
#define USB_SERIAL

// ISR globals
__IO uint32_t systick_count = 0;

// USB globals (interrupt driven)
__IO uint32_t packet_sent;
__IO uint32_t packet_receive;
__IO uint8_t Receive_Buffer[64];

void run_example(struct at86rf212_s *radio, uint16_t pan, uint16_t address);

// printf binding for newlib (nano)
int _write_r(struct _reent *r, int fd, const void *data, unsigned int count)
{
#ifdef USB_SERIAL
    CDC_Send_DATA((uint8_t *)data, count);
#endif
    return 0;
}

// Elliots USB print function, use this one
void USB_print(char* string){
#ifdef USB_SERIAL
    #include "string.h"
    uint8_t length = strlen(string);
    CDC_Send_DATA((uint8_t *)string, length);
    delay_ms(1);
#endif
}

// Systick ISR, useful for timing
void SysTick_Handler(void)
{
    systick_count ++;
}

void delay_ms(uint32_t ms)
{
    uint32_t now = systick_count;
    while (systick_count < (now + ms));
}

void delay_us(uint32_t us)
{
    for (volatile int i = 0; i < (SystemCoreClock / 1000 / 1000); i++);
}

void error_flash(int error_id, int error_code)
{
    int i;
    error_id = (error_id < 0) ? -error_id : error_id;
    error_code = (error_code < 0) ? -error_code : error_code;

    while (1) {

        for (i = 0; i < error_id; i++) {
            GPIO_write(LED1_PORT, LED1_PIN, 1);
            delay_ms(100);
            GPIO_write(LED1_PORT, LED1_PIN, 0);
            delay_ms(100);
        }

        delay_ms(500);

        for (i = 0; i < error_code; i++) {
            GPIO_write(LED1_PORT, LED1_PIN, 1);
            delay_ms(100);
            GPIO_write(LED1_PORT, LED1_PIN, 0);
            delay_ms(100);
        }

        delay_ms(2000);
    }
}


int main(void)
{
    int res;

    // Enable systick timer
    SysTick_Config(SystemCoreClock / 1000);

    // Initialise GPIO
    GPIO_init();

    // Initialise USB CDC
#ifdef USB_SERIAL
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();
#endif

    // Initialise radio SPI
    struct spi_ctx_s radio_spi_ctx = RADIO_SPI_DEFAULT,
                     imu_spi_ctx   = IMU_SPI_DEFAULT;
    SPI_init(&radio_spi_ctx, 0, 0);
    SPI_init(&imu_spi_ctx, 0, 0);

    delay_ms(10);

    // Initialise radio
    struct at86rf212_s radio;
    struct at86rf212_driver_s radio_driver = RADIO_ADAPTOR_DEFAULT;
    res = at86rf212_init(&radio, &radio_driver, (void*) &radio_spi_ctx);
    if (res < 0) {
        error_flash(1, -res);
    }

    at86rf212_set_channel(&radio, CHANNEL);

#ifdef BEACON_MODE
    // Master / Slave Beacon Logic
    if (ADDRESS == 0x0001) {
        run_master(&radio, PAN_ID, ADDRESS);
    } else {
        run_slave(&radio, PAN_ID, ADDRESS);
    }
#endif

    // Initialising MPU9250
    struct mpu9250_s mpu9250;
    struct mpu9250_driver_s mpu9250_driver;
    mpu9250_driver.spi_transfer = &imu_spi_transfer;
    mpu9250_init(&mpu9250, &mpu9250_driver, &imu_spi_ctx);

    uint16_t x,y,z;
    int recieved_mpu_value;
    char txt_buffer[32];
    delay_ms(4000);
    // ------

    // Initializing RF
    struct at86rf212_s RF_device;
    struct at86rf212_driver_s RF_driver;
    RF_driver.spi_transfer = &radio_spi_transfer;
    res = at86rf212_init(&RF_device, &RF_driver, &radio_spi_ctx);
    //at86rf212_set_channel(&RF_device, 1);
    if (res < 0) {
        error_flash(1, -res);
    }
    //at86rf212_set_state(&RF_device, 6);
    uint8_t length;
    uint8_t data[18];
    uint8_t strength[5];
    res = at86rf212_start_rx(&RF_device);
    if (res < 0) {
        error_flash(1, -res);
    }

    while(1) {
        //recieved_mpu_value = mpu9250_read_gyro_raw(&mpu9250, &x,&y,&z);
        sprintf(txt_buffer, "start\n");
        USB_print(txt_buffer);
        while(at86rf212_check_rx(&RF_device)<1){
          sprintf(txt_buffer, "start\n");
          USB_print(txt_buffer);
          delay_ms(1);
        }
        at86rf212_get_rx(&RF_device, &length, data);
        strength[data[7]-1] = data[16];
        sprintf(txt_buffer, "Node1: %d,Node2: %d,Node3: %d,Node4: %d,  \n", strength[0],  strength[1], strength[2], strength[3] );
        USB_print(txt_buffer);
        sprintf(txt_buffer, "length: %d \n", length);
        USB_print(txt_buffer);
        for(int i = 0;i<length;i++){
          sprintf(txt_buffer, "%.2x ", data[i]);
          USB_print(txt_buffer);
        }
        sprintf(txt_buffer, "\n");
        USB_print(txt_buffer);
        // Pin flashing test
        LED0_PORT->ODR ^= LED0_PIN;
        delay_ms(100);
        LED0_PORT->ODR ^= LED0_PIN;
        delay_ms(1000);
    }

}
