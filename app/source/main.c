
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

#define GROUP_N       4

// Network configuration
#define PAN_ID        (0x0010|GROUP_N)  //!< PAN ID for beacon network
#define ADDRESS       0x0001            //!< Master address is 0x0001, other beacons must increment this
#define CHANNEL       (2+GROUP_N)       //!< Channel for beacon use

//#define BEACON_MODE
//#define USB_SERIAL

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
    struct spi_ctx_s radio_spi_ctx = RADIO_SPI_DEFAULT;
    SPI_init(&radio_spi_ctx, 0, 0);

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

    while(1) {
        // Pin flashing test
        LED0_PORT->ODR ^= LED0_PIN;
        delay_ms(1000);
        LED0_PORT->ODR ^= LED0_PIN;
        delay_ms(200);
    }

}
