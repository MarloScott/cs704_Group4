
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
#include "mpu9250/mpu9250_regs.h"
#include "at86rf212/at86rf212.h"

#include "imu.h"

#define GROUP_N       4

// Network configuration
#define PAN_ID        (0x0010|GROUP_N)  //!< PAN ID for beacon network
#define ADDRESS       0x0001            //!< Master address is 0x0001, other beacons must increment this
#define CHANNEL       (2+GROUP_N)       //!< Channel for beacon use

//#define BEACON_MODE
#define USB_SERIAL
//#define BaseSerial
//MODES OF OPERATION
//#define BECON_STRENGTHS
#define ACCEL_RAW
//#define SEND_MSG
//#define RECV_MSG

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
    delay_ms(10);
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

void send_message(struct at86rf212_s *radio, uint8_t *data)
{
  int res;
  struct fifteen_four_header_s header_out = FIFTEEN_FOUR_DEFAULT_HEADER(1, 1, 1, 1);
  data;

  // Build packet
  uint8_t packet[sizeof(struct fifteen_four_header_s) + sizeof(data)];
  memcpy(packet, &header_out, sizeof(struct fifteen_four_header_s));
  memcpy(packet + sizeof(struct fifteen_four_header_s), data, sizeof(data));

  // Start send
  res = at86rf212_start_tx(radio, sizeof(packet), packet);
  if (res < 0) {
      error_flash(9, -res);
  }

  // Await completion
  while ((res = at86rf212_check_tx(radio)) == 0) {
      delay_ms(1);
  }
}

uint8_t status_data;

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

    #ifdef BEACON_MODE
        // Master / Slave Beacon Logic
        if (ADDRESS == 0x0001) {
            run_master(&radio, PAN_ID, ADDRESS);
        } else {
            run_slave(&radio, PAN_ID, ADDRESS);
        }
    #endif

    // Initialise radio SPI
    struct spi_ctx_s radio_spi_ctx = RADIO_SPI_DEFAULT, imu_spi_ctx   = IMU_SPI_DEFAULT;
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

    //at86rf212_set_channel(&radio, CHANNEL);



    // Initialising MPU9250
    struct mpu9250_s mpu9250;
    struct mpu9250_driver_s mpu9250_driver;
    mpu9250_driver.spi_transfer = &imu_spi_transfer;
    mpu9250_init(&mpu9250, &mpu9250_driver, &imu_spi_ctx);

    int16_t x,y,z;
    int16_t x1, y1, z1;
    float testf = 10.2;
    int recieved_mpu_value;
    char txt_buffer[256];
    delay_ms(4000);
    sprintf(txt_buffer, "Prog START NOW\n");
    USB_print(txt_buffer);

    int i;

    uint8_t length;
    uint8_t data[18];
    uint8_t strength[4]={0};
    res = at86rf212_start_rx(&radio);
    if (res < 0) {
        error_flash(1, -res);
    }


    while(1) {

      #ifdef BECON_STRENGTHS
        while(at86rf212_check_rx(&radio)<1){
            delay_ms(10);
        }
        at86rf212_get_rx(&radio, &length, data);
        if(data[7]>0&&data[7]<5){
            strength[data[7]-1] = data[16];
        }else{
            sprintf(txt_buffer, "ERROR1");
            USB_print(txt_buffer);
        }
        sprintf(txt_buffer, "Node1: [%.2d] ,Node2: [%.2d],Node3: [%.2d],Node4: [%.2d]\n", strength[0],  strength[1], strength[2], strength[3] );
        USB_print(txt_buffer);
      #endif

      #ifdef ACCEL_RAW
      struct IMU_DEVICE_POSE test;
      //IMU_DEVICE_INIT(&test, mpu9250);
      while(1)
      {
        //IMU_UPDATE(&test);
        sprintf(txt_buffer, "test: \n");
      }
        //IMU_POLL_DATA_RDY(&mpu9250);
        //recieved_mpu_value = mpu9250_read_gyro_raw(&mpu9250, &x,&y,&z);
        //recieved_mpu_value = mpu9250_read_accel_raw(&mpu9250, &x1,&y1,&z1);
        //sprintf(txt_buffer, "GYRO : X: %d Y: %d Z: %d ACCEl : X: %d Y: %d Z: %d \n", x,y,z,x1, y1, z1);
        //USB_print(txt_buffer);
      #endif

      #ifdef SEND_MSG
        at86rf212_set_channel(&radio, CHANNEL);
        uint8_t test_data[] = {0xdd, 0xcc, 0xbb, 0xaa};
        send_message(&radio,test_data);
      #endif

      #ifdef RECV_MSG
        at86rf212_set_channel(&radio, CHANNEL);
        while(at86rf212_check_rx(&radio)<1){
            delay_ms(10);
        }
        at86rf212_get_rx(&radio, &length, data);
        i = 0;
        for(i;i<length;i++){
          sprintf(txt_buffer, " %.2x ",data[i]);
          USB_print(txt_buffer);
        }
        sprintf(txt_buffer, "\n",data[i]);
        USB_print(txt_buffer);
      #endif

      #ifdef BaseSerial
      int message[4];
        message[0]=1;
        message[1]=2;
        message[2]=3;
        int i=0;
        while(1){
          sprintf(txt_buffer, "%d \n ",message[i]);
          USB_print(txt_buffer);
          i++;
          if(i==4){
            i=0;
          }
      }

      #endif


    }

}
