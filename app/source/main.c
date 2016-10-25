
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
#include "trilaterate.h"
#include "kalman.h"

#define GROUP_N       4

// Network configuration
#define PAN_ID        (0x0010|GROUP_N)  //!< PAN ID for beacon network
#define ADDRESS       0x0001            //!< Master address is 0x0001, other beacons must increment this
#define CHANNEL       (10+GROUP_N)      //!< Channel for beacon use

//#define INTER_TEST
//#define TRI_TEST

//#define BEACON_MODE

#define USB_SERIAL
//#define BaseSerial
//MODES OF OPERATION
#define BECON_STRENGTHS
//#define ACCEL_RAW
#define SEND_MSG
//#define RECV_MSG

// IMU functs
//#define IMU_INIT_ON
//#define IMU_TEST_ON

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


    uint8_t length;
    uint8_t data[18];
    char txt_buffer[256];
    int i,j;
#ifdef IMU_INIT_ON
    // Initialising MPU9250
    struct mpu9250_s mpu9250;
    struct mpu9250_driver_s mpu9250_driver;
    mpu9250_driver.spi_transfer = &imu_spi_transfer;
    mpu9250_init(&mpu9250, &mpu9250_driver, &imu_spi_ctx);

    int16_t x,y,z;
    int16_t x1, y1, z1;
    float testf = 10.2;
    int recieved_mpu_value;
    delay_ms(2000);
    sprintf(txt_buffer, "Prog START NOW\n");
    USB_print(txt_buffer);



    res = at86rf212_start_rx(&radio);
    if (res < 0) {
        error_flash(1, -res);
    }
#endif

#ifdef INTER_TEST
    Point B[2] = {
        {100, 0},
        {0, 100},};
    Point *Bp[2] = {&B[0],&B[1]};
    int32_t D[2] = {80,80};
    Intersects I;
    while(1) {
        calculate_intersects(Bp, D, &I);
        sprintf(txt_buffer, "(%ld,%ld) (%ld,%ld)\n",I.I1.x,I.I1.y,I.I2.x,I.I2.y);
        USB_print(txt_buffer);
        delay_ms(1000);
    }
#endif

#ifdef TRI_TEST
    Point P_est = {5000,7000};
    uint8_t EDs[4] = { 10,12,5,15};
    do {
        trilaterate(EDs, &P_est, &P_est);
        sprintf(txt_buffer, "(%ld,%ld)\n",P_est.x,P_est.y);
        USB_print(txt_buffer);
        delay_ms(1000);
    } while(1);
#endif
    Point P_out = {5000,5000};
    Point P_est = {5000,5000};
    uint8_t str_av[4];
    uint8_t strength[4][5] = {25};
    uint8_t str_count[4] = {0};
    int16_t test_data[2] = {1234,-2678};

    while(1) {

    #ifdef BECON_STRENGTHS
        at86rf212_set_channel(&radio, 1);
        at86rf212_start_rx(&radio);
        for(i=0;i<20;i++){
            while(at86rf212_check_rx(&radio)<1){
                delay_ms(1);
            }
            at86rf212_get_rx(&radio, &length, data);
            if(data[7]>0&&data[7]<5){
                strength[data[7]-1][str_count[data[7]-1]] = data[16];
                str_count[data[7]-1] = (str_count[data[7]-1]+1)%5;
            }else{
                sprintf(txt_buffer, "ERROR1\n");
                USB_print(txt_buffer);
            }
        }
        uint32_t total[4];
        for(i=0;i<4;i++){
            total[i] = 0;
            for(j=0;j<5;j++){
                total[i] += strength[i][j];
            }
            str_av[i] = total[i]/5;
        }

        trilaterate(str_av, &P_est, &P_out);

        //P_est.x += imuInfo.distance.X;
        //P_est.y += imuInfo.distance.Y;

        //sprintf(txt_buffer, "X: %d Y: %d Z: %dX: %d Y: %d Z: %dX: %d Y: %d Z: %d\n",(int)((imuInfo.distance.X)*100),(int)((imuInfo.distance.Y)*100),(int)((imuInfo.distance.Z)*100),
        //(int)((imuInfo.rotation.row2.X)*100),(int)((imuInfo.rotation.row2.Y)*100),(int)((imuInfo.rotation.row2.Z)*100),
        //(int)((imuInfo.rotation.row3.X)*100),(int)((imuInfo.rotation.row3.Y)*100),(int)((imuInfo.rotation.row3.Z)*100));

        USB_print(txt_buffer);
        //IMU_RESET(imuInfo);

        Guassian2d prediction;
        prediction.mean = P_est;
        prediction.std = 1500;

        Guassian2d measurement;
        measurement.mean = P_out;
        measurement.std = 2500;

        Guassian2d fused;

        kalman2d(&prediction, &measurement, 1, &fused);

        P_est = fused.mean;
        //P_est = P_out;

        sprintf(txt_buffer, "\rB1[%.2d] B2[%.2d] B3[%.2d] B4[%.2d] => (%ld,%ld)@%ld    ",
                            str_av[0],  str_av[1], str_av[2], str_av[3],P_est.x,P_est.y,fused.std);
        USB_print(txt_buffer);

        test_data[0] = P_est.x;
        test_data[1] = P_est.y;

    #endif

      #ifdef ACCEL_RAW
      struct IMU_DEVICE_POSE test;
      IMU_DEVICE_INIT(&test, mpu9250);
      int val = 0;

      struct VECTOR sample,skew;
      SKEW_VECTOR(&skew, &sample);
      while(1)
      {
        val = (val+1)%100;
        if(val==0){
          IMU_RESET(&test);
        }
        IMU_UPDATE(&test);
        sprintf(txt_buffer, "X: %d Y: %d Z: %d\n",(int)((test.distance.X)*100),(int)((test.distance.Y)*100),(int)((test.distance.Z)*100));

        //sprintf(txt_buffer, "X: %d Y: %d Z: %d\n",(int)((test.acceleration.X)*100),(int)((test.acceleration.Y)*100),(int)((test.acceleration.Z)*100));

        //sprintf(txt_buffer, "X: %d Y: %d Z: %d\n",(int)((test.velocity.X)*100),(int)((test.velocity.Y)*100),(int)((test.velocity.Z)*100));

        // sprintf(txt_buffer, "X: %d Y: %d Z: %dX: %d Y: %d Z: %dX: %d Y: %d Z: %d\n",(int)((test.rotation.row1.X)*100),(int)((test.rotation.row1.Y)*100),(int)((test.rotation.row1.Z)*100),
        // (int)((test.rotation.row2.X)*100),(int)((test.rotation.row2.Y)*100),(int)((test.rotation.row2.Z)*100),
        // (int)((test.rotation.row3.X)*100),(int)((test.rotation.row3.Y)*100),(int)((test.rotation.row3.Z)*100));

        USB_print(txt_buffer);
      }
        //IMU_POLL_DATA_RDY(&mpu9250);
        //recieved_mpu_value = mpu9250_read_gyro_raw(&mpu9250, &x,&y,&z);
        //recieved_mpu_value = mpu9250_read_accel_raw(&mpu9250, &x1,&y1,&z1);
        //sprintf(txt_buffer, "GYRO : X: %d Y: %d Z: %d ACCEl : X: %d Y: %d Z: %d \n", x,y,z,x1, y1, z1);
        //USB_print(txt_buffer);
      #endif

      #ifdef SEND_MSG
        at86rf212_set_channel(&radio, CHANNEL);
        send_message(&radio,(uint8_t *)test_data);
      #endif

      #ifdef RECV_MSG
      int xPos;
      int yPos;
        at86rf212_set_channel(&radio,CHANNEL);
        at86rf212_start_rx(&radio);
        while(at86rf212_check_rx(&radio)<1){
          //sprintf(txt_buffer, "waiting");
          //USB_print(txt_buffer);
            delay_ms(10);
        }


        at86rf212_get_rx(&radio, &length, data);

        xPos= data[10]*256+data[9];
        yPos=data[12]*256+data[11];
        sprintf(txt_buffer, "*%.5ld",xPos);
        USB_print(txt_buffer);
        sprintf(txt_buffer, "%.5ld",yPos);
        USB_print(txt_buffer);

      #endif
      int message[2];
      #ifdef BaseSerial
      message[0]=10123;
      message[1]=11999;

      int i=0;
      while(1){
        sprintf(txt_buffer, "*%.5d",message[0]);
        USB_print(txt_buffer);
        sprintf(txt_buffer, "%.5d",message[1]);
        USB_print(txt_buffer);

      }
      #endif


    }

}
