/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"
#include "stdio.h"
#include "string.h"


/* Registers Address Declaration*/
#define DeviceAddress        0x0D
#define CtrlReg1             0x09
#define CtrlReg2             0x0A
#define SetResetReg          0x0B

static i2cflags_t errors = 0;


static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  400000,
  FAST_DUTY_CYCLE_2,
};


uint8_t data[2]={0x01,0x1D};
uint8_t rxData[6];
uint16_t X_Axis, Y_Axis, Z_Axis;
uint8_t StatusReg =0x06;

uint8_t setReset[2]={SetResetReg, 0x01};
uint8_t cr1[2]={CtrlReg1, 0x1D};
float compassValue, compassHeading;


/*
 * Serial Config for Terminal
 *
 */
static const SerialConfig serial_terminal = {
  115200,
  0,
  USART_CR2_STOP1_BITS,
  0
};


static void print(char *p) {

  while (*p) {
    sdPut(&SD2, *p++);
  }
}

/**
 *
 */
static void println(char *p) {

  while (*p) {
    sdPut(&SD2, *p++);
  }
  sdWriteTimeout(&SD2, (uint8_t *)"\r\n", 2, TIME_INFINITE);
}

/**
 *
 */
static void printn(int16_t n) {
  char buf[16], *p;

  if (n > 0)
    sdPut(&SD2, '+');
  else{
    sdPut(&SD2, '-');
    n = abs(n);
  }

  if (!n)
    sdPut(&SD2, '0');
  else {
    p = buf;
    while (n)
      *p++ = (n % 10) + '0', n /= 10;
    while (p > buf)
      sdPut(&SD2, *--p);
  }
}

/*
 * Application entry point.
 */


#define gb_pcf8574_WADDR 0b01111110  //Slave Write
#define gb_pcf8574_RADDR 0b01111111   //Slave Read
#define gb_pcf8574_ADDR  0b0111111
#define gb_LCD_BACKLIGHT         0x08
#define gb_LCD_NOBACKLIGHT       0x00
#define gb_display_rate 50


int main(void)
{
  msg_t status = MSG_OK;
  //char buffer[20];
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */

  halInit();
  chSysInit();


      /*
      * Starts I2C
      */
        i2cStart(&I2CD3, &i2ccfg);


      /*
       * Activates the serial driver 2 using the driver default configuration.
       * PA2(TX) and PA3(RX) are routed to USART2.
       */
      sdStart(&SD2, &serial_terminal);
      palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
      palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

      sdWrite(&SD2, (unsigned char*)"Write a String\n", 15);

        /*
         * Starting the transmitter and receiver threads.
         */
      palSetPadMode(GPIOA, GPIOA_PIN8, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST);    /* I2C_SCK PA8 */
      palSetPadMode(GPIOC,GPIOC_PIN9,  PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST);    /* I2C_SDA PC9 */
      /* Prepare the Magnetometer*/
     // i2cAcquireBus(&I2CD3);
       status = i2cMasterTransmitTimeout(&I2CD3,DeviceAddress,setReset,2,NULL,0,10); //Write Register 0BH by 0x01 (Define Set/Reset period)
      //status = i2cMasterTransmit(&I2CD3,DeviceAddress,setReset,2,rxData,0);
     // status = i2cMasterTransmitTimeout(&I2CD3,DeviceAddress,cr1,2,NULL,0,10);     // Write Register 09H by 0x1D (Define OSR = 512, Full Scale Range = 8 Gauss, ODR = 200Hz, set continuous measurement mode
    //  i2cReleaseBus(&I2CD3);
      //status = i2cMasterTransmit(&I2CD3,DeviceAddress,cr1,2,rxData,0);
      print("status1 & 2:");
      printn(status);
      println("");
      if (status != MSG_OK)
      {
            errors = i2cGetErrors(&I2CD3);
            print("Errors: ");
            printn(errors);
            println("");
       }

      chThdSleepMilliseconds(100);

       /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
          while (true) {
            //    i2cAcquireBus(&I2CD3);
                status = i2cMasterTransmitTimeout(&I2CD3, DeviceAddress, &StatusReg,1,rxData,1,10); // Check status register 06H[0]
               // i2cReleaseBus(&I2CD3);
                if (status != MSG_OK)
                {
                   errors = i2cGetErrors(&I2CD3);
                   print("Errors1: ");
                   printn(errors);
                   println("");
                }
                else
                {
                  print("all is clear!");
                  println("");

                }


                if((rxData[0]&0x01) == 1){  // checking Drdy bit if '1' means ready to read.
                      palTogglePad(GPIOD, GPIOD_LED4);
                      chThdSleepMilliseconds(500);
                    //  i2cAcquireBus(&I2CD3);
                      status = i2cMasterTransmitTimeout(&I2CD3, DeviceAddress, 0x00,1,rxData,6,10); //Reading Mag X,Y,Z registers .
                      //status = i2cMasterTransmit(&I2CD3, DeviceAddress, 0x00,1,rxData,6);
                    //  i2cReleaseBus(&I2CD3);
                      if (status != MSG_OK){
                            errors = i2cGetErrors(&I2CD3);
                            print("Errors2: ");
                            printn(errors);
                            println("");
                       }
                      X_Axis=rxData[1]<<8 |rxData[0];
                      Y_Axis=rxData[3]<<8 |rxData[2];
                      Z_Axis=rxData[5]<<8 |rxData[4];
                      print("X: ");
                      printn(X_Axis);
                      println("");
                      print(" Y: ");
                      printn(Y_Axis);
                      println("");
                      print(" Z: ");
                      printn(Z_Axis);
                      println("");
                      chThdSleepMilliseconds(1000);
                 }

  // char buffer[20];

 //  sdWrite(&SD2, (unsigned char*)"Write a String\n", 15);
 //sdRead(&SD2, buffer, 6);

    //chThdSleepMilliseconds(100);

    }
   // chThdSleepMilliseconds(500);
  }

