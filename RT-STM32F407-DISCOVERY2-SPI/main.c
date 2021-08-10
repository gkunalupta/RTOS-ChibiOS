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

//#include "usbcfg.h"
//#include "chprintf.h"
//#define cls(chp)  chprintf(chp, "\033[2J\033[1;1H")



#define WriteEnable 0x06
#define WriteDisable 0x04
#define Dummybyte 0xA5

#define ReadSR1 0x05
#define WriteSR1 0x01
#define ReadSR2 0x35  //0x35: 00110101
#define WriteSR2 0x31
#define ReadSR3 0x15
#define WriteSR3 0x11


#define Write_Enab_for_Vol_status_regist 0x50

#define ReadData 0x03
#define WriteData 0x02
#define ReadDataFast 0x0B


#define JEDECID 0x9F
#define UinqueID 0x4B

#define SectErase4KB 0x20
#define SectErase32KB 0x52
#define SectErase64KB 0xD8
#define chiperase 0xC7

#define reset1 0x66
#define reset2 0x99

#define read_addr1 0x020000
#define read_addr2 0x030000
#define read_addr3 0x040000

#define cs_set() palSetPad(GPIOB, GPIOB_PIN12)
#define cs_reset() palClearPad(GPIOB, GPIOB_PIN12)

uint8_t rx_buf[1025];
uint8_t tx_buf[10];


void SPI2_Send (uint8_t *dt, uint16_t cnt)
{
  spiSend(&SPID2, cnt, dt);
}

void SPI2_Recv (uint8_t *dt, uint16_t cnt)
{
  spiReceive(&SPID2, cnt, dt);
}

void W25_Reset (void)
{
  cs_reset();
  tx_buf[0] = reset1;
  tx_buf[1] = reset2;
  SPI2_Send(tx_buf, 2);
  cs_set();
}


void WriteEnable_flash(void)
{
    cs_reset();
    tx_buf[0] = WriteEnable;
    SPI2_Send(tx_buf,1);
    cs_set();
}

void W25_Read_Data(uint32_t addr, uint8_t* data, uint32_t sz)
{
  cs_reset();
  tx_buf[0] = ReadData;
  tx_buf[1] = (addr >> 16) & 0xFF;
  tx_buf[2] = (addr >> 8) & 0xFF;
  tx_buf[3] = addr & 0xFF;
  SPI2_Send(tx_buf, 4);
  SPI2_Recv(data, sz);
  cs_set();
}

void W25_Write_Data(uint32_t addr, uint8_t* data, uint32_t sz)
{
  WriteEnable_flash();
  cs_reset();
  tx_buf[0] = WriteData;
  tx_buf[1] = (addr >> 16) & 0xFF;
  tx_buf[2] = (addr >> 8) & 0xFF;
  tx_buf[3] = addr & 0xFF;
  SPI2_Send(tx_buf, 4);
  SPI2_Send(data, sz);
  cs_set();
}
uint32_t W25_Read_ID(void)
{
  uint8_t dt[4];
  tx_buf[0] = JEDECID;
  cs_reset();
  SPI2_Send(tx_buf, 1);
  SPI2_Recv(dt,3);
  cs_set();
  return (dt[0] << 16) | (dt[1] << 8) | dt[2];
}
void W25_Ini(void)
{
  chThdSleepMilliseconds(100);
  W25_Reset();
  chThdSleepMilliseconds(100);
 // unsigned int id = W25_Read_ID();


}
void erase_sector4KB(uint32_t addr)
{
    WriteEnable_flash();
    cs_reset();
    tx_buf[0] = SectErase4KB;
    tx_buf[1] = (addr >> 16) & 0xFF;
    tx_buf[2] = (addr >> 8) & 0xFF;
    tx_buf[3] = addr & 0xFF;
    SPI2_Send(tx_buf,4);
    cs_set();
}
void erase_sector32KB(uint32_t addr)
{
    WriteEnable_flash();
    cs_reset();
    tx_buf[0] = SectErase32KB;
    tx_buf[1] = (addr >> 16) & 0xFF;
    tx_buf[2] = (addr >> 8) & 0xFF;
    tx_buf[3] = addr & 0xFF;
    SPI2_Send(tx_buf,4);
    cs_set();
}
void erase_sector64KB(uint32_t addr)
{
    WriteEnable_flash();
    cs_reset();
    tx_buf[0] = SectErase64KB;
    tx_buf[1] = (addr >> 16) & 0xFF;
    tx_buf[2] = (addr >> 8) & 0xFF;
    tx_buf[3] = addr & 0xFF;
    SPI2_Send(tx_buf,4);
    cs_set();
}
void chip_erase(void)
{
    WriteEnable_flash();
    cs_reset();
    tx_buf[0] = chiperase;
    SPI2_Send(tx_buf,1);
    cs_set();
}

void Uinque_ID(uint8_t uinque[])
{
    cs_reset();
    tx_buf[0] = UinqueID;

}

void WriteSR(uint8_t SR_address, uint8_t SR_data)
{
    WriteEnable_flash();
    cs_reset();
    tx_buf[0] = SR_address;
    tx_buf[1] = SR_data;
    SPI2_Send(tx_buf,2);
    cs_set();

}
uint8_t ReadSR(uint8_t SR_address)
{
    uint8_t RSR[1] = {0};
    cs_reset();
    tx_buf[0] =  SR_address;
    SPI2_Send(tx_buf,1);
    SPI2_Recv(RSR,1);
    cs_set();
    return RSR[0];
}



#define SPI_LOOPBACK

/*
 * Maximum speed SPI configuration (27MHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig hs_spicfg = {
  FALSE,
  NULL,
  GPIOB,         /*cs port */
  GPIOB_PIN12,   /* cs pin1*/
  SPI_CR1_BR_2 |SPI_CR1_SSM |SPI_CR1_SSI,  /*CONFIGURING CR1 REGISTER*/
  0
};


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

/*
 * SPI TX and RX buffers.
 * Note, the buffer are aligned to a 32 bytes boundary because limitations
 * imposed by the data cache. Note, this is GNU specific, it must be
 * handled differently for other compilers.
 */

//uint8_t senddata[25] = "Kunal gupta gettobyte";
uint8_t receivedata[25] = {0};
uint8_t* tran_buff = "The STM32 H7-series is a group of high performance STM32 microcontrollers based on the ARM Cortex-M7F core with double-precision floating point unit and optional second Cortex-M4F core with single-precision floating point. Cortex-M7F core can reach working frequency up to 480 MHz, while Cortex-M4F ";
uint8_t x ,y,z;
//static BaseSequentialStream* chp = (BaseSequentialStream*)&SDU1;
static THD_WORKING_AREA(spi_thread_1_wa, 1000);
static THD_FUNCTION(spi_thread_1, p) {

  (void)p;
  chRegSetThreadName("SPI thread 1");
  while (true) {

    sdWrite(&SD2, (unsigned char*)"wq275", 5);
    /* Bush acquisition and SPI reprogramming.*/
    spiAcquireBus(&SPID2);
    spiStart(&SPID2, &hs_spicfg);

  //  W25_Read_ID();
    x = ReadSR(ReadSR1);

    y = ReadSR(ReadSR2);
    z = ReadSR(ReadSR3);

    sdPut(&SD2,(int)x);
    sdPut(&SD2,(int)y);
    sdPut(&SD2,(int)z);
    chThdSleepMilliseconds(100);

    erase_sector4KB(read_addr2);
    chThdSleepMilliseconds(100);
    W25_Write_Data(read_addr2,tran_buff,299);
    chThdSleepMilliseconds(100);
    W25_Read_Data(read_addr2,rx_buf,299);
    chThdSleepMilliseconds(100);
    sdWrite(&SD2, (unsigned char*)rx_buf, 35);
    //chprintf(chp,"Received Data = %s \r\n",rx_buf);
    chThdSleepMilliseconds(1000);
    /* Releasing the bus.*/
    spiReleaseBus(&SPID2);
  }
}
/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palSetPad(GPIOD, GPIOD_LED3);       /* Orange.  */
    chThdSleepMilliseconds(100);
    palClearPad(GPIOD, GPIOD_LED3);     /* Orange.  */
    chThdSleepMilliseconds(100);
  }
}
/*
 * Application entry point.
 */
int main(void) {

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

  /* Initializes a serial-over-USB CDC driver.*/
   //   sduObjectInit(&SDU1);
   ///   sduStart(&SDU1, &serusbcfg);

      /*
       * Activates the USB driver and then the USB bus pull-up on D+.
       * Note, a delay is inserted in order to not have to disconnect the cable
       * after a reset.
       */
  //    usbDisconnectBus(serusbcfg.usbp);
   //   chThdSleepMilliseconds(1500);
    //  usbStart(serusbcfg.usbp, &usbcfg);
    //  usbConnectBus(serusbcfg.usbp);

      /*
       * Activates the serial driver 2 using the driver default configuration.
       * PA2(TX) and PA3(RX) are routed to USART2.
       */
      sdStart(&SD2, &serial_terminal);
      palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
      palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

      sdWrite(&SD2, (unsigned char*)"Write a String\n", 15);
      /*
         * SPI2 I/O pins setup.
         */
      palSetPadMode(GPIOB, GPIOB_PIN10,
                    PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);    /* New SCK */
      palSetPadMode(GPIOC,GPIOC_PIN2,
                    PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);    /* New MISO*/
      palSetPadMode(GPIOC, GPIOC_PIN3,
                    PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);    /* New MOSI*/
      palSetPadMode(GPIOB,GPIOB_PIN12,
                    PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); /* New CS*/

        /*
           * Starting the transmitter and receiver threads.
           */
          chThdCreateStatic(spi_thread_1_wa, sizeof(spi_thread_1_wa),
                            NORMALPRIO + 1, spi_thread_1, NULL);

  /*
   * Creates the example thread.
   */
 // chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true)
  {
  // char buffer[20];

 //  sdWrite(&SD2, (unsigned char*)"Write a String\n", 15);
 //sdRead(&SD2, buffer, 6);

    // chThdSleepMilliseconds(100);

    }
   // chThdSleepMilliseconds(500);
  }

