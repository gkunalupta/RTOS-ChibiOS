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
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
//  static const SerialConfig my_sd_config =
//  {
//     9600,    // baud rate <--> speed parameter in SerialConfig Structure
//     0,                         // Value of CR1 Register <--> CR1 register parameter in SerialConfig Structure
//     USART_CR2_STOP1_BITS,      // Value of CR2 Register <--> CR2 Register parameter in SerialConfig Structure
//     0
//  }

  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  sdStart(&SD4, NULL);
  palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(8));
  palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(8));
  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    char buffer[20];
    //const unsigned char &val;
//int v =45;
//    if (palReadPad(GPIOA, GPIOA_BUTTON)) {
//    test_execute((BaseSequentialStream *)&SD2, &rt_test_suite);
//    test_execute((BaseSequentialStream *)&SD2, &oslib_test_suite);
//    }
  sdPut(&SD2,(char)'A');
 uint8_t token = sdGet(&SD2);
 sdWrite(&SD2, (unsigned char*)"Kunal\n", 5);
   //sdPut(&SD2, (uint8_t)token);
   sdRead(&SD2, buffer, 5);


     chThdSleepMilliseconds(100);

    }
    chThdSleepMilliseconds(500);
  }

