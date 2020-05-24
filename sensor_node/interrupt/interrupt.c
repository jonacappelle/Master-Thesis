/*
 * interrupt.c
 *
 *  Created on: Nov 13, 2019
 *      Author: jonac
 */


/***************************************************************************//**
 * @file interrupt.c
 * @brief Code to detect and generate interrupts
 * @version 1.0
 * @author Jona Cappelle
 * *****************************************************************************/

#include "interrupt.h"
#include "pinout.h"
#include "debug_dbprint.h"
#include "delay.h"
//#include "ICM20948.h"

#include "em_usart.h"

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "bsp.h"

#include <stdint.h>




/**************************************************************************//**
 * @brief GPIO interrupt initialization
 *
 * @details
 * 	Interrupt functionality used by IMU
 *****************************************************************************/
void initGPIO_interrupt(void)
{
  // Configure GPIO pins
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure PB0 and PB1 as input with glitch filter enabled
  GPIO_PinModeSet(ICM_20948_INTERRUPT_PORT, ICM_20948_INTERRUPT_PIN, gpioModeInputPullFilter, 0);


  /* Clear all odd pin interrupt flags (just in case)
   * NVIC_ClearPendingIRQ(GPIO_ODD_IRQn); would also work but is less "readable" */
  GPIO_IntClear(0xAAAA);

  /* Clear all even pin interrupt flags (just in case)
   * NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn); would also work but is less "readable" */
  GPIO_IntClear(0x5555);

  // Enable IRQ for even numbered GPIO pins
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  // Enable IRQ for odd numbered GPIO pins
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  /* Enable rising-edge interrupts for ICM_20948 */
  GPIO_ExtIntConfig(ICM_20948_INTERRUPT_PORT, ICM_20948_INTERRUPT_PIN, ICM_20948_INTERRUPT_PIN, true, false, true);
}



