/***************************************************************************//**
 * @file util.c
 * @brief Handy functions
 * @version 1.0
 * @author Jona Cappelle
 * *****************************************************************************/


/*
 * util.c
 *
 *  Created on: Mar 11, 2020
 *      Author: jonac
 */

#include <util.h>
#include "stdbool.h"
#include "stdint.h"
#include "em_gpio.h"
#include "pinout.h"


/**************************************************************************//**
 * @brief
 *   Check if two arrays are equal
 *
 *
 * @param[in] arr1
 *   first array
 * @param[in] arr2
 *   second array
 * @param[in] length
 *   length array
 *
 *****************************************************************************/
bool Check_Equal(uint8_t *arr1, uint8_t *arr2, uint8_t length)
{
	for (uint8_t i = 0; i < length; i++)
	{
		if (arr1[i] != arr2[i])
		{
			return false;
		}
	}
	return true;
}

/**************************************************************************//**
 * @brief
 *   Function to blink LED on sensor node
 *
 *
 * @param[in] times
 *   amount of times to blink the LED
 *
 *****************************************************************************/
void blink(uint8_t times)
{
#if DIY == 1
	GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);
	for(int i = 0; i < times; i++){
		GPIO_PinOutSet(LED_PORT, LED_PIN);
		delay(10);
		GPIO_PinOutClear(LED_PORT, LED_PIN);
		delay(100);
		GPIO_PinOutSet(LED_PORT, LED_PIN);
		delay(10);
		GPIO_PinOutClear(LED_PORT, LED_PIN);
		delay(1000);
	}
	GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModeDisabled, 0);
#endif
}
