/***************************************************************************//**
 * @file timer.c
 * @brief millis and micros functionality
 * @version 1.0
 * @author Jona Cappelle
 * ****************************************************************************/

/*
 * timer.c
 *
 *  Created on: Feb 25, 2020
 *      Author: jonac
 */

#include <timer.h>
#include "em_rtc.h"
#include "rtcdriver.h"
#include "rtcdrv_config.h" // voor millis(); config file

/* Needed to use uintx_t */
#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"

/**************************************************************************//**
 * @brief
 *   Millis Arduino like functionality for EFM32HG
 *
 * @return
 *	millis
 *
 *****************************************************************************/
uint32_t millis(void)
{
	return RTCDRV_TicksToMsec(RTCDRV_GetWallClockTicks64());
}


/**************************************************************************//**
 * @brief
 *   Micros Arduino like functionality for EFM32HG
 *
 * @return
 *	micros
 *
 *****************************************************************************/
uint32_t micros(void)
{
	return 1000 * RTCDRV_TicksToMsec(RTCDRV_GetWallClockTicks64());
}
