/***************************************************************************//**
 * @file adc.h
 * @brief Read battery voltage
 * @version 1.0
 * @author Jona Cappelle
 * ****************************************************************************/


/*
 * adcbatt.h
 *
 *  Created on: Nov 19, 2019
 *      Author: jonac
 */

#ifndef ADC_ADC_H_
#define ADC_ADC_H_

#include "stdint.h"

/*************************************/

#define	ADC_FREQ		13000000	/* ADC sample frequency */

volatile uint32_t sample;			/* Variable to store the sampled battery voltage value */
volatile uint32_t millivolts;		/* Variable to store the battery voltage */

/*************************************/

void initADC (void);
void ADC0_IRQHandler(void);
void ADC_Batt_Read(void);
void ADC_Batt_print(void);
void ADC_get_batt( uint8_t *percent );

#endif /* ADC_ADC_H_ */
