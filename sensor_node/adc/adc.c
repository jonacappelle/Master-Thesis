/***************************************************************************//**
 * @file adc.c
 * @brief Read battery voltage
 * @version 1.0
 * @author Jona Cappelle
 * ****************************************************************************/


/*
 * adcbatt.c
 *
 *  Created on: Nov 19, 2019
 *      Author: jonac
 */


#include <adc.h>
#include "em_adc.h"
#include "em_cmu.h"
#include "debug_dbprint.h"

/**************************************************************************//**
 * @brief
 *   Initialisation of the ADC
 *
 * @details
 *	 Reference: VDD (=2V)
 *	 Resolution 12bits
 *	 Channel 4
 *
 *****************************************************************************/
void initADC (void)
{
  // Enable ADC0 clock
  CMU_ClockEnable(cmuClock_ADC0, true);

  // Declare init structs
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  // Modify init structs and initialize
  init.prescale = ADC_PrescaleCalc(ADC_FREQ, 0); // Init to max ADC clock for Series 0
  init.timebase = ADC_TimebaseCalc(0);

  initSingle.diff       = false;        // single ended
  //initSingle.reference  = adcRef2V5;    // internal 2.5V reference
  initSingle.reference  = adcRefVDD;    // vdd reference voltage
  initSingle.resolution = adcRes12Bit;  // 12-bit resolution

  // Select ADC input. See README for corresponding EXP header pin.
  initSingle.input = adcSingleInputCh4;

  ADC_Init(ADC0, &init);
  ADC_InitSingle(ADC0, &initSingle);

  // Enable ADC Single Conversion Complete interrupt
  ADC_IntEnable(ADC0, ADC_IEN_SINGLE);

  // Enable ADC interrupts
  NVIC_ClearPendingIRQ(ADC0_IRQn);
  NVIC_EnableIRQ(ADC0_IRQn);
}

/**************************************************************************//**
 * @brief
 *   Interrupt handler for ADC
 *
 * @details
 *	 Conversion from ADC read (0-4096) to millivolts
 *
 *
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  // Clear the interrupt flag
  ADC_IntClear(ADC0, ADC_IFC_SINGLE);

  // Get ADC result
  sample = ADC_DataSingleGet(ADC0);

  // Calculate input voltage in mV, input voltage reference: 3.3V (3300)
  /* 2 V --|500k|--|1Meg|--- GND */
  millivolts = ( sample * 2000 * 15 ) / (4096 * 5 );

  // Start next ADC conversion
  /* This is for continuous measurements, not needed */
  /* When we want to read the ADC, we just call ADC_Start(ADC0, adcStartSingle) */
  //ADC_Start(ADC0, adcStartSingle);
}

/**************************************************************************//**
 * @brief
 *   Do single ADC conversion (read)
 *
 * @note
 * 	 Takes minimal 1 µs
 *
 *
 *****************************************************************************/
void ADC_Batt_Read(void)
{
  // Start first conversion
  ADC_Start(ADC0, adcStartSingle);

}

/**************************************************************************//**
 * @brief
 *   Print battery in millivolts using dbprint
 *
 *
 *****************************************************************************/
void ADC_Batt_print(void)
{
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
	dbprint("Batt:	");
	dbprintInt(millivolts);
#endif /* DEBUG_DBPRINT */
}

/**************************************************************************//**
 * @brief
 *   Get battery in percentage
 *
 * @details
 *	 20 - 40 - 60 - 80 - 100 percent
 *
 * @param[out] percent
 *   Percentage of the remaining battery capacity
 *
 *****************************************************************************/
void ADC_get_batt( uint8_t *percent )
{
	ADC_Batt_Read();

	if(millivolts <= 3670) percent[0] = 20;
	else if(millivolts <= 3720) percent[0] = 40;
	else if(millivolts <= 3800) percent[0] = 60;
	else if(millivolts <= 3950) percent[0] = 80;
	else percent[0] = 100;

}

