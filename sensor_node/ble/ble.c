/*
 * ble.c
 *
 *  Created on: Nov 27, 2019
 *      Author: jonac
 */

/***************************************************************************//**
 * @file ble.c
 * @brief Send data via BLE wirelessly
 * @version 1.0
 * @author Jona Cappelle
 * ****************************************************************************/

#include "ble.h"

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_rtc.h"
#include "em_adc.h"
#include "em_wdog.h"
#include "rtcdriver.h"
#include "em_core.h"

#include "debug_dbprint.h"
#include "delay.h"
//#include "ICM20948.h"
#include "interrupt.h"
#include "adc.h"

#include "pinout.h"

#include <stdint.h>
#include <stdbool.h>

#include "uart.h"

bool ble_Initialized = false;

bool check = false;

uint8_t buffer[80];
uint8_t BLE_POWER_ON_RETURN[7] = { 0x02, 0x41, 0x02, 0x00, 0x01, 0x01, 0x41 };

extern bool bleConnected;


/**************************************************************************//**
 * @brief
 *   BLE chip init
 *
 * @details
 *	 Init UART
 *	 Init BLE power pin
 *
 *****************************************************************************/
void BLE_Init()
{
	uart_Init();

	/* Set pinmode for reading BLE LED 2 state */
	GPIO_PinModeSet(BLE_LED2_PORT, BLE_LED2_PIN, gpioModeInputPullFilter, 0);
}


/**************************************************************************//**
 * @brief
 *   Turn power on/off for BLE module
 *
 *
 * @note
 * 	 Extra 30 µF capacitor needed on VDD line to prevent too much voltage drop

 *
 * @param[in] enable
 *   @li 'true' - BLE on
 *   @li 'false' - BLE off
 *
 *****************************************************************************/
void BLE_power(bool enable)
{
	/* Initialize VDD pin if not already the case */
//	if (!ble_Initialized)
//	{
//		/* In the case of gpioModePushPull", the last argument directly sets the pin state */
//		GPIO_PinModeSet(BLE_POWER_PORT, BLE_POWER_PIN, gpioModePushPull, enable);

		/* Check if the right sequence is returned */
//		BLE_readData( buffer, BUFFER_SIZE );
//
//		check = Check_Equal(buffer, BLE_POWER_ON_RETURN, 7);
//		if ( check != true )
//		{
//			return BLE_ERROR;
//		}

//		ble_Initialized = true;
//	}
//	else
//	{
		if (enable) GPIO_PinModeSet(BLE_POWER_PORT, BLE_POWER_PIN, gpioModePushPull, 1); /* Enable VDD pin */
		else {
			GPIO_PinModeSet(BLE_POWER_PORT, BLE_POWER_PIN, gpioModeDisabled, 0); /* Disable VDD pin */
		}
//	}
//	return 0;
}


/**************************************************************************//**
 * @brief
 *   Enable disable RX/TX pins from BLE module
 *
 *
 * @note
 * 	 Disable when going to sleep, can otherwise draw some unwanted power
 *
 *
 * @param[in] enable
 *   @li 'true' - enable
 *   @li 'false' - disable
 *
 *****************************************************************************/
void BLE_rxtx_enable(bool enable)
{
	if(enable)
	{
		  GPIO_PinModeSet(BLE_PORT, BLE_PIN_RX, gpioModeInput, 0);
		  GPIO_PinModeSet(BLE_PORT, BLE_PIN_TX, gpioModePushPull, 1);
	}else{
		/* In ACTION_SLEEP mode the UART is disabled, so the module will not receive or transmit any
data. To prevent leakage current, the host shall not pull the UART_RX to LOW level (as the
module has an internal pull-up resistor enabled on this pin).*/
		GPIO_PinModeSet(BLE_PORT, BLE_PIN_RX, gpioModeDisabled, 0);
		GPIO_PinModeSet(BLE_PORT, BLE_PIN_TX, gpioModeDisabled, 0);
	}
}

/**************************************************************************//**
 * @brief
 *   Connect sequence BLE
 *
 * @details
 *	 The only function that's not send interrupt based (can be a future improvement)
 *
 *
 *****************************************************************************/
void BLE_connect()
{
//	while(!bleConnected)
//	{
		/* Check if BLE is connected to device */
//		BLE_check_connect();

		//Connect A -> B
		const char connect[] = { 0x02, 0x06, 0x06, 0x00, 0xBB, 0x04, 0x20, 0xDA, 0x18, 0x00, 0x5F };
		for (int i=0 ; i < sizeof(connect) ; i++ )
		{
		  USART_Tx(USART1, connect[i]);
		}
//	}

}


/**************************************************************************//**
 * @brief
 *   Check if LED pin on BLE is high to check if the module is connected to a receiver
 *
 * @note
 * 	 There is some delay on the actual connection and the pin turning high
 *
 *
 *****************************************************************************/
void BLE_check_connect()
{
	bleConnected = GPIO_PinInGet(BLE_LED2_PORT, BLE_LED2_PIN);
}


/**************************************************************************//**
 * @brief
 *   Disconnect BLE from receiver
 *
 *
 * @note
 * 	 takes approx 1sec
 *
 *
 *****************************************************************************/
void BLE_disconnect()
{
	  //Disconnect A -> B
	  const char disconnect[] = { 0x02, 0x07, 0x00, 0x00, 0x05 };
	  for (int i=0 ; i < sizeof(disconnect) ; i++ )
	  {
		  USART_Tx(USART1, disconnect[i]);
	  }
}


/**************************************************************************//**
 * @brief
 *   Test function, not used
 *
 *
 *****************************************************************************/
void BLE_sendData4(uint8_t data_in[])
{
	int d=0;

	//Preface
	/* Start signal - command - length - payload - checksum */
	uint8_t data[9] = {0x02, 0x04, 0x04, 0x00 };

	//concatenate
	while (d < (4))
	  {
		  data[4+d] = data_in[d];
		  d++;
	  }


	//calculate Checksum CS
	uint8_t checksum = 0x00;
	for (int j=0;j<(sizeof(data)/sizeof(uint8_t));j++)
	{
		checksum = checksum^data[j];
	}

	data[8] = checksum;


	//Send data out
	for (int i = 0; i < 9; i++)
	{
		  USART_Tx(USART1, data[i]);
	}
}


/**************************************************************************//**
 * @brief
 *   Send all the data over BLE
 *
 *
 * @param[in] data
 *   Euler angles
 * @param[in] length
 *   packet length
 * @param[in] batt
 *    battery in percentage
 * @param[out] ble_data
 *   return ble_data for debugging purposes
 *
 *****************************************************************************/
void BLE_sendData( uint8_t *data, uint8_t *batt, uint8_t length, uint8_t *ble_data ) // max packet length: 255
{
	int ble_packet_length = length + 5;
	// uint8_t ble_data[ble_packet_length];
	int d = 0;

	ble_data[0] = 0x02;
	ble_data[1] = 0x04;
	ble_data[2] = length;
	ble_data[3] = 0x00;

	while ( d < length)
	{
		ble_data[4+d] = data[d];
		d++;
	}

	ble_data[ble_packet_length-2] = batt[0];

	//calculate Checksum CS
	uint8_t checksum = 0x00;
	for (int j=0;j<(ble_packet_length-1);j++)
	{
		checksum = checksum^ble_data[j];
	}

	ble_data[ble_packet_length-1] = checksum;

	//Send data out
//	for (int i = 0; i < ble_packet_length; i++)
//	{
//		  USART_Tx(USART1, ble_data[i]);
//	}

	/* Send data interrupt driven */

	uartPutData( ble_data, ble_packet_length );

}


/**************************************************************************//**
 * @brief
 *   Test function, not used
 *
 *
 *****************************************************************************/
void BLE_readData( uint8_t *readData, uint8_t length )
{
//	  while (1)
//	  {
//	    // Read a line from the UART
//	    for (int i = 0; i < BUFFER_SIZE - 1 ; i++ )
//	    {
//	      RX_buffer[i] = USART_Rx(USART1);
//	      if (RX_buffer[i] == '\r')
//	      {
//	        break; // Breaking on CR prevents it from being counted in the number of bytes
//	      }
//	    }
//	  }

	uartGetData( readData, length );

}


/**************************************************************************//**
 * @brief
 *   Test function, not used
 *
 *
 *****************************************************************************/
void BLE_sendIMUData(uint8_t *gyroData, uint8_t *accelData, uint8_t *magnData)
{
	int d=0;

	//Preface
	/* Start signal - command - length - payload - checksum */
	char data[23] = { 0x02, 0x04, 0x06, 0x00 };

	//concatenate
	while (d < 6)
	  {
		  data[4+d] = gyroData[d];
		  d++;
	  }
	d=0;
	while (d < 6)
	{
		data[10+d] = accelData[d];
		d++;
	}
	d=0;
	while (d < 6)
	{
		data[16+d] = magnData[d];
		d++;
	}

	//calculate Checksum CS
	char checksum = 0x00;
	for (int j=0;j<(sizeof(data)/sizeof(char));j++)
	{
		checksum = checksum^data[j];
	}

	data[22] = checksum;


	//Send data out
	for (int i = 0; i < 23; i++) //i runs until length of char data[]
	{
		  USART_Tx(USART1, data[i]);
	}
}


/**************************************************************************//**
 * @brief
 *   Float to uint8_t conversion
 *
 *
 * @param[in] input
 *   float
 *
 * @param[out] out
 *   4 x uint8_t's
 *
 *****************************************************************************/
void float_to_uint8_t( float *input, uint8_t *out )
{
	union{
		float in;
		uint8_t bytes[sizeof(float)];
	}thing;

	thing.in = input[0];

	for( int i=0; i<sizeof(float); i++)
	{
		out[i] = thing.bytes[i];
	}
}


/**************************************************************************//**
 * @brief
 *   Float to uint8_t conversion x3
 *
 *
 * @param[in] input
 *   float
 *
 * @param[out] out
 *   4 x uint8_t's
 *
 *****************************************************************************/
void float_to_uint8_t_x3( float *in, uint8_t *out ) // Converts euler angles to uint8_t to be transmitted on ble
{

	float_to_uint8_t( &in[0], &out[0] );
	float_to_uint8_t( &in[1], &out[4] );
	float_to_uint8_t( &in[2], &out[8] );

}


/**************************************************************************//**
 * @brief
 *   Set BLE output power
 *
 * @details
 *	 Momentarily not used, since the reset value of the BLE is the one used
 *
 *
 * @param[in] power
 *   power value (see datasheet for options)
 *
 *****************************************************************************/
void BLE_set_output_power( uint8_t power )
{
		int len = 7;
		uint8_t data[7];

		data[0] = 0x02;
		data[1] = 0x11;
		data[2] = 0x02;
		data[3] = 0x00;
		data[4] = 0x11;
		data[5] = power;

		//calculate Checksum CS
		uint8_t checksum = 0x00;
		for (int j=0;j<(7-1);j++)
		{
			checksum = checksum^data[j];
		}

		data[len-1] = checksum;

		uartPutData( data, len );
}

