/***************************************************************************//**
 * @file ble.h
 * @brief Send data via BLE wirelessly
 * @version 1.0
 * @author Jona Cappelle
 * ****************************************************************************/

/*
 * ble.h
 *
 *  Created on: Nov 27, 2019
 *      Author: jonac
 */

#ifndef BLE_BLE_H_
#define BLE_BLE_H_

#include <stdint.h>
#include <stdbool.h>


#define DIY			1

///////////////////////////////////////////////////////////////////
#define BUFFER_SIZE 80
char RX_buffer[BUFFER_SIZE];

#define BLE_ERROR	99

///////////////////////////////////////////////////////////////////
#define BLE_POWER_PIN		11
#define BLE_POWER_PORT		gpioPortB
#define BLE_PIN_TX			0
#define BLE_PIN_RX			1
#define BLE_PORT			gpioPortC
#define BLE_USART			USART1

#if DIY == 0
#define BLE_LED2_PIN		10
#define BLE_LED2_PORT		gpioPortE
#endif

#if DIY == 1
#define BLE_LED2_PIN		10
#define BLE_LED2_PORT		gpioPortA
#endif

#define BLE_OUTPUT_POWER_4DB		0x04
#define BLE_OUTPUT_POWER_0DB		0x00
#define BLE_OUTPUT_POWER_N4DB		0xFC
#define BLE_OUTPUT_POWER_N8DB		0xF8
#define BLE_OUTPUT_POWER_N12DB		0xF4
#define BLE_OUTPUT_POWER_N16DB		0xF0
#define BLE_OUTPUT_POWER_N20DB		0xEC
#define BLE_OUTPUT_POWER_N40DB		0xD8


///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////





///////////////////////////////////////////////////////////////////
void BLE_power(bool enable);
void BLE_Init();
void BLE_rxtx_enable(bool enable);
void BLE_connect();
void BLE_check_connect();
void BLE_disconnect();
void BLE_sendData4(uint8_t data_in[]);
void BLE_sendData( uint8_t *data, uint8_t *batt, uint8_t length, uint8_t *ble_packet );
void BLE_readData( uint8_t *readData, uint8_t length );

void BLE_sendIMUData(uint8_t *gyroData, uint8_t *accelData, uint8_t *magnData);

void float_to_uint8_t( float *input, uint8_t *out );
void float_to_uint8_t_x3( float *input, uint8_t *out );

void BLE_set_output_power( uint8_t power );
///////////////////////////////////////////////////////////////////

#endif /* BLE_BLE_H_ */
