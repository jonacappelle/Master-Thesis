/***************************************************************************//**
 * @file uart.h
 * @brief UART communication with BLE
 * @version 1.0
 * @author Jona Cappelle
 * ****************************************************************************/


/*
 * uart.h
 *
 *  Created on: Mar 8, 2020
 *      Author: jonac
 */

#ifndef BLE_UART_H_
#define BLE_UART_H_

#include <stdint.h>
#include <stdbool.h>


void uart_Init();
uint8_t uartGetChar( );
void uartPutChar(uint8_t ch);
void uartPutData(uint8_t * dataPtr, uint32_t dataLen);
uint32_t uartGetData(uint8_t * dataPtr, uint32_t dataLen);
void UART1_RX_IRQHandler(void);
void UART1_TX_IRQHandler(void);






#endif /* BLE_UART_H_ */
