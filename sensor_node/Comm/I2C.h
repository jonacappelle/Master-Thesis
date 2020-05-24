/*
 * I2C.h
 *
 *  Created on: Nov 18, 2019
 *      Author: jonac
 */

/***************************************************************************//**
 * @file I2C.h
 * @brief I2C function to read-write the I2C bus, based on DRAMCO code
 * @version 1.0
 * @author Jona Cappelle
 * ****************************************************************************/


#ifndef I2C_H_
#define I2C_H_



#include <stdbool.h>

void IIC_Init(void);
void IIC_Reset(void);
bool IIC_WriteBuffer(uint8_t iicAddress, uint8_t * wBuffer, uint8_t wLength);
bool IIC_ReadBuffer(uint8_t iicAddress, uint8_t * rBuffer, uint8_t rLength);
bool IIC_WriteReadBuffer(uint8_t iicAddress, uint8_t * wBuffer, uint8_t wLength, uint8_t *rBuffer, uint8_t rLength);
void IIC_Enable( bool enable );

#endif /* AMG8833_I2C_H_ */
