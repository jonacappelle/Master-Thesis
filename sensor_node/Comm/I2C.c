/*
 * I2C.c
 *
 *  Created on: Nov 18, 2019
 *      Author: jonac
 */
/***************************************************************************//**
 * @file I2C.c
 * @brief I2C function to read-write the I2C bus, based on DRAMCO code
 * @version 1.0
 * @author Jona Cappelle
 * ****************************************************************************/


/*  ____  ____      _    __  __  ____ ___
 * |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
 * | | | | |_) |  / _ \ | |\/| | |  | | | |
 * | |_| |  _ <  / ___ \| |  | | |__| |_| |
 * |____/|_| \_\/_/   \_\_|  |_|\____\___/
 *                           research group
 *                             dramco.be/
 *
 *  KU Leuven - Technology Campus Gent,
 *  Gebroeders De Smetstraat 1,
 *  B-9000 Gent, Belgium
 *
 *         File: iic.c
 *      Created: 2018-03-22
 *       Author: Geoffrey Ottoy
 *
 *  Description: I2C functionality (wrapper for EFM 32).
 *  	Used for interfacing with sensors.
 */

#include <i2cspm.h>
#include <em_i2c.h>

#include "I2C.h"

#include "pinout.h"

#define I2C_PORT_LOCATION	1

I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;


/**************************************************************************//**
 * @brief
 *   Setup I2C functionality
 *
 *****************************************************************************/
void IIC_Init(void){

	/* We use the standard I2C pins and ports, so no need to adjust */
	/*
	i2cInit.port = I2C0;
	i2cInit.sclPort = gpioPortD;
	i2cInit.sclPin = 7;
	i2cInit.sdaPort = gpioPortD;
	i2cInit.sdaPin = 6;
	i2cInit.i2cClhr = i2cClockHLRStandard;
	i2cInit.i2cRefFreq = 0 ;
	i2cInit.i2cMaxFreq = I2C_FREQ_STANDARD_MAX;
	i2cInit.portLocation = I2C_PORT_LOCATION; //zie datasheet �controller (hashtag getal in pinout)
*/
//i2cInit.i2cMaxFreq = I2C_FREQ_FAST_MAX;

	I2CSPM_Init(&i2cInit);
}

void IIC_Enable( bool enable )
{
	if (enable)
	{
		GPIO_PinModeSet(ICM_20948_SDA_PORT, ICM_20948_SDA_PIN, gpioModeWiredAndPullUp, 1);
		GPIO_PinModeSet(ICM_20948_SCL_PORT, ICM_20948_SCL_PIN, gpioModeWiredAndPullUp, 1);
	}
	else
	{
		GPIO_PinModeSet(ICM_20948_SDA_PORT, ICM_20948_SDA_PIN, gpioModeDisabled, 1);
		GPIO_PinModeSet(ICM_20948_SCL_PORT, ICM_20948_SCL_PIN, gpioModeDisabled, 1);
	}
}

/**************************************************************************//**
 * @brief
 *   Resets the I2C interface
 *
 * @note
 *   Normally not needed, just in case
 *
 * @param[in] void
 *****************************************************************************/
void IIC_Reset(void){
	I2C_Reset(i2cInit.port);
}


/**************************************************************************//**
 * @brief
 *   I2C write functionality
 *
 * @param[in] iicAddress
 *   10bit I2C address
 *
 * @param[in] wBuffer
 *	 Address where the data is that needs to be written
 *
 *	@param[in] wLength
 *	Length of the data that has to be written
 *****************************************************************************/
bool IIC_WriteBuffer(uint8_t iicAddress, uint8_t * wBuffer, uint8_t wLength){
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;
	uint8_t i2c_read_data[0];

	seq.addr  = iicAddress;
	seq.flags = I2C_FLAG_WRITE;
	/* Point to write buffer (contains command & data) */
	seq.buf[0].data   = wBuffer;
	seq.buf[0].len    = wLength;
	/* Select location/length of data to be read */
	seq.buf[1].data = i2c_read_data;
	seq.buf[1].len  = 0;

	ret = I2CSPM_Transfer(i2cInit.port, &seq);

	if (ret != i2cTransferDone) {
		return false;
	}

	return true;
}

/**************************************************************************//**
 * @brief
 *   I2C read functionality
 *
 * @param[in] iicAddress
 *   10bit I2C address
 *
 * @param[in] regCommand
 *	 Address that needs to be read
 *
 * @param[in] rBuffer
 *	 Address where the read data needs to be stored
 *
 *	@param[in] rLength
 *	Length of the data that has to be read
 *
 *****************************************************************************/
bool IIC_ReadBuffer(uint8_t iicAddress, uint8_t * rBuffer, uint8_t rLength){
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;

	seq.addr  = iicAddress;
	seq.flags = I2C_FLAG_READ;

	/* Select location/length of data to be read */
	seq.buf[0].data = rBuffer;
	seq.buf[0].len  = rLength;

	ret = I2CSPM_Transfer(i2cInit.port, &seq);

	if (ret != i2cTransferDone) {
		*rBuffer = 0;
		return false;
	}
	return true;
}


/**************************************************************************//**
 * @brief
 *   I2C read/write functionality, to read and write at the same time
 *
 * @param[in] iicAddress
 *   10bit I2C address
 *
 * @param[in] regCommand
 *	 Address that needs to be read
 *
 * @param[in] rBuffer
 *	 Address where the read data needs to be stored
 *
 *	@param[in] rLength
 *	Length of the data that has to be read
 *
 *****************************************************************************/
bool IIC_WriteReadBuffer(uint8_t iicAddress, uint8_t * wBuffer, uint8_t wLength, uint8_t *rBuffer, uint8_t rLength){
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;

	seq.addr  = iicAddress;
	seq.flags = I2C_FLAG_WRITE_READ;
	/* Point to write buffer (contains command & data) */
	seq.buf[0].data   = wBuffer;
	seq.buf[0].len    = wLength;

	/* Select location/length of data to be read */
	seq.buf[1].data = rBuffer;
	seq.buf[1].len  = rLength;

	ret = I2CSPM_Transfer(i2cInit.port, &seq);

	if (ret != i2cTransferDone) {
		*rBuffer = 0;
		return false;
	}

	return true;
}
