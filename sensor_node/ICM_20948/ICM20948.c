/*
 * ICM20948.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: jonac
 */


/***************************************************************************//**
 * @file ICM20948.c
 * @brief Advanced funcions to control and read data from the ICM-20948
 * @details Partly copied from silabs thunderboard code for ICM-20689, partly self written
 * @version 1.0
 * @author Jona Cappelle
 * *****************************************************************************/

#include "ICM20948.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "debug_dbprint.h"
#include "delay.h"
#include <stdint.h>
#include "pinout.h"

#include "timer.h"				/* Home brew millis() & micros() Arduino like functionality */

#include "I2C.h"				/* DRAMCO inspired I2C read - write - readwrite library */
#include "em_i2c.h"
#include "i2cspm.h"				/* I2C higher level library to easily setup and use I2C */
#include "em_core.h"
#include "em_wdog.h"
#include "rtcdriver.h"
#include "em_rtc.h"

#include "bsp.h"

bool ICM_20948_Initialized = false;				/**< Variable to keep track if the IMU is initialized */
#define M_PI		3.14159265358979323846		/**< Declaration of PI as a constant, used for conversion angle <--> rad */
int32_t mag_minimumRange = 40;					/**< Minimum value to look for on each axis of the magnetometer, to make sure the magnetomeres is being rotated */

////////////////////////////////////////

float mag_res = 4912.0f / 32752.0f;				/**<  Factor to calulate magn in microTesla from raw values of registers */
int16_t _hxcounts,_hycounts,_hzcounts;			/**< Magnetometer x, y and z axis values (not calibrated)*/



// transformation matrix
/* transform the magnetometer values to match the coordinate system of the IMU */
int16_t tX[3] = {1,  0,  0};		/**< Transformation matrix to put magnetometer in same reference frame as gyro and accel */
int16_t tY[3] = {0, -1,  0};		/**< Transformation matrix to put magnetometer in same reference frame as gyro and accel */
int16_t tZ[3] = {0,  0, -1};		/**< Transformation matrix to put magnetometer in same reference frame as gyro and accel */

float _hx, _hy, _hz;				/**< Not used, int16_t _hxcounts,_hycounts,_hzcounts are used to store magnetometer values */

float accRawScaling = 32767.5f; 	/**< =(2^16-1)/2 16 bit representation of acc value to cover +/- range */
float gyroRawScaling = 32767.5f; 	/**< =(2^16-1)/2 16 bit representation of gyro value to cover +/- range */
float magRawScaling = 32767.5f; 	/**< =(2^16-1)/2 16 bit representation of gyro value to cover +/- range */

float _magScale = (4912.0f) / (32767.5f);	/**<  Factor to calulate magn in microTesla from raw values of registers, same as magscale */

// static to keep callibration values in memory
static float _hxb = 0, _hyb = 0, _hzb = 0;			/**< Offsets of the magnetometer, DEFAULT=0 */
static float _hxs = 1.0f, _hys = 1.0f,_hzs = 1.0f;	/**< Scale factors of the magnetometer, DEFAULT=1 */


extern bool IMU_MEASURING;							/**<  Variable to check if IMU is measuring */
////////////////////////

/***************************************************************************//**
 * @brief
 *    Init function for ICM20948
 * @details
 * 		I2C init
 * 		Power on IMU
 * 		Reset IMU
 * 		Init 2
 * 		Calibrate Gyro + Accel
 * 		Calibrate Magn
 *
 ******************************************************************************/
void ICM_20948_Init ()
{
	/* Enable GPIO clock in order to set pin states */
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	delay(10);

	/* Setup SPI / IIC interface for ICM_20948 */
	/* Comment / Uncomment to select */
	//ICM_20948_Init_SPI();
	IIC_Init();
	delay(10);

	/* Power the ICM_20948 */
	ICM_20948_power(true);
	delay(200);

	/* Reset ICM_20948, just to be sure */
	ICM_20948_reset();
	delay(100); // 100ms delay needed for reset sequence


	ICM_20948_Init2();


#if DIY == 0
    BSP_LedSet(0);
#endif

    /* CALIBRATION */
/////////////////////////////////////////////////////////////////
 	 float accelCal[3];
 	 float gyroCal[3];
 	 float magOffset[3];
 	 float magScale[3];
 	 ICM_20948_accelGyroCalibrate( accelCal, gyroCal );
/////////////////////////////////////////////////////////////////

#if DIY == 0
 	 BSP_LedClear(0);
#endif

#if DIY == 0
	BSP_LedSet(1);
#endif

	ICM_20948_Init2();

	////////////////////////////////////
#if DIY == 1
	GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);
	GPIO_PinOutSet(LED_PORT, LED_PIN);
#endif
	uint8_t temp[1];
    ICM_20948_set_mag_mode(AK09916_MODE_100HZ);
    delay(100);
	ICM_20948_read_mag_register(0x31, 1, temp);
	ICM_20948_read_mag_register(0x11, 8, temp);

	ICM_20948_calibrate_mag( magOffset, magScale );

#if DIY == 1
	GPIO_PinOutClear(LED_PORT, LED_PIN);
	GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModeDisabled, 0);
#endif

////////////////////////////////////
#if DIY == 0
	BSP_LedClear(1);
#endif




	ICM_20948_Initialized = true;
}

/***************************************************************************//**
 * @brief
 *    Second init function for ICM20948, use when waking from sleep
 *@details
 *	Set sample rates, bandwidths, check whoAmI, enable sensors
 *
 ******************************************************************************/
void ICM_20948_Init2()
{


	    /* Auto select best available clock source PLL if ready, else use internal oscillator */
	    ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_1, ICM_20948_BIT_CLK_PLL);
	    /* PLL startup time - no spec in data sheet */
	    delay(30);

	    /* Reset I2C Slave module and use SPI */
	    /* Enable I2C Master I/F module */    				/* ICM_20948_BIT_I2C_IF_DIS om te vermijden dat toch I2C zal gebruikt worden */
	    //ICM_20948_registerWrite(ICM_20948_REG_USER_CTRL, ICM_20948_BIT_I2C_MST_EN);//ICM_20948_BIT_I2C_IF_DIS | ICM_20948_BIT_I2C_MST_EN);

	    /* Set I2C Master clock frequency */
	    //ICM_20948_registerWrite(ICM_20948_REG_I2C_MST_CTRL, ICM_20948_I2C_MST_CTRL_CLK_400KHZ);

	    uint8_t temp[8];

	    /* Read ICM20948 "Who am I" register */
	    ICM_20948_registerRead(ICM_20948_REG_WHO_AM_I, 1, temp);

	    /* Check if "Who am I" register was successfully read */
	    if (temp[0] == ICM20948_DEVICE_ID) {
	#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
	    	dbprint("ICM20948 WHOAMI OKE");
	#endif /* DEBUG_DBPRINT */

	    }



	    /* Make sure ICM_20948 is not in sleep mode */
		ICM_20948_sleepModeEnable( false );

		/* Enable all ICM_20948 sensors */
		ICM_20948_sensorEnable(true, true, true);
		delay(10);

		/* Set ICM_20948 gyro and accel sample rate to 75Hz */
		ICM_20948_sampleRateSet(50);

		/* Set full scale range: gyro & accel */
		ICM_20948_gyroFullscaleSet(ICM_20948_GYRO_FULLSCALE_2000DPS);
		ICM_20948_accelFullscaleSet(ICM_20948_ACCEL_FULLSCALE_4G);

		/* Setup 50us interrupt */
		ICM_20948_latchEnable(true);

	    // TODO: Bandwidth gyro + accel
		// Standard low pass filters are enabled
//		ICM_20948_accelBandwidthSet( ICM_20948_ACCEL_BW_1210HZ );
//		ICM_20948_gyroBandwidthSet( ICM_20948_GYRO_BW_120HZ );

	    /* Auto select best available clock source PLL if ready, else use internal oscillator */
	    ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_1, ICM_20948_BIT_CLK_PLL);
	    delay(30);


		/* Magnetometer */
		/* IIC passtrough: magnetometer can be accessed on IIC bus */
		ICM_20948_registerWrite(ICM_20948_REG_INT_PIN_CFG, 0x02);
		delay(10);

		/* Reset magnetometer */
		ICM_20948_write_mag_register(0x32, 0x01);
		delay(100); // reset takes 100ms

	    /* Read AK09916 "Who am I" register */
	    ICM_20948_read_mag_register(AK09916_REG_WHO_AM_I, 1, temp);

	    /* Check if AK09916 "Who am I" register was successfully read */
	    if (temp[0] == AK09916_DEVICE_ID) {
	#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
	    	dbprint("AK09916 WHOAMI OKE");
	#endif /* DEBUG_DBPRINT */

	    }

	    /* Configure magnetometer */
	    ICM_20948_set_mag_mode(AK09916_MODE_50HZ);
	    delay(10);

		ICM_20948_read_mag_register(0x31, 1, temp);
		/* Update data by reading through till ST2 register */
		ICM_20948_read_mag_register(0x11, 8, temp);





	//	/* Set interrupt to trigger every 100ms when the data is ready */
	//	IMU_MEASURING = true;
	//	ICM_20948_interruptEnable(true, false);

}

/**************************************************************************//**
 * @brief
 *   Initiate SPI functionality for ICM20948
 *
 * @note
 * 	 SPI not used
 *
 *****************************************************************************/
void ICM_20948_Init_SPI ()
{

	  /*****************************/

	  /* Enable necessary clocks */
	  	CMU_ClockEnable(cmuClock_HFPER, true); /* GPIO and USART0/1 are High Frequency Peripherals */
	  	CMU_ClockEnable(cmuClock_GPIO, true);
	  	CMU_ClockEnable(cmuClock_USART0, true);

	  	/* Configure GPIO */
	  	/* In the case of gpioModePushPull", the last argument directly sets the pin state */
	    GPIO_PinModeSet(ICM_20948_CLK_PORT, ICM_20948_CLK_PIN, gpioModePushPull, 0); // US0_CLK is push pull
	    /* Normally this is gpioPortE: pin 13 */
	    /* Apparently, can't use the US0_CS port (PE13) to manually set/clear CS line */
	    GPIO_PinModeSet(ICM_20948_CS_PORT, ICM_20948_CS_PIN, gpioModePushPull, 1); // US0_CS is push pull
	    GPIO_PinModeSet(ICM_20948_MISO_PORT, ICM_20948_MISO_PIN, gpioModeInput, 1);    // US0_RX (MISO) is input
	    GPIO_PinModeSet(ICM_20948_MOSI_PORT, ICM_20948_MOSI_PIN, gpioModePushPull, 1); // US0_TX (MOSI) is push pull

	    // Start with default config, then modify as necessary
	      USART_InitSync_TypeDef config = USART_INITSYNC_DEFAULT;
	      config.master       = true;            // master mode
	      //config.baudrate     = 1000000;         // CLK freq is 1 MHz
	      config.autoCsEnable = true;            // CS pin controlled by hardware, not firmware
	      config.clockMode    = usartClockMode0; // clock idle low, sample on rising/first edge
	      config.msbf         = true;            // send MSB first
	      config.enable       = usartDisable;    // making sure to keep USART disabled until we've set everything up

	      /* Modify some settings */
	      config.clockMode    = usartClockMode0; 	/* clock idle low, sample on rising/first edge (Clock polarity/phase mode = CPOL/CPHA) */
	      config.refFreq      = 0;			 	/* USART/UART reference clock assumed when configuring baud rate setup. Set to 0 to use the currently configured reference clock. */
	      config.baudrate     = 4000000;         // CLK freq is 4 MHz
	      config.databits     = usartDatabits8;	/* master mode */
	      config.autoTx = false; 					/* If enabled: Enable AUTOTX mode. Transmits as long as RX is not full. Generates underflows if TX is empty. */
	      config.autoCsEnable = false;            /* CS pin controlled by hardware, not firmware */

	      /* Initialize USART0/1 with the configured parameters */
	      USART_InitSync(USART0, &config);

	      // Set and enable USART pin locations
	      USART0->ROUTE = USART_ROUTE_CLKPEN | USART_ROUTE_CSPEN | USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_LOCATION_LOC0;

	  	/* Enable USART0/1 */
	  	USART_Enable(USART0, usartEnable);

	  	/* Set CS high (active low!) */
	  	/* Normally this isn't needed, in the case of gpioModePushPull", the last argument directly sets the pin state */
	  	GPIO_PinOutSet(gpioPortE, 13);


	  /*****************************/
}

/**************************************************************************//**
 * @brief
 *   Enables the necessary SPI interface to communicate with ICM20948
 *
 * @note
 *   SPI can draw some power
 *
 *   @note
 *   SPI is not used
 *
 * @note
 * 	 By setting the GPIO's to 0 state when going to sleep
 * 	 150 micro A <--> 270 micro A
 *
 * @param[in] enable
 *   @li `true` - Enable SPI communication.
 *   @li `false` - Disable SPI communication.
 *****************************************************************************/
void ICM_20948_enable_SPI(bool enable)
{
	if (enable)
	{
		/* Enable USART clock and peripheral */
		CMU_ClockEnable(cmuClock_USART0, true);

		USART_Enable(USART0, usartEnable);

	  	/* Configure GPIO */
	  	/* In the case of gpioModePushPull", the last argument directly sets the pin state */
	    GPIO_PinModeSet(ICM_20948_CLK_PORT, ICM_20948_CLK_PIN, gpioModePushPull, 0); // US0_CLK is push pull
	    /* Normally this is gpioPortE: pin 13 */
	    /* Apparently, can't use the US0_CS port (PE13) to manually set/clear CS line */
	    GPIO_PinModeSet(ICM_20948_CS_PORT, ICM_20948_CS_PIN, gpioModePushPull, 1); // US0_CS is push pull
	    GPIO_PinModeSet(ICM_20948_MISO_PORT, ICM_20948_MISO_PIN, gpioModeInput, 1);    // US0_RX (MISO) is input
	    GPIO_PinModeSet(ICM_20948_MOSI_PORT, ICM_20948_MOSI_PIN, gpioModePushPull, 1); // US0_TX (MOSI) is push pull
	}
	else
	{
		/* Disable USART clock and peripheral */
		CMU_ClockEnable(cmuClock_USART0, false);

		USART_Enable(USART0, usartDisable);

	  	/* Configure GPIO */
	  	/* In the case of gpioModePushPull", the last argument directly sets the pin state */
	    GPIO_PinModeSet(ICM_20948_CLK_PORT, ICM_20948_CLK_PIN, gpioModeDisabled, 0); // US0_CLK is push pull
	    /* Normally this is gpioPortE: pin 13 */
	    /* Apparently, can't use the US0_CS port (PE13) to manually set/clear CS line */
	    GPIO_PinModeSet(ICM_20948_CS_PORT, ICM_20948_CS_PIN, gpioModeDisabled, 1); // US0_CS is push pull
	    GPIO_PinModeSet(ICM_20948_MISO_PORT, ICM_20948_MISO_PIN, gpioModeDisabled, 1);    // US0_RX (MISO) is input
	    GPIO_PinModeSet(ICM_20948_MOSI_PORT, ICM_20948_MOSI_PIN, gpioModeDisabled, 1); // US0_TX (MOSI) is push pull

	}
}

/**************************************************************************//**
 * @brief
 *   Turns the GPIO pin high to provide power to the ICM20948
 *
 * @note
 *   max current 20mA, in total max 100mA (all GPIO pins combined)
 *
 * @param[in] enable
 *   @li `true` - Provide power to the IMU.
 *   @li `false` - Cut-off power to the IMU.
 *****************************************************************************/
void ICM_20948_power (bool enable)
{
	/* Initialize VDD pin if not already the case */
//	if (!ICM_20948_Initialized)
//	{
//		/* In the case of gpioModePushPull", the last argument directly sets the pin state */
//		GPIO_PinModeSet(ICM_20948_POWER_PORT, ICM_20948_POWER_PIN, gpioModePushPull, enable);

//		ICM_20948_Initialized = true;
//	}
//	else
//	{
		if (enable) GPIO_PinModeSet(ICM_20948_POWER_PORT, ICM_20948_POWER_PIN, gpioModePushPull, enable); /* Enable VDD pin */
		else GPIO_PinModeSet(ICM_20948_POWER_PORT, ICM_20948_POWER_PIN, gpioModeDisabled, 0); /* Disable VDD pin */
//	}
}

/**************************************************************************//**
 * @brief
 *   Print all gyro + accel + magn data using dbprint
 *
 *****************************************************************************/
void ICM_20948_printAllData()
{
	/* Gyro [DPS]
	 * Accel [G *1000] (to print it in Integers
	 */

	  /* Variables to store gyro & accel data
	   * GYRO: 	X --> ICM_20948_gyro[0];
	   * 		Y --> ICM_20948_gyro[1];
	   * 		Z --> ICM_20948_gyro[2];
	   * ACCEL:	X --> ICM_20948_accel[0]:
	   * 		Y --> ICM_20948_accel[1]:
	   * 		Z --> ICM_20948_accel[2]:
	   */
#if DEBUG_DBPRINTs == 1 /* DEBUG_DBPRINT */
	  dbprint("	Gyro x: ");
	  dbprintInt((int) ICM_20948_gyro[0] );
	  dbprint("	Gyro y: ");
	  dbprintInt((int) ICM_20948_gyro[1] );
	  dbprint("	Gyro z: ");
	  dbprintInt((int) ICM_20948_gyro[2] );
	  dbprint("	Accel x: ");
	  dbprintInt((int) (ICM_20948_accel[0]*1000) );
	  dbprint("	Accel y: ");
	  dbprintInt((int) (ICM_20948_accel[1]*1000) );
	  dbprint("	Accel z: ");
	  dbprintInt((int) (ICM_20948_accel[2]*1000) );
	  dbprint("	Magn x: ");
	  dbprintInt((int) ICM_20948_magn[0] );
	  dbprint("	Magn y: ");
	  dbprintInt((int) ICM_20948_magn[1] );
	  dbprint("	Magn z: ");
	  dbprintlnInt((int) ICM_20948_magn[2] );
#endif /* DEBUG_DBPRINT */

}

/**************************************************************************//**
 * @brief
 *   Check WhoAmI register of ICM20948
 *
 * @note
 * 	 Hangs when WhoAmI is not found or is not correct
 *
 *****************************************************************************/
void ICM_20948_check_WhoAmI()
{
	uint8_t WhoAmI;
	while( WhoAmI != 0xEA )
	{
		WhoAmI = ICM_20948_read( ICM_20948_BANK_0 | 0x00 );
	}
}

/**************************************************************************//**
 * @brief
 *   Set chipselect pin for SPI functionality
 *
 * @details
 *	 Only when using SPI
 *
 * @param[in] enable
 *   @li 'true' - CS pin high
 *   @li 'false' - CS pin low
 *
 *****************************************************************************/
void ICM_20948_chipSelectSet(bool enable)
{
	if ( enable )
	{
		/* Set CS low (active low) to start transmission */
		GPIO_PinOutClear(ICM_20948_CS_PORT, ICM_20948_CS_PIN);
	}
	else
	{
		/* Set CS high (active low) */
		GPIO_PinOutSet(ICM_20948_CS_PORT, ICM_20948_CS_PIN);
	}
}

//void ICM_20948_bankSelect(uint8_t bank)
//{
//  /* Enable chip select */
//  ICM_20948_chipSelectSet(true);
//
//  /* Select the Bank Select register */
//  USART_Tx(SPI, ICM_20948_REG_BANK_SEL);
//  USART_Rx(SPI);
//
//  /* Write the desired bank address 0..3 */
//  USART_Tx(SPI, (bank << 4) );
//  USART_Rx(SPI);
//
//  /* Disable chip select */
//  ICM_20948_chipSelectSet(false);
//
//  return;
//}

/**************************************************************************//**
 * @brief
 *   Select appropriate bank in ICM20948 based on first bits of address
 *
 * @note
 *	 Works only with I2C
 *
 * @details
 * 	wBuffer[0] = ICM_20948_REG_BANK_SEL;
	wBuffer[1] = (bank << 4);
 *
 * @param[in] bank
 *   Register to be read
 *
 *****************************************************************************/
void ICM_20948_bankSelect(uint8_t bank)
{
	uint8_t wBuffer[2];
	wBuffer[0] = ICM_20948_REG_BANK_SEL;
	wBuffer[1] = (bank << 4);

	IIC_WriteBuffer(ICM_20948_I2C_ADDRESS, wBuffer, 2);
}

/**************************************************************************//**
 * @brief
 *   Read single register from IMU using SPI
 *
 * @param[in] addr
 *   address 16 bits - first bits are bank, the rest of the bits is the address
 *
 *****************************************************************************/
uint8_t ICM_20948_read ( uint16_t addr )
{
	uint8_t data;
	uint8_t reg_address;
	uint8_t bank;

	/* Select last 7 bytes (A6 - A0) to use as register address */
	/* Address format: | R/W | A6 | A5 | A4 | A3 | A2 | A1 | A0 | */
	/* Use AND to change only last 7 bytes */
	reg_address = (uint8_t) (addr & 0b01111111);

	bank = (uint8_t) (addr >> 7);

	/* Select right bank */
	ICM_20948_bankSelect(bank);

	/* Enable CS */
	ICM_20948_chipSelectSet(true);

	/* Set the first byte to 1 to read (according to data sheet) */
	/* Address format: | R/W | A6 | A5 | A4 | A3 | A2 | A1 | A0 | */
	/* Read setup */
	USART_SpiTransfer(SPI, (reg_address | 0b10000000));

	/* Read data */
	data = USART_SpiTransfer(SPI, 0x00);

	/* Disable CS */
	ICM_20948_chipSelectSet(false);

	return (data);
}

//void ICM_20948_registerRead(uint16_t addr, int numBytes, uint8_t *data)
//{
//  uint8_t regAddr;
//  uint8_t bank;
//
//  regAddr = (uint8_t) (addr & 0x7F);
//  bank = (uint8_t) (addr >> 7);
//
//  ICM_20948_bankSelect(bank);
//
//  /* Enable chip select */
//  ICM_20948_chipSelectSet(true);
//
//  /* Set R/W bit to 1 - read */
//  USART_Tx(SPI, (regAddr | 0x80) );
//  USART_Rx(SPI);
//  /* Transmit 0's to provide clock and read the data */
//  while ( numBytes-- ) {
//    USART_Tx(SPI, 0x00);
//    *data++ = USART_Rx(SPI);
//  }
//
//  /* Disable chip select */
//  ICM_20948_chipSelectSet(false);
//
//  return;
//}


//void ICM_20948_registerWrite(uint16_t addr, uint8_t data)
//{
//  uint8_t regAddr;
//  uint8_t bank;
//
//  regAddr = (uint8_t) (addr & 0x7F);
//  bank = (uint8_t) (addr >> 7);
//
//  ICM_20948_bankSelect(bank);
//
//  /* Enable chip select */
//  ICM_20948_chipSelectSet(true);
//
//  /* clear R/W bit - write, send the address */
//  USART_Tx(SPI, (regAddr & 0x7F) );
//  USART_Rx(SPI);
//
//  /* Send the data */
//  //USART_SpiTransfer( SPI, data );
//
//  USART_Tx(SPI, data);
//  USART_Rx(SPI);
//
//  /* Disable chip select */
//  ICM_20948_chipSelectSet(false);
//
//  return;
//}


/**************************************************************************//**
 * @brief
 *   Read registers from IMU
 *
 * @details
 *	 Using I2C
 *
 *
 * @param[in] addr
 *   address
 * @param[in] numBytes
 * 	numer of bytes to read
 * @param[out] data
 * 	pointer to store data
 *
 *
 *****************************************************************************/
void ICM_20948_registerRead(uint16_t addr, int numBytes, uint8_t *data)
{
	uint8_t regAddr;
	uint8_t bank;



	uint8_t rLength = (uint8_t) numBytes;


	regAddr = (uint8_t) (addr & 0x7F);
	bank = (uint8_t) (addr >> 7);

	ICM_20948_bankSelect(bank);

	uint8_t wBuffer[2];
	wBuffer[0] = regAddr;
	wBuffer[1] = 0x00;	/* 0x00 to read */

	// TODO

	IIC_WriteReadBuffer(ICM_20948_I2C_ADDRESS, wBuffer, 1, data, rLength);


	return;
}


/**************************************************************************//**
 * @brief
 *   Writes registers from IMU
 *
 * @details
 *	 Using I2C
 *
 *
 * @param[in] addr
 *   address
 * @param[out] data
 * 	data to be written
 *
 *
 *****************************************************************************/
void ICM_20948_registerWrite(uint16_t addr, uint8_t data)
{
	uint8_t regAddr;
	uint8_t bank;

	regAddr = (uint8_t) (addr & 0x7F);
	bank = (uint8_t) (addr >> 7);

	ICM_20948_bankSelect(bank);

	uint8_t wBuffer[2];
	wBuffer[0] = regAddr;
	wBuffer[1] = data;


	IIC_WriteBuffer(ICM_20948_I2C_ADDRESS, wBuffer, 2);

	return;
}

/**************************************************************************//**
 * @brief
 *   Sleep mode enable function
 *
 *
 * @param[in] enable
 *   @li 'true' - sleep
 *   @li 'false' - active
 *
 *
 *****************************************************************************/
uint32_t ICM_20948_sleepModeEnable(bool enable)
{
  uint8_t reg;

  /* Read the Sleep Enable register */
  ICM_20948_registerRead(ICM_20948_REG_PWR_MGMT_1, 1, &reg);

  if ( enable ) {
    /* Sleep: set the SLEEP bit */
    reg |= ICM_20948_BIT_SLEEP;
  } else {
    /* Wake up: clear the SLEEP bit */
    //reg &= ~(ICM_20948_BIT_SLEEP); /* this was the provided code */

	/* My own solution */
	/* AND to define the bits that can be changed: here bit nr 6 */
	reg &= 0b10111111;
	/*OR met 0 to set the SLEEP bit to 0, not really necessary in this case */
	reg |= 0b00000000;
  }

  ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_1, reg);


  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Reset IMU
 *
 *
 *****************************************************************************/
uint32_t ICM_20948_reset(void)
{
  /* Set H_RESET bit to initiate soft reset */
  ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_1, ICM_20948_BIT_H_RESET);

  /* Wait 100ms to complete the reset sequence */
  delay(100);

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Read gyroscope data from IMU
 *
 *
 * @param[out] gyro
 *   pointer to store data
 *
 *****************************************************************************/
uint32_t ICM_20948_gyroDataRead(float *gyro)
{
  uint8_t rawData[6];
  float gyroRes;
  int16_t temp;

  /* Retrieve the current resolution */
  ICM_20948_gyroResolutionGet(&gyroRes);

  /* Read the six raw data registers into data array */
  ICM_20948_registerRead(GYRO_XOUT_H, 6, &rawData[0]);

  /* Convert the MSB and LSB into a signed 16-bit value and multiply by the resolution to get the dps value */
  temp = ( (int16_t) rawData[0] << 8) | rawData[1];
  gyro[0] = (float) temp * gyroRes;
  temp = ( (int16_t) rawData[2] << 8) | rawData[3];
  gyro[1] = (float) temp * gyroRes;
  temp = ( (int16_t) rawData[4] << 8) | rawData[5];
  gyro[2] = (float) temp * gyroRes;

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Set gyro resolution
 *
 *
 * @param[in/out] gyroRes
 *   gyro resulution input
 *   gyro actual resolution output
 *
 *****************************************************************************/
uint32_t ICM_20948_gyroResolutionGet(float *gyroRes)
{
  uint8_t reg;

  /* Read the actual gyroscope full scale setting */
  ICM_20948_registerRead(ICM_20948_REG_GYRO_CONFIG_1, 1, &reg);
  reg &= ICM_20948_MASK_GYRO_FULLSCALE;

  /* Calculate the resolution */
  switch ( reg ) {
    case ICM_20948_GYRO_FULLSCALE_250DPS:
      *gyroRes = 250.0 / 32768.0;
      break;

    case ICM_20948_GYRO_FULLSCALE_500DPS:
      *gyroRes = 500.0 / 32768.0;
      break;

    case ICM_20948_GYRO_FULLSCALE_1000DPS:
      *gyroRes = 1000.0 / 32768.0;
      break;

    case ICM_20948_GYRO_FULLSCALE_2000DPS:
      *gyroRes = 2000.0 / 32768.0;
      break;
  }

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Read staus of interrupt IMU
 *
 *
 * @param[out] intStatus
 *   pointer to store status
 *
 *****************************************************************************/
uint32_t ICM_20948_interruptStatusRead(uint32_t *intStatus)
{
  uint8_t reg[4];

  ICM_20948_registerRead(ICM_20948_REG_INT_STATUS, 4, reg);
  *intStatus = (uint32_t) reg[0];
  *intStatus |= ( ( (uint32_t) reg[1]) << 8);
  *intStatus |= ( ( (uint32_t) reg[2]) << 16);
  *intStatus |= ( ( (uint32_t) reg[3]) << 24);

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Enable interrupt for data ready or wake on motion
 *
 *
 * @param[in] dataReadyEnable
 *   @li 'true' - enable
 *   @li 'false' - disable
 *
 * @param[in] womEnable
 *   @li 'true' - enable
 *   @li 'false' - disable
 *
 *****************************************************************************/
uint32_t ICM_20948_interruptEnable(bool dataReadyEnable, bool womEnable)
{
  uint8_t intEnable;

  /* All interrupts disabled by default */
  intEnable = 0;

  /* Enable one or both of the interrupt sources if required */
  if ( womEnable ) {
    intEnable = ICM_20948_BIT_WOM_INT_EN;
  }
  /* Write value to register */
  ICM_20948_registerWrite(ICM_20948_REG_INT_ENABLE, intEnable);

  /* All interrupts disabled by default */
  intEnable = 0;

  if ( dataReadyEnable ) {
    intEnable = ICM_20948_BIT_RAW_DATA_0_RDY_EN;
  }

  /* Write value to register */
  ICM_20948_registerWrite(ICM_20948_REG_INT_ENABLE_1, intEnable);

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Combination function to set sample rate of gyro and accel at once
 *
 * @details
 *	 will automatically calculate a sample rate that fits the sample rate divider register
 *
 * @param[in/out] sampleRate
 *   desired sample rate
 *
 *****************************************************************************/
uint32_t ICM_20948_sampleRateSet(float sampleRate)
{
  ICM_20948_gyroSampleRateSet(sampleRate);
  ICM_20948_accelSampleRateSet(sampleRate);

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Sets the gyro sample rate
 *
 * @details
 *	 will automatically calculate a sample rate that fits the sample rate divider register
 *
 * @param[in/out] sampleRate
 *   desired sample rate
 *
 * @return
 * 	the actual sample rate
 *
 *****************************************************************************/
float ICM_20948_gyroSampleRateSet(float sampleRate)
{
  uint8_t gyroDiv;
  float gyroSampleRate;

  /* Calculate the sample rate divider */
  gyroSampleRate = (1125.0 / sampleRate) - 1.0;

  /* Check if it fits in the divider register */
  if ( gyroSampleRate > 255.0 ) {
    gyroSampleRate = 255.0;
  }

  if ( gyroSampleRate < 0 ) {
    gyroSampleRate = 0.0;
  }

  /* Write the value to the register */
  gyroDiv = (uint8_t) gyroSampleRate;
  ICM_20948_registerWrite(ICM_20948_REG_GYRO_SMPLRT_DIV, gyroDiv);

  /* Calculate the actual sample rate from the divider value */
  gyroSampleRate = 1125.0 / (gyroDiv + 1);

  return gyroSampleRate;
}

/**************************************************************************//**
 * @brief
 *   Sets the accelerometer sample rate
 *
 * @details
 *	 will automatically calculate a sample rate that fits the sample rate divider register
 *
 * @param[in/out] sampleRate
 *   desired sample rate
 *
 * @return
 * 	the actual sample rate
 *
 *****************************************************************************/
float ICM_20948_accelSampleRateSet(float sampleRate)
{
  uint16_t accelDiv;
  float accelSampleRate;

  /* Calculate the sample rate divider */
  accelSampleRate = (1125.0 / sampleRate) - 1.0;

  /* Check if it fits in the divider registers */
  if ( accelSampleRate > 4095.0 ) {
    accelSampleRate = 4095.0;
  }

  if ( accelSampleRate < 0 ) {
    accelSampleRate = 0.0;
  }

  /* Write the value to the registers */
  accelDiv = (uint16_t) accelSampleRate;
  ICM_20948_registerWrite(ICM_20948_REG_ACCEL_SMPLRT_DIV_1, (uint8_t) (accelDiv >> 8) );
  ICM_20948_registerWrite(ICM_20948_REG_ACCEL_SMPLRT_DIV_2, (uint8_t) (accelDiv & 0xFF) );

  /* Calculate the actual sample rate from the divider value */
  accelSampleRate = 1125.0 / (accelDiv + 1);

  return accelSampleRate;
}

/**************************************************************************//**
 * @brief
 *   Enter low power mode for selected sensors
 *
 *
 * @param[in] enAccel
 *   @li 'true' - go low power
 *   @li 'false' - low noise mode
 *
 * @param[in] enGyro
 *   @li 'true' - go low power
 *   @li 'false' - low noise mode
 *
 * @param[in] enTemp
 *   @li 'true' - go low power
 *   @li 'false' - low noise mode
 *
 * @return
 * 	OK if successful
 *
 *****************************************************************************/
uint32_t ICM_20948_lowPowerModeEnter(bool enAccel, bool enGyro, bool enTemp)
{
  uint8_t data;

  ICM_20948_registerRead(ICM_20948_REG_PWR_MGMT_1, 1, &data);

  if ( enAccel || enGyro || enTemp ) {
    /* Make sure that the chip is not in sleep */
    ICM_20948_sleepModeEnable(false);

    /* And in continuous mode */
    ICM_20948_cycleModeEnable(false);

    /* Enable the accelerometer and the gyroscope*/
    ICM_20948_sensorEnable(enAccel, enGyro, enTemp);
    delay(50);

    /* Enable cycle mode */
    ICM_20948_cycleModeEnable(true);

    /* Set the LP_EN bit to enable low power mode */
    data |= ICM_20948_BIT_LP_EN;
  } else {
    /* Enable continuous mode */
    ICM_20948_cycleModeEnable(false);

    /* Clear the LP_EN bit to disable low power mode */
    data &= ~ICM_20948_BIT_LP_EN;
  }

  /* Write the updated value to the PWR_MGNT_1 register */
  ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_1, data);

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Enable selected sensors
 *
 *
 * @param[in] accel
 *   @li 'true' - enable
 *   @li 'false' - disable
 *
 * @param[in] gyro
 *   @li 'true' - enable
 *   @li 'false' - disable
 *
 * @param[in] temp
 *   @li 'true' - enable
 *   @li 'false' - disable
 *
 * @return
 * 	OK if successful
 *
 *****************************************************************************/
uint32_t ICM_20948_sensorEnable(bool accel, bool gyro, bool temp)
{
  uint8_t pwrManagement1;
  uint8_t pwrManagement2;

  ICM_20948_registerRead(ICM_20948_REG_PWR_MGMT_1, 1, &pwrManagement1);
  pwrManagement2 = 0;

  /* To enable the accelerometer clear the DISABLE_ACCEL bits in PWR_MGMT_2 */
  if ( accel ) {
    pwrManagement2 &= ~(ICM_20948_BIT_PWR_ACCEL_STBY);
  } else {
    pwrManagement2 |= ICM_20948_BIT_PWR_ACCEL_STBY;
  }

  /* To enable gyro clear the DISABLE_GYRO bits in PWR_MGMT_2 */
  if ( gyro ) {
    pwrManagement2 &= ~(ICM_20948_BIT_PWR_GYRO_STBY);
  } else {
    pwrManagement2 |= ICM_20948_BIT_PWR_GYRO_STBY;
  }

  /* To enable the temperature sensor clear the TEMP_DIS bit in PWR_MGMT_1 */
  if ( temp ) {
    pwrManagement1 &= ~(ICM_20948_BIT_TEMP_DIS);
  } else {
    pwrManagement1 |= ICM_20948_BIT_TEMP_DIS;
  }

  /* Write back the modified values */
  ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_1, pwrManagement1);
  ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_2, pwrManagement2);

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Enable cycle mode on IMU
 *
 *
 * @param[in/out] enable
 *   @li 'true' - enable cycle mode
 *   @li 'false' - disable cycle mode
 *
 *****************************************************************************/
uint32_t ICM_20948_cycleModeEnable(bool enable)
{
  uint8_t reg;

  reg = 0x00;

  if ( enable ) {
    reg = ICM_20948_BIT_ACCEL_CYCLE | ICM_20948_BIT_GYRO_CYCLE;
  }

  ICM_20948_registerWrite(ICM_20948_REG_LP_CONFIG, reg);

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Read RAW accelerometer data
 *
 *
 * @param[out] accel
 *   pointer to resulting memory locaion
 *
 * @return
 * OK when done
 *
 *****************************************************************************/
uint32_t ICM_20948_accelDataRead(float *accel)
{
  uint8_t rawData[6];
  float accelRes;
  int16_t temp;

  /* Retrieve the current resolution */
  ICM_20948_accelResolutionGet(&accelRes);

  /* Read the six raw data registers into data array */
  ICM_20948_registerRead(ICM_20948_REG_ACCEL_XOUT_H_SH, 6, &rawData[0]);

  /* Convert the MSB and LSB into a signed 16-bit value and multiply by the resolution to get the G value */
  temp = ( (int16_t) rawData[0] << 8) | rawData[1];
  accel[0] = (float) temp * accelRes;
  temp = ( (int16_t) rawData[2] << 8) | rawData[3];
  accel[1] = (float) temp * accelRes;
  temp = ( (int16_t) rawData[4] << 8) | rawData[5];
  accel[2] = (float) temp * accelRes;

  return ICM_20948_OK;
}

/**************************************************************************//**
 * @brief
 *   Get accelerometer resultion
 *
 *
 * @param[out] accelRes
 *   pointer to accelerometer resulution
 *
 *****************************************************************************/
uint32_t ICM_20948_accelResolutionGet(float *accelRes)
{
  uint8_t reg;

  /* Read the actual acceleration full scale setting */
  ICM_20948_registerRead(ICM_20948_REG_ACCEL_CONFIG, 1, &reg);
  reg &= ICM_20948_MASK_ACCEL_FULLSCALE;

  /* Calculate the resolution */
  switch ( reg ) {
    case ICM_20948_ACCEL_FULLSCALE_2G:
      *accelRes = 2.0 / 32768.0;
      break;

    case ICM_20948_ACCEL_FULLSCALE_4G:
      *accelRes = 4.0 / 32768.0;
      break;

    case ICM_20948_ACCEL_FULLSCALE_8G:
      *accelRes = 8.0 / 32768.0;
      break;

    case ICM_20948_ACCEL_FULLSCALE_16G:
      *accelRes = 16.0 / 32768.0;
      break;
  }

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Set accelerometer full scale range
 *
 *
 * @param[in] accelFs
 *   full scale range, see datasheet for permitted values
 *
 *****************************************************************************/
uint32_t ICM_20948_accelFullscaleSet(uint8_t accelFs)
{
  uint8_t reg;

  accelFs &= ICM_20948_MASK_ACCEL_FULLSCALE;
  ICM_20948_registerRead(ICM_20948_REG_ACCEL_CONFIG, 1, &reg);
  reg &= ~(ICM_20948_MASK_ACCEL_FULLSCALE);
  reg |= accelFs;
  ICM_20948_registerWrite(ICM_20948_REG_ACCEL_CONFIG, reg);

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Set gyroscope full scale range
 *
 *
 * @param[in] gyroFs
 *   full scale range, see datasheet for permitted values
 *
 *****************************************************************************/
uint32_t ICM_20948_gyroFullscaleSet(uint8_t gyroFs)
{
  uint8_t reg;

  gyroFs &= ICM_20948_MASK_GYRO_FULLSCALE;
  ICM_20948_registerRead(ICM_20948_REG_GYRO_CONFIG_1, 1, &reg);
  reg &= ~(ICM_20948_MASK_GYRO_FULLSCALE);
  reg |= gyroFs;
  ICM_20948_registerWrite(ICM_20948_REG_GYRO_CONFIG_1, reg);

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Set wake on motion interrupt
 *
 *
 * @param[in] enable
 *   @li 'true' - enable wom interrupt
 *   @li 'false' - disable wom interrupt
 *
 * @param[in] womThreshold
 * 	value above which an interrupt is generated
 *
 * @param[in] sampleRate
 * 	sample rate of accel in duty cycle mode (low power mode)
 *
 *****************************************************************************/
uint32_t ICM_20948_wakeOnMotionITEnable(bool enable, uint8_t womThreshold, float sampleRate)
{
  if ( enable ) {
    /* Make sure that the chip is not in sleep */
    ICM_20948_sleepModeEnable(false);

    /* And in continuous mode */
    ICM_20948_cycleModeEnable(false);

    /* Enable only the accelerometer */
    ICM_20948_sensorEnable(true, false, false);

    /* Set sample rate */
    ICM_20948_sampleRateSet(sampleRate);

    /* Set the bandwidth to 1210Hz */
    ICM_20948_accelBandwidthSet(ICM_20948_ACCEL_BW_1210HZ);

    /* Accel: 2G full scale */
    ICM_20948_accelFullscaleSet(ICM_20948_ACCEL_FULLSCALE_2G);

    /* Enable the Wake On Motion interrupt */
    ICM_20948_interruptEnable(false, true);
    delay(50);

    /* Enable Wake On Motion feature */
    ICM_20948_registerWrite(ICM_20948_REG_ACCEL_INTEL_CTRL, ICM_20948_BIT_ACCEL_INTEL_EN | ICM_20948_BIT_ACCEL_INTEL_MODE);

    /* Set the wake on motion threshold value */
    ICM_20948_registerWrite(ICM_20948_REG_ACCEL_WOM_THR, womThreshold);

    /* Enable low power mode */
    ICM_20948_lowPowerModeEnter(true, false, false);
  } else {
    /* Disable Wake On Motion feature */
    ICM_20948_registerWrite(ICM_20948_REG_ACCEL_INTEL_CTRL, 0x00);

    /* Disable the Wake On Motion interrupt */
    ICM_20948_interruptEnable(false, false);

    /* Disable cycle mode */
    ICM_20948_cycleModeEnable(false);
  }

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Set gyroscope bandwidth
 *
 *
 * @param[in] gyroBw
 *   bandwidth, see datasheet for permitted values
 *
 *****************************************************************************/
uint32_t ICM_20948_gyroBandwidthSet(uint8_t gyroBw)
{
  uint8_t reg;

  /* Read the GYRO_CONFIG_1 register */
  ICM_20948_registerRead(ICM_20948_REG_GYRO_CONFIG_1, 1, &reg);
  reg &= ~(ICM_20948_MASK_GYRO_BW);

  /* Write the new bandwidth value to the gyro config register */
  reg |= (gyroBw & ICM_20948_MASK_GYRO_BW);
  ICM_20948_registerWrite(ICM_20948_REG_GYRO_CONFIG_1, reg);

  return ICM_20948_OK;
}

/**************************************************************************//**
 * @brief
 *   Set acceleromter bandwidth
 *
 *
 * @param[in] accelBw
 *   bandwidth, see datasheet for permitted values
 *
 *****************************************************************************/
uint32_t ICM_20948_accelBandwidthSet(uint8_t accelBw)
{
  uint8_t reg;

  /* Read the GYRO_CONFIG_1 register */
  ICM_20948_registerRead(ICM_20948_REG_ACCEL_CONFIG, 1, &reg);
  reg &= ~(ICM_20948_MASK_ACCEL_BW);

  /* Write the new bandwidth value to the gyro config register */
  reg |= (accelBw & ICM_20948_MASK_ACCEL_BW);
  ICM_20948_registerWrite(ICM_20948_REG_ACCEL_CONFIG, reg);

  return ICM_20948_OK;
}


/**************************************************************************//**
 * @brief
 *   Enables / disables the 50 micro seconds interrupt pin functionality
 *
 * @note
 *   written myself, changes the whole register, maybe not so good...
 *
 * @param[in] enable
 *   @li `true` - Enable 50 micro second interrupt.
 *   @li `false` - INT1 pin level held until interrupt status is cleared.
 *****************************************************************************/
uint32_t ICM_20948_latchEnable(bool enable)
{
	uint8_t temp;
	ICM_20948_registerRead(ICM_20948_REG_INT_PIN_CFG, 1, &temp);

	if(enable)
	{
		/* Set interrupt pin to pulse, no need to read a interrupt register for proceeding */
		/*  1 ï¿½ INT1 pin level held until interrupt status is cleared.
		 *	0 ï¿½ INT1 pin indicates interrupt pulse is width 50 us  <----
		 */
		ICM_20948_registerWrite(ICM_20948_REG_INT_PIN_CFG, temp | 0b00010000);
		//ICM_20948_registerWrite(ICM_20948_REG_INT_PIN_CFG, 0b00010000);
	}else{
		ICM_20948_registerWrite(ICM_20948_REG_INT_PIN_CFG, 0b00000000);
	}

	return ICM_20948_OK;
}



/**********************************************************************/
/**************              Magnetometer         *********************/
/**********************************************************************/

/**************************************************************************//**
 * @brief
 *   Set magnetometer data transfer on integrated I2C controller inside ICM20948
 *
 *
 * @note
 * 	 Not used, this function is only being used when using SPI, not working properly
 *
 *
 * @param[in] read
 *   @li 'true' - enable
 *   @li 'false' - disable
 *
 *****************************************************************************/
void ICM_20948_set_mag_transfer(bool read) {
    uint8_t MAG_BIT_READ   = AK09916_BIT_I2C_SLV_ADDR | ICM_20948_BIT_I2C_SLV_READ;
    uint8_t MAG_BIT_WRITE  = AK09916_BIT_I2C_SLV_ADDR;

    bool read_old = !read;

    /* Set transfer mode if it has changed */
    if (read != read_old) {
        if (read) {
            ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV0_ADDR, MAG_BIT_READ);
        }
        else {
        	ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV0_ADDR, MAG_BIT_WRITE);
        }
        read_old = read;
    }
    return;
}

/**************************************************************************//**
 * @brief
 *   Wait for I2C slave (internal I2C controller on ICM20948)
 *
 *
 * @note
 * 	 Not used, not working properly
 *
 *
 *****************************************************************************/
void waitForSlave4()
{
  // Wait until the data is ready:

    delay(100);
    unsigned char status = ICM_20948_read(ICM_20948_REG_I2C_MST_STATUS);
    if (status & ICM_20948_BIT_SLV4_NACK)
    {
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
		dbprint("Failed to communicate with compass: NACK");
#endif /* DEBUG_DBPRINT */
    }
    if (status & ICM_20948_BIT_SLV4_DONE)
    {
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
		dbprint("Transaction Done");
#endif /* DEBUG_DBPRINT */
    }
}


/**************************************************************************//**
 * @brief
 *   Read magnetometer register from I2C controller in ICM20948
 *
 *
 * @note
 * 	 Not used, not working properly
 *
 *
 * @param[in/out] magreg
 *   register to be read
 *
 * @return
 * 	read value
 *
 *****************************************************************************/
unsigned char readMagRegister(unsigned char magreg)
{
  // We use slave4, which is oneshot:
	ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV4_ADDR, ICM_20948_BIT_I2C_READ | AK09916_BIT_I2C_SLV_ADDR);
	ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV4_REG, magreg);
	ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV4_CTRL, ICM_20948_BIT_I2C_SLV_EN);

  waitForSlave4();

  return ICM_20948_read(ICM_20948_REG_I2C_SLV4_DI);
}



/**************************************************************************//**
 * @brief
 *   Write magnetometer register using internal I2C interface on ICM20948
 *
 *
 * @note
 * 	 Not used, not working properly
 *
 *
 * @param[in] magreg
 *   register
 * @param[out] val
 *   output register
 *
 *****************************************************************************/
void writeMagRegister(unsigned char magreg, unsigned char val)
{
  // We use slave4, which is oneshot:
	ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV4_ADDR, AK09916_BIT_I2C_SLV_ADDR);
	ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV4_REG, magreg);
	ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV4_DO, val);
	ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV4_CTRL, ICM_20948_BIT_I2C_SLV_EN);

  waitForSlave4();
}










/***************************************************************************//**
 * @brief
 *    Read register in the AK09916 magnetometer device
 *
 * @param[in] addr
 *    Register address to read from
 *    Bit[6:0] - register address
 *
 * @param[in] numBytes
 *    Number of bytes to read
 *
 * @param[out] data
 *    Data read from register
 *
 * @return
 *    None
 ******************************************************************************/
//void ICM_20948_read_mag_register(uint8_t addr, uint8_t numBytes, uint8_t *data) { // functie correct!
//
//    /* Set transfer mode to read */
//    ICM_20948_set_mag_transfer(true);
//
//    /* Set SLV0_REG to magnetometer register address */
//    ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV0_REG, addr);
//
//    /* Request bytes */
//    ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV0_CTRL, ICM_20948_BIT_I2C_SLV_EN | numBytes);
//
//    /* Wait some time for registers to fill */
//    delay(1);
//
//    /* Read bytes from the ICM20948 EXT_SLV_SENS_DATA registers */
//    ICM_20948_registerRead(ICM_20948_REG_EXT_SLV_SENS_DATA_00, numBytes, data);
//
//    return;
//}
void ICM_20948_read_mag_register(uint8_t addr, uint8_t numBytes, uint8_t *data) {
	uint8_t wBuffer[2];
	wBuffer[0] = addr;
	wBuffer[1] = 0x00;

	IIC_WriteReadBuffer( ( AK09916_BIT_I2C_SLV_ADDR << 1 ), wBuffer, 1, data, numBytes);
}

/**************************************************************************//**
 * @brief
 *   Writes a uint8_t to the I2C address of the magnetometer of the ICM_20948
 *
 * @note
 *   Values not updating, register must be read to update...
 *
 * @param[in] addr
 * The address that needs to be written to.
 * @param[in] data
 * The data that needs to be written.
 *
 *****************************************************************************/
//void ICM_20948_write_mag_register(uint8_t addr, uint8_t data) { // functie correct!
//
//	/* Set transfer mode to write */
//    ICM_20948_set_mag_transfer(false);
//    //ICM_20948_registerRead(ICM_20948_REG_I2C_SLV0_ADDR, 1, a);
//
//    /* Set SLV0_REG to magnetometer register address */
//    ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV0_REG, addr);
//
//
//    /* Store data to write inside SLV0_DO */
//    //ICM_20948_registerRead(ICM_20948_REG_I2C_SLV0_DO, 1, a);
//    ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV0_DO, data);
//    //ICM_20948_registerRead(ICM_20948_REG_I2C_SLV0_DO, 1, a);
//
//    /* Send one byte */
//    ICM_20948_registerWrite(ICM_20948_REG_I2C_SLV0_CTRL, ICM_20948_BIT_I2C_SLV_EN | 0x01);
//
//    //ICM_20948_registerRead(ICM_20948_REG_I2C_SLV0_DO, 1, a);
//    //ICM_20948_registerRead(ICM_20948_REG_I2C_SLV0_ADDR, 1, a);
//
//    /* Wait some time for registers to fill */
//    //delay(10);
//
//    uint8_t temp[1];
//    ICM_20948_read_mag_register(addr, 1, temp);
//    if(temp[0] == data)
//    {
//    	dbprintln("Mag data write correct!");
//    }
//
//    return;
//}
void ICM_20948_write_mag_register(uint8_t addr, uint8_t data) {
	uint8_t wBuffer[2];
	wBuffer[0] = addr;
	wBuffer[1] = data;

	IIC_WriteBuffer( ( AK09916_BIT_I2C_SLV_ADDR << 1 ), wBuffer, 2);
}


/**************************************************************************//**
 * @brief
 *   Set magnetometer mode (sample rate / on-off)
 *
 *
 * @note
 * 	 Must read special register to update values
 *
 *
 * @param[in] magMode
 *   Magnetometer mode
 *
 *****************************************************************************/
uint32_t ICM_20948_set_mag_mode(uint8_t magMode){
    switch ( magMode ) {
        case AK09916_BIT_MODE_POWER_DOWN:
            break;
        case AK09916_MODE_SINGLE:
            break;
        case AK09916_MODE_10HZ:
            break;
        case AK09916_MODE_20HZ:
            break;
        case AK09916_MODE_50HZ:
            break;
        case AK09916_MODE_100HZ:
            break;
        case AK09916_MODE_ST:
            break;
        default:
            return ERROR;
    }

    ICM_20948_write_mag_register(AK09916_REG_CONTROL_2, magMode);

    return OK;
}
/**************************************************************************//**
 * @brief
 *   Read magnetometer raw values
 *
 *
 * @note
 * 	 To update values, read all the way through the registers
 *
 *
 * @param[out] raw_magn
 *   output magnetometer raw values
 *
 *****************************************************************************/
void ICM_20948_magRawDataRead(float *raw_magn) {

	uint8_t data[8];
	ICM_20948_read_mag_register(0x11, 8, data);

	/* Convert the LSB and MSB into a signed 16-bit value */
	_hxcounts = (((int16_t) data[1] << 8) | data[0] );
	_hycounts = (((int16_t) data[3] << 8) | data[2] );
	_hzcounts = (((int16_t) data[5] << 8) | data[4] );

	/* Transform to same coordinate system as gyro & accel */
	raw_magn[0] = (float)(tX[0]*_hxcounts + tX[1]*_hycounts + tX[2]*_hzcounts);
	raw_magn[1] = (float)(tY[0]*_hxcounts + tY[1]*_hycounts + tY[2]*_hzcounts);
	raw_magn[2] = (float)(tZ[0]*_hxcounts + tZ[1]*_hycounts + tZ[2]*_hzcounts);

#if DEBUG_DBPRINTs == 1 /* DEBUG_DBPRINT */
			dbprint("rawMag x:  ");
			dbprintInt((int) raw_magn[0]);
			dbprint("	rawMag y:  ");
			dbprintInt((int) raw_magn[1]);
			dbprint("	minMag z:  ");
			dbprintInt((int) raw_magn[2]);
			dbprintln("");
#endif /* DEBUG_DBPRINT */

}


/**************************************************************************//**
 * @brief
 *   Convert raw magnetometer values to calibrated and scaled ones
 *
 *
 *
 * @param[out] magn
 *   calibrated values in �T
 *
 *****************************************************************************/
void ICM_20948_magDataRead(float *magn) {

	uint8_t data[8];
	ICM_20948_read_mag_register(0x11, 8, data);

	/* Convert the LSB and MSB into a signed 16-bit value */
	_hxcounts = (((int16_t) data[1] << 8) | data[0] );
	_hycounts = (((int16_t) data[3] << 8) | data[2] );
	_hzcounts = (((int16_t) data[5] << 8) | data[4] );

	magn[0] = (((float)(tX[0]*_hxcounts + tX[1]*_hycounts + tX[2]*_hzcounts) * _magScale) + _hxb)*_hxs;
	magn[1] = (((float)(tY[0]*_hxcounts + tY[1]*_hycounts + tY[2]*_hzcounts) * _magScale) + _hyb)*_hys;
	magn[2] = (((float)(tZ[0]*_hxcounts + tZ[1]*_hycounts + tZ[2]*_hzcounts) * _magScale) + _hzb)*_hzs;

}


/**************************************************************************//**
 * @brief
 *   Reset magnetometer
 *
 *
 *****************************************************************************/
uint32_t ICM_20948_reset_mag(void) {
    /* Set SRST bit to initiate soft reset */
    ICM_20948_write_mag_register(AK09916_REG_CONTROL_3, AK09916_BIT_SRST);

    /* Wait 100ms to complete reset sequence */
    delay(100);

    return OK;
}


/* Embedded 2 */

/**************************************************************************//**
 * @brief
 *   Test function for non-tilt compensated compass
 *
 *
 * @param[in] magn
 *   magnetometer values
 *
 * @param[out] angle
 * 	angle in degrees
 *
 *****************************************************************************/
void ICM_20948_magn_to_angle(float *magn, float *angle){

	angle[0] = atan2(magn[1], magn[0]) * M_PI/180.0f;

}


///////////////////////////////////////////////////////////////////////////////
/////////////////             CALIBRATION        //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/***************************************************************************//**
 * @brief
 *    Accelerometer and gyroscope calibration function. Reads the gyroscope
 *    and accelerometer values while the device is at rest and in level. The
 *    resulting values are loaded to the accel and gyro bias registers to cancel
 *    the static offset error.
 *
 * @param[out] accelBiasScaled
 *    The mesured acceleration sensor bias in mg
 *
 * @param[out] gyroBiasScaled
 *    The mesured gyro sensor bias in deg/sec
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM_20948_accelGyroCalibrate(float *accelBiasScaled, float *gyroBiasScaled)
{
  uint8_t data[12];
  uint16_t i, packetCount, fifoCount;
  int32_t gyroBias[3] = { 0, 0, 0 };
  int32_t accelBias[3] = { 0, 0, 0 };
  int32_t accelTemp[3];
  int32_t gyroTemp[3];
  int32_t accelBiasFactory[3];
  int32_t gyroBiasStored[3];
  float gyroRes, accelRes;

  /* Enable the accelerometer and the gyro */
  ICM_20948_sensorEnable(true, true, false);

  /* Set 1kHz sample rate */
  ICM_20948_sampleRateSet(1100.0);

  /* 246Hz BW for the accelerometer and 200Hz for the gyroscope */
  ICM_20948_accelBandwidthSet(ICM_20948_ACCEL_BW_246HZ);
  ICM_20948_gyroBandwidthSet(ICM_20948_GYRO_BW_12HZ);

  /* Set the most sensitive range: 2G full scale and 250dps full scale */
  ICM_20948_accelFullscaleSet(ICM_20948_ACCEL_FULLSCALE_2G);
  ICM_20948_gyroFullscaleSet(ICM_20948_GYRO_FULLSCALE_250DPS);

  /* Retrieve the resolution per bit */
  ICM_20948_accelResolutionGet(&accelRes);
  ICM_20948_gyroResolutionGet(&gyroRes);

  /* The accel sensor needs max 30ms, the gyro max 35ms to fully start */
  /* Experiments show that the gyro needs more time to get reliable results */
  delay(50);

  /* Disable the FIFO */
  ICM_20948_registerWrite(ICM_20948_REG_USER_CTRL, ICM_20948_BIT_FIFO_EN);
  ICM_20948_registerWrite(ICM_20948_REG_FIFO_MODE, 0x0F);

  /* Enable accelerometer and gyro to store the data in FIFO */
  ICM_20948_registerWrite(ICM_20948_REG_FIFO_EN_2, ICM_20948_BIT_ACCEL_FIFO_EN | ICM_20948_BITS_GYRO_FIFO_EN);

  /* Reset the FIFO */
  ICM_20948_registerWrite(ICM_20948_REG_FIFO_RST, 0x0F);
  ICM_20948_registerWrite(ICM_20948_REG_FIFO_RST, 0x00);

  /* Enable the FIFO */
  ICM_20948_registerWrite(ICM_20948_REG_USER_CTRL, ICM_20948_BIT_FIFO_EN);

  /* The max FIFO size is 4096 bytes, one set of measurements takes 12 bytes */
  /* (3 axes, 2 sensors, 2 bytes each value ) 340 samples use 4080 bytes of FIFO */
  /* Loop until at least 4080 samples gathered */
  fifoCount = 0;
  while ( fifoCount < 4080 ) {
    delay(5);
    /* Read FIFO sample count */
    ICM_20948_registerRead(ICM_20948_REG_FIFO_COUNT_H, 2, &data[0]);
    /* Convert to a 16 bit value */
    fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);
  }

  /* Disable accelerometer and gyro to store the data in FIFO */
  ICM_20948_registerWrite(ICM_20948_REG_FIFO_EN_2, 0x00);

  /* Read FIFO sample count */
  ICM_20948_registerRead(ICM_20948_REG_FIFO_COUNT_H, 2, &data[0]);

  /* Convert to a 16 bit value */
  fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);

  /* Calculate the number of data sets (3 axis of accel an gyro, two bytes each = 12 bytes) */
  packetCount = fifoCount / 12;

  /* Retrieve the data from the FIFO */
  for ( i = 0; i < packetCount; i++ ) {
    ICM_20948_registerRead(ICM_20948_REG_FIFO_R_W, 12, &data[0]);
    /* Convert to 16 bit signed accel and gyro x,y and z values */
    accelTemp[0] = ( (int16_t) (data[0] << 8) | data[1]);
    accelTemp[1] = ( (int16_t) (data[2] << 8) | data[3]);
    accelTemp[2] = ( (int16_t) (data[4] << 8) | data[5]);
    gyroTemp[0] = ( (int16_t) (data[6] << 8) | data[7]);
    gyroTemp[1] = ( (int16_t) (data[8] << 8) | data[9]);
    gyroTemp[2] = ( (int16_t) (data[10] << 8) | data[11]);

    /* Sum the values */
    accelBias[0] += accelTemp[0];
    accelBias[1] += accelTemp[1];
    accelBias[2] += accelTemp[2];
    gyroBias[0] += gyroTemp[0];
    gyroBias[1] += gyroTemp[1];
    gyroBias[2] += gyroTemp[2];
  }

  /* Divide by packet count to get the average */
  accelBias[0] /= packetCount;
  accelBias[1] /= packetCount;
  accelBias[2] /= packetCount;
  gyroBias[0] /= packetCount;
  gyroBias[1] /= packetCount;
  gyroBias[2] /= packetCount;

  /* Acceleormeter: add or remove (depending on the orientation of the chip) 1G (gravity) from the Z axis value */
  if ( accelBias[2] > 0L ) {
    accelBias[2] -= (int32_t) (1.0 / accelRes);
  } else {
    accelBias[2] += (int32_t) (1.0 / accelRes);
  }

  /* Convert the values to degrees per sec for displaying */
  gyroBiasScaled[0] = (float) gyroBias[0] * gyroRes;
  gyroBiasScaled[1] = (float) gyroBias[1] * gyroRes;
  gyroBiasScaled[2] = (float) gyroBias[2] * gyroRes;

  /* Read stored gyro trim values. After reset these values are all 0 */
  ICM_20948_registerRead(ICM_20948_REG_XG_OFFS_USRH, 2, &data[0]);
  gyroBiasStored[0] = ( (int16_t) (data[0] << 8) | data[1]);
  ICM_20948_registerRead(ICM_20948_REG_YG_OFFS_USRH, 2, &data[0]);
  gyroBiasStored[1] = ( (int16_t) (data[0] << 8) | data[1]);
  ICM_20948_registerRead(ICM_20948_REG_ZG_OFFS_USRH, 2, &data[0]);
  gyroBiasStored[2] = ( (int16_t) (data[0] << 8) | data[1]);

  /* The gyro bias should be stored in 1000dps full scaled format. We measured in 250dps to get */
  /* the best sensitivity, so need to divide by 4 */
  /* Substract from the stored calibration value */
  gyroBiasStored[0] -= gyroBias[0] / 4;
  gyroBiasStored[1] -= gyroBias[1] / 4;
  gyroBiasStored[2] -= gyroBias[2] / 4;

  /* Split the values into two bytes */
  data[0] = (gyroBiasStored[0] >> 8) & 0xFF;
  data[1] = (gyroBiasStored[0]) & 0xFF;
  data[2] = (gyroBiasStored[1] >> 8) & 0xFF;
  data[3] = (gyroBiasStored[1]) & 0xFF;
  data[4] = (gyroBiasStored[2] >> 8) & 0xFF;
  data[5] = (gyroBiasStored[2]) & 0xFF;

  /* Write the  gyro bias values to the chip */
  ICM_20948_registerWrite(ICM_20948_REG_XG_OFFS_USRH, data[0]);
  ICM_20948_registerWrite(ICM_20948_REG_XG_OFFS_USRL, data[1]);
  ICM_20948_registerWrite(ICM_20948_REG_YG_OFFS_USRH, data[2]);
  ICM_20948_registerWrite(ICM_20948_REG_YG_OFFS_USRL, data[3]);
  ICM_20948_registerWrite(ICM_20948_REG_ZG_OFFS_USRH, data[4]);
  ICM_20948_registerWrite(ICM_20948_REG_ZG_OFFS_USRL, data[5]);

  /* Calculate the accelerometer bias values to store in the hardware accelerometer bias registers. These registers contain */
  /* factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold */
  /* non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature */
  /* compensation calculations(? the datasheet is not clear). Accelerometer bias registers expect bias input */
  /* as 2048 LSB per g, so that the accelerometer biases calculated above must be divided by 8. */

  /* Read factory accelerometer trim values */
  ICM_20948_registerRead(ICM_20948_REG_XA_OFFSET_H, 2, &data[0]);
  accelBiasFactory[0] = ( (int16_t) (data[0] << 8) | data[1]);
  ICM_20948_registerRead(ICM_20948_REG_YA_OFFSET_H, 2, &data[0]);
  accelBiasFactory[1] = ( (int16_t) (data[0] << 8) | data[1]);
  ICM_20948_registerRead(ICM_20948_REG_ZA_OFFSET_H, 2, &data[0]);
  accelBiasFactory[2] = ( (int16_t) (data[0] << 8) | data[1]);

  /* Construct total accelerometer bias, including calculated average accelerometer bias from above */
  /* Scale the 2g full scale (most sensitive range) results to 16g full scale - divide by 8 */
  /* Clear the last bit (temperature compensation? - the datasheet is not clear) */
  /* Substract from the factory calibration value */

  accelBiasFactory[0] -= ( (accelBias[0] / 8) & ~1);
  accelBiasFactory[1] -= ( (accelBias[1] / 8) & ~1);
  accelBiasFactory[2] -= ( (accelBias[2] / 8) & ~1);

  /* Split the values into two bytes */
  data[0] = (accelBiasFactory[0] >> 8) & 0xFF;
  data[1] = (accelBiasFactory[0]) & 0xFF;
  data[2] = (accelBiasFactory[1] >> 8) & 0xFF;
  data[3] = (accelBiasFactory[1]) & 0xFF;
  data[4] = (accelBiasFactory[2] >> 8) & 0xFF;
  data[5] = (accelBiasFactory[2]) & 0xFF;

  /* Store them in the accelerometer offset registers */
  ICM_20948_registerWrite(ICM_20948_REG_XA_OFFSET_H, data[0]);
  ICM_20948_registerWrite(ICM_20948_REG_XA_OFFSET_L, data[1]);
  ICM_20948_registerWrite(ICM_20948_REG_YA_OFFSET_H, data[2]);
  ICM_20948_registerWrite(ICM_20948_REG_YA_OFFSET_L, data[3]);
  ICM_20948_registerWrite(ICM_20948_REG_ZA_OFFSET_H, data[4]);
  ICM_20948_registerWrite(ICM_20948_REG_ZA_OFFSET_L, data[5]);

  /* Convert the values to G for displaying */
  accelBiasScaled[0] = (float) accelBias[0] * accelRes;
  accelBiasScaled[1] = (float) accelBias[1] * accelRes;
  accelBiasScaled[2] = (float) accelBias[2] * accelRes;

  /* Turn off FIFO */
  ICM_20948_registerWrite(ICM_20948_REG_USER_CTRL, 0x00);

  /* Disable all sensors */
  ICM_20948_sensorEnable(false, false, false);

  return ICM_20948_OK;
}



/***************************************************************************//**
 * @brief
 *    Gyroscope calibration function. Reads the gyroscope
 *    values while the device is at rest and in level. The
 *    resulting values are loaded to the gyro bias registers to cancel
 *    the static offset error.
 *
 * @param[out] gyroBiasScaled
 *    The mesured gyro sensor bias in deg/sec
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM_20948_gyroCalibrate(float *gyroBiasScaled)
{
  uint8_t data[12];
  uint16_t i, packetCount, fifoCount;
  int32_t gyroBias[3] = { 0, 0, 0 };
  int32_t gyroTemp[3];
  int32_t gyroBiasStored[3];
  float gyroRes;

  /* Enable the accelerometer and the gyro */
  ICM_20948_sensorEnable(true, true, false);

  /* Set 1kHz sample rate */
  ICM_20948_sampleRateSet(1100.0);

  /* Configure bandwidth for gyroscope to 12Hz */
  ICM_20948_gyroBandwidthSet(ICM_20948_GYRO_BW_12HZ);

  /* Configure sensitivity to 250dps full scale */
  ICM_20948_gyroFullscaleSet(ICM_20948_GYRO_FULLSCALE_250DPS);

  /* Retrieve the resolution per bit */
  ICM_20948_gyroResolutionGet(&gyroRes);

  /* The accel sensor needs max 30ms, the gyro max 35ms to fully start */
  /* Experiments show that the gyro needs more time to get reliable results */
  delay(50);

  /* Disable the FIFO */
  ICM_20948_registerWrite(ICM_20948_REG_USER_CTRL, ICM_20948_BIT_FIFO_EN);
  ICM_20948_registerWrite(ICM_20948_REG_FIFO_MODE, 0x0F);

  /* Enable accelerometer and gyro to store the data in FIFO */
  ICM_20948_registerWrite(ICM_20948_REG_FIFO_EN_2, ICM_20948_BITS_GYRO_FIFO_EN);

  /* Reset the FIFO */
  ICM_20948_registerWrite(ICM_20948_REG_FIFO_RST, 0x0F);
  ICM_20948_registerWrite(ICM_20948_REG_FIFO_RST, 0x00);

  /* Enable the FIFO */
  ICM_20948_registerWrite(ICM_20948_REG_USER_CTRL, ICM_20948_BIT_FIFO_EN);

  /* The max FIFO size is 4096 bytes, one set of measurements takes 12 bytes */
  /* (3 axes, 2 sensors, 2 bytes each value ) 340 samples use 4080 bytes of FIFO */
  /* Loop until at least 4080 samples gathered */
  fifoCount = 0;
  while ( fifoCount < 4080 ) {
    delay(5);

    /* Read FIFO sample count */
    ICM_20948_registerRead(ICM_20948_REG_FIFO_COUNT_H, 2, &data[0]);

    /* Convert to a 16 bit value */
    fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);
  }

  /* Disable accelerometer and gyro to store the data in FIFO */
  ICM_20948_registerWrite(ICM_20948_REG_FIFO_EN_2, 0x00);

  /* Read FIFO sample count */
  ICM_20948_registerRead(ICM_20948_REG_FIFO_COUNT_H, 2, &data[0]);

  /* Convert to a 16 bit value */
  fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);

  /* Calculate the number of data sets (3 axis of accel an gyro, two bytes each = 12 bytes) */
  packetCount = fifoCount / 12;

  /* Retrieve the data from the FIFO */
  for ( i = 0; i < packetCount; i++ ) {
    ICM_20948_registerRead(ICM_20948_REG_FIFO_R_W, 12, &data[0]);
    /* Convert to 16 bit signed accel and gyro x,y and z values */
    gyroTemp[0] = ( (int16_t) (data[6] << 8) | data[7]);
    gyroTemp[1] = ( (int16_t) (data[8] << 8) | data[9]);
    gyroTemp[2] = ( (int16_t) (data[10] << 8) | data[11]);

    /* Sum the values */
    gyroBias[0] += gyroTemp[0];
    gyroBias[1] += gyroTemp[1];
    gyroBias[2] += gyroTemp[2];
  }

  /* Divide by packet count to get the average */
  gyroBias[0] /= packetCount;
  gyroBias[1] /= packetCount;
  gyroBias[2] /= packetCount;

  /* Convert the values to degrees per sec for displaying */
  gyroBiasScaled[0] = (float) gyroBias[0] * gyroRes;
  gyroBiasScaled[1] = (float) gyroBias[1] * gyroRes;
  gyroBiasScaled[2] = (float) gyroBias[2] * gyroRes;

  /* Read stored gyro trim values. After reset these values are all 0 */
  ICM_20948_registerRead(ICM_20948_REG_XG_OFFS_USRH, 2, &data[0]);
  gyroBiasStored[0] = ( (int16_t) (data[0] << 8) | data[1]);

  ICM_20948_registerRead(ICM_20948_REG_YG_OFFS_USRH, 2, &data[0]);
  gyroBiasStored[1] = ( (int16_t) (data[0] << 8) | data[1]);

  ICM_20948_registerRead(ICM_20948_REG_ZG_OFFS_USRH, 2, &data[0]);
  gyroBiasStored[2] = ( (int16_t) (data[0] << 8) | data[1]);

  /* The gyro bias should be stored in 1000dps full scaled format. We measured in 250dps to get */
  /* the best sensitivity, so need to divide by 4 */
  /* Substract from the stored calibration value */
  gyroBiasStored[0] -= gyroBias[0] / 4;
  gyroBiasStored[1] -= gyroBias[1] / 4;
  gyroBiasStored[2] -= gyroBias[2] / 4;

  /* Split the values into two bytes */
  data[0] = (gyroBiasStored[0] >> 8) & 0xFF;
  data[1] = (gyroBiasStored[0]) & 0xFF;
  data[2] = (gyroBiasStored[1] >> 8) & 0xFF;
  data[3] = (gyroBiasStored[1]) & 0xFF;
  data[4] = (gyroBiasStored[2] >> 8) & 0xFF;
  data[5] = (gyroBiasStored[2]) & 0xFF;

  /* Write the  gyro bias values to the chip */
  ICM_20948_registerWrite(ICM_20948_REG_XG_OFFS_USRH, data[0]);
  ICM_20948_registerWrite(ICM_20948_REG_XG_OFFS_USRL, data[1]);
  ICM_20948_registerWrite(ICM_20948_REG_YG_OFFS_USRH, data[2]);
  ICM_20948_registerWrite(ICM_20948_REG_YG_OFFS_USRL, data[3]);
  ICM_20948_registerWrite(ICM_20948_REG_ZG_OFFS_USRH, data[4]);
  ICM_20948_registerWrite(ICM_20948_REG_ZG_OFFS_USRL, data[5]);

  /* Turn off FIFO */
  ICM_20948_registerWrite(ICM_20948_REG_USER_CTRL, 0x00);

  /* Disable all sensors */
  ICM_20948_sensorEnable(false, false, false);

  return ICM_20948_OK;
}


/*Code based on:
 * https://appelsiini.net/2018/calibrate-magnetometer/
 * */


/***************************************************************************//**
 * @brief
 *    Calculate minimum and maximum magnetometer values,
 *    which are used for calibration.
 *
 *
 * @param[out] minMag
 * 	pointer to minima from each axis
 * @param[out] maxMag
 * 	pointer to maxima from each axis
 *
 * @return
 *    'OK' if successful,
 *    'ERROR' on error.
 ******************************************************************************/
uint32_t ICM_20948_min_max_mag(int16_t *minMag, int16_t *maxMag) {
    float m[3];

    minMag[0] = 32767; // alles kleiner dan dit wordt het minimum, daar staat er hier geen '-'!
    minMag[1] = 32767;
    minMag[2] = 32767;

    maxMag[0] = -32768; // hier hetzelfde, starten met de kleinste waarde, dan aanpassen dat het groter en groter wordt tot het maximum gevonden wordt!
    maxMag[1] = -32768;
    maxMag[2] = -32768;

    uint32_t max_time = 15000;
    uint32_t t_start = millis();

    /* Flags for if axis mag_minimumRange is fulfilled */
    bool miniumRange_mx = false;
    bool miniumRange_my = false;
    bool miniumRange_mz = false;


    while (!miniumRange_mx || !miniumRange_my || !miniumRange_mz) {
        while ((millis() - t_start) < max_time)  {

            /* read magnetometer measurement */
        	ICM_20948_magDataRead( m );

			if (m[0] < minMag[0]) {
				minMag[0] = m[0];
			}
			if (m[0] > maxMag[0]) {
				maxMag[0] = m[0];
			}

			if (m[1] < minMag[1]) {
				minMag[1] = m[1];
			}
			if (m[1] > maxMag[1]) {
				maxMag[1] = m[1];
			}

			if (m[2] < minMag[2]) {
				minMag[2] = m[2];
			}
			if (m[2] > maxMag[2]) {
				maxMag[2] = m[2];
			}

			if ((maxMag[0] - minMag[0]) > mag_minimumRange) {
				miniumRange_mx = true;
			}
			if ((maxMag[1] - minMag[1]) > mag_minimumRange) {
				miniumRange_my = true;
			}
			if ((maxMag[2] - minMag[2]) > mag_minimumRange) {
				miniumRange_mz = true;
			}

//                DEBUG_PRINT(min_mx); DEBUG_PRINT("\t"); DEBUG_PRINT(min_my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(min_mz);
//                DEBUG_PRINT(max_mx); DEBUG_PRINT("\t"); DEBUG_PRINT(max_my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(max_mz);
//                DEBUG_PRINTLN();
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
			dbprint("minMag:  ");
			dbprintInt((int) minMag[0]);
			dbprint("	minMag:  ");
			dbprintInt((int) minMag[1]);
			dbprint("	minMag:  ");
			dbprintInt((int) minMag[2]);
			dbprint("	maxMag:  ");
			dbprintInt((int) maxMag[0]);
			dbprint("	maxMag:  ");
			dbprintInt((int) maxMag[1]);
			dbprint("	maxMag:  ");
			dbprintInt((int) maxMag[2]);
			dbprintln("");
#endif /* DEBUG_DBPRINT */
        }
    }

    return OK;
}



/** Magnetometer calibration function. Get magnetometer minimum and maximum values, while moving
 *  the device in a figure eight. Those values are then used to cancel out hard and soft iron distortions.
 *

 */
/**************************************************************************//**
 *
 * @brief
 * Magnetometer calibration function. Get magnetometer minimum and maximum values, while moving the device in a figure eight. Those values are then used to cancel out hard and soft iron distortions.
 *
 * @param[out] offset
 * Magnetometer XYZ axis hard iron distortion correction.
 * @param[out] scale
 * Magnetometer XYZ axis soft iron distortion correction.
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 *
 *****************************************************************************/
bool ICM_20948_calibrate_mag(float *offset, float *scale) {
    int16_t minMag[3];
    int16_t maxMag[3];
    int32_t sum_mx, sum_my, sum_mz;
    int32_t dif_mx, dif_my, dif_mz, dif_m;

//    /* Reset hard and soft iron correction before calibration. */
    _hxb = 0.0f;
    _hyb = 0.0f;
    _hzb = 0.0f;
    _hxs = 1.0f;
    _hys = 1.0f;
    _hzs = 1.0f;

//    DEBUG_PRINTLN(F("Calibrating magnetometer. Move the device in a figure eight ..."));

    ICM_20948_min_max_mag( minMag, maxMag );

    sum_mx = (int32_t) maxMag[0] + minMag[0];
    sum_my = (int32_t) maxMag[1] + minMag[1];
    sum_mz = (int32_t) maxMag[2] + minMag[2];

    dif_mx = (int32_t) maxMag[0] - minMag[0];
    dif_my = (int32_t) maxMag[1] - minMag[1];
    dif_mz = (int32_t) maxMag[2] - minMag[2];

    offset[0] = -0.5f * sum_mx;
    offset[1] = -0.5f * sum_my;
    offset[2] = -0.5f * sum_mz;

    dif_m = (dif_mx + dif_my + dif_mz) / 3;

    scale[0] = (float) dif_m / dif_mx;
    scale[1] = (float) dif_m / dif_my;
    scale[2] = (float) dif_m / dif_mz;

//    Store offset values in local variables, to be accessable by magread functions
    _hxb = offset[0];
    _hyb = offset[1];
    _hzb = offset[2];

    _hxs = scale[0];
    _hys = scale[1];
    _hzs = scale[2];


//    DEBUG_PRINTLN(F("Hard iron correction values (center values):"));
//    DEBUG_PRINT2(offset_mx, 1);
//    DEBUG_PRINT("\t");
//    DEBUG_PRINT2(offset_my, 1);
//    DEBUG_PRINT("\t");
//    DEBUG_PRINT2(offset_mz, 1);
//    DEBUG_PRINT("\t");
//    DEBUG_PRINTLN();
//
//    DEBUG_PRINTLN(F("Soft iron correction values (scale values):"));
//    DEBUG_PRINT2(scale_mx, 4);
//    DEBUG_PRINT("\t");
//    DEBUG_PRINT2(scale_my, 4);
//    DEBUG_PRINT("\t");
//    DEBUG_PRINT2(scale_mz, 4);
//    DEBUG_PRINTLN();

    return true;
}



