/* Masterproef Design Sensor Node */
/* Made By Jona Cappelle */

/*************************************************/
/****                                     ********/
/******             MAIN FILE               ******/
/****                                     ********/
/*************************************************/

/***************************************************************************//**
 * @file main.c
 * @brief Code used in master thesis, MAIN FILE
 * @details Development of sensor node to use for threating NMSA
 * @version 1.0
 * @author Jona Cappelle
 * ******************************************************************************
 *
 * @section Versions
 *
 *   Please check https://github.com/jonacappelle/Master-Thesis to find the latest version!
 *
 *   @li v1.0: Master Thesis result
 *
 * ******************************************************************************
 *
 * @todo
 *   **Future improvements:**@n
 *     - Measurement of accuracy of sensor node against reference system
 *     - Enable remote calibration by receiving special packet from BLE
 *
 * ******************************************************************************
 *
 * @section License
 *
 *   **Copyright (C) 2020 - Jona Cappelle**
 *
 *
 ******************************************************************************/










/*
 * Measurement of time
 *
 * uint32_t start = millis();
 * uint32_t duration = millis() - start;
 *
 */

/* Include system libraries */
#include <adc.h>
#include "em_system.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_rtc.h"
#include "em_adc.h"
//#include "em_wdog.h"
#include "rtcdriver.h"
#include "em_core.h"

/* Include home made libraries */
#include "debug_dbprint.h"
#include "delay.h"
#include "ICM20948.h"
#include "interrupt.h"
#include "ble.h"
#include "uart.h"
#include "timer.h"
#include "datatypes.h"
#include "util.h"

/* Need to make a separate file called "rtcdrv_config.h" and place:
 *
 * #define EMDRV_RTCDRV_WALLCLOCK_CONFIG
 * To enable clock functionality
 // */
#include "rtcdrv_config.h" // voor millis();

/* Separate file for Pins and Constants */
#include "pinout.h"

/* Needed to use uintx_t */
#include <stdint.h>
#include <stdbool.h>
//#include "stdio.h"

/* Communication */
#include "I2C.h"
//#include "em_i2c.h"
//#include "i2cspm.h"				/* I2C higher level library to easily setup and use I2C */

/* Sensor fusion */
#include "MadgwickAHRS.h"
#include "math.h"

/* LED's */
#include "bsp.h"

/* Energy profiles trace code */
#include "bsp_trace.h"

/*************************************************/
/*************************************************/

#define DIY				1							/**< Variable to change between pinout of sensor node and pinout of development board */

/* The use of switch - cases makes the code more user friendly */
static volatile APP_State_t appState;				/**< Struct to keep track of the appState */

/* Keep the measurement data */
MeasurementData_t data;								/**< Struct to store al the measured data and calibration values */

/* bool to check if bluetooth is connected */
bool bleConnected = false;							/**< Keep track of the state of the BLE module */
bool IMU_MEASURING = false;							/**< Keep track of the state of the IMU */
bool IDLE = false;									/**< Variable to keep track if the system is IDLE of the IDLE state, not used at the moment */
bool _sleep = false;								/**< Variable to fix some problems with IMU generating interrupt and thus waking up the system when trying to go to sleep */

uint8_t idle_count = 0;								/**< Seconds that the IMU is idle */
uint8_t ble_sec_not_connected = 0;					/**< 200 ms that the BLE is not connected */

uint32_t interruptStatus[1];						/**< Not used at the moment */

/* Timer for IMU idle checking */
RTCDRV_TimerID_t IMU_Idle_Timer;					/**< Timer used for checking variables every second */

/* Test pin to check frequency of execution */

bool helft = false;									/**< Not used at the moment */

uint8_t teller = 0;									/**< Not used at the moment */
uint8_t teller_accuracy = 0;						/**< Counter to keep track when to enter accuracy mode on Madgwick filter */

volatile float beta = 1.0f;							/**< Beta parameter of Madgwick filter */


/*************************************************/
/*************************************************/

/**************************************************************************//**
 * @brief
 *   Function called by RTC timer every second
 *
 * @details
 *	 Check batt
 *	 Check gyro idle
 *	 Check BLE connected
 *	 Dynamically set Madgwick beta parameter
 *
 *
 *****************************************************************************/
void CheckIMUidle( void )
{

	/* Read battery in percent */
	ADC_get_batt(data.batt);

	float mean_gyro = (uint8_t) ( data.ICM_20948_gyro[0] + data.ICM_20948_gyro[1] + data.ICM_20948_gyro[2] ) / 3;

	/* Add 1 sec to the count of idle seconds */
	if( mean_gyro < 0.1 )
	{
		idle_count++;
	}else{ /* If there is movement, reset idle seconds */
		idle_count = 0;
	}


	if( (idle_count > 60) || (ble_sec_not_connected > 5*5)) // 5sec * 75Hz
	{
		idle_count = 0;
		ble_sec_not_connected = 0;
		teller_accuracy = 0;
		beta = 1.0f;
		/* Dont't check idle state in sleep */
		RTCDRV_StopTimer( IMU_Idle_Timer );
		/* Stop generating interrupts */
		ICM_20948_interruptEnable(false, false);
		appState = SLEEP;
		_sleep = true;
	}
	// Reset timer is BLE is connected
	if(bleConnected)
	{
		ble_sec_not_connected = 0;
	}

/* For visual representation where in the code */
#if DIY == 0
	BSP_LedToggle(1);
#endif

	if(teller_accuracy < 10)
	{
		teller_accuracy++;
	}

	if(teller_accuracy >= 10)
	{
		beta = 0.05f;
	}

}

/**************************************************************************//**
 * @brief
 *   Function called by interrupt from IMU @50 Hz
 *
 * @details
 *	 Measure Gyro + Accel + Magn
 *	 Sensor fusion with Madgwick filter
 *	 Convert quaternions to Euler angles
 *	 Convert floats to uint8_t for transmission
 *	 Send data via UART to BLE module (interrupt based)
 *	 Toggle pin to check speed
 *
 * @note
 * 	 Max execution speed 75 Hz
 *
 *
 *
 *****************************************************************************/
void measure_send( void )
{

	/* Check connection */
#if DEBUG_DBPRINT == 0 /* DEBUG_DBPRINT */
	BLE_check_connect();
	if (!bleConnected)
	{
		ble_sec_not_connected++;
		BLE_power(true);
		delay(200);
		BLE_connect();
	}

#endif /* DEBUG_DBPRINT */

	/* Read all sensors */
	ICM_20948_gyroDataRead(data.ICM_20948_gyro);
	ICM_20948_accelDataRead(data.ICM_20948_accel);
	ICM_20948_magDataRead(data.ICM_20948_magn);

	// TODO: embedded ICM_20948_magn_to_angle( ICM_20948_magn, ICM_20948_magn_angle );

//	uint32_t start = millis();

	/* Sensor fusion */
	MadgwickAHRSupdate(data.ICM_20948_gyro[0] * M_PI / 180.0f,
			data.ICM_20948_gyro[1] * M_PI / 180.0f,
			data.ICM_20948_gyro[2] * M_PI / 180.0f,
			data.ICM_20948_accel[0], data.ICM_20948_accel[1],
			data.ICM_20948_accel[2], data.ICM_20948_magn[0], data.ICM_20948_magn[1], data.ICM_20948_magn[2]);
	QuaternionsToEulerAngles(data.ICM_20948_euler_angles);



//	uint32_t duration = millis() - start;

	/* Convert float's to uint8_t arrays for transmission over BLE */
	float_to_uint8_t_x3(data.ICM_20948_euler_angles,
			data.BLE_euler_angles);

//	helft = !helft;


#if DEBUG_DBPRINT == 0 /* DEBUG_DBPRINT */
//	if(helft == 1)
//	{

//	if(teller < 3)
//	{
	BLE_sendData(data.BLE_euler_angles, data.batt, 13, data.BLE_data);
	/* Test frequency */
	GPIO_PinOutSet(gpioPortE, 11);
	GPIO_PinOutClear(gpioPortE, 11);
//	}else{
//		teller = 0;
//	}
//	teller++;


//	}
#endif /* DEBUG_DBPRINT */

#if DIY == 0
	BSP_LedToggle(0);
#endif


}


/**************************************************************************//**
 * @brief
 *   Main function
 *
 * @details
 *	State diagram
 *	BATT_READ and CALLIBRATE will never be reached
 *	Future update: calibrate when receiving special BLE packet
 *
 *
 *****************************************************************************/
int main(void)
{

	/* Start with initialization */
	appState = INIT;

	/* Infinite loop */
	while (1)
	{
		switch (appState)
		{
		/***************************/
		case SYS_IDLE:
		{
			/* Wait in EM2 */
//			EMU_EnterEM2(true);

		}
			break;
		/***************************/
		case INIT:
		{
			/* Chip errata */
			CHIP_Init();


			/* ENABLE CODE CORRELATION --  uses SWO */
			/* If first word of user data page is non-zero, enable eA Profiler trace */
//			BSP_TraceProfilerSetup();

			/* FULL SPEED */
			CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
//			CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

			CMU_ClockEnable(cmuClock_HFPER, true);
			CMU_ClockEnable(cmuClock_GPIO, true);

			/* Set output pin to check frequency of execution */

			GPIO_PinModeSet(gpioPortE, 11, gpioModePushPull, 0);

#if DIY == 1
			GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 1);
			delay(1000);
			GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModeDisabled, 0);

			blink(3);
#endif

			/* Leds on development board */
#if DIY == 0
			BSP_LedsInit();
#endif

			/* Setup printing to virtual COM port, w */
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
			dbprint_INIT(USART1, 4, true, false);
#endif /* DEBUG_DBPRINT */

			/* Timer init */
			RTCDRV_Init();
			RTCDRV_AllocateTimer(&IMU_Idle_Timer);


			/* Initialize ICM_20948 + SPI interface */
			ICM_20948_Init();
			//ICM_20948_Init_SPI();

			/* Initialize GPIO interrupts on port C 2 */
			initGPIO_interrupt();

			/* Initialize ADC to read battery voltage */
			initADC();
//			ADC_get_batt(data.batt);



#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
			dbprintln("INIT");
#endif /* DEBUG_DBPRINT */

			/* Bluetooth */
#if DEBUG_DBPRINT == 0 /* DEBUG_DBPRINT */
			BLE_Init();
#endif /* DEBUG_DBPRINT */

			delay(200);
			/* Set output power */
//			BLE_set_output_power(BLE_OUTPUT_POWER_0DB);
//			delay(100);

			// TODO Temporary
			/* Check connection */
#if DEBUG_DBPRINT == 0 /* DEBUG_DBPRINT */
			BLE_check_connect();
			if (!bleConnected)
			{
				BLE_power(true);
				delay(1000);
				BLE_connect();
			}
#endif /* DEBUG_DBPRINT */

			/* Set interrupt to trigger every 100ms when the data is ready */
			IMU_MEASURING = true;
			ICM_20948_interruptEnable(true, false);

			/* Fancy LED's */
#if DIY == 0
			for(uint8_t i=0; i<8; i++)
			{
				BSP_LedSet(0);
				BSP_LedSet(1);
				delay(100);
				BSP_LedClear(0);
				BSP_LedClear(1);
				delay(100);
			}
#endif

			/* Timer for checking if IMU is idle */
			  RTCDRV_StartTimer( IMU_Idle_Timer, rtcdrvTimerTypePeriodic, 2000, (RTCDRV_Callback_t)CheckIMUidle, NULL);



			appState = SYS_IDLE;


		}
			break;
			/***************************/
		case SENSORS_READ:
		{

#if DEBUG_DBPRINTs == 1 /* DEBUG_DBPRINT */
			dbprintln("SENSORS_READ");
#endif /* DEBUG_DBPRINT */

			measure_send();

#if DEBUG_DBPRINTs == 1 /* DEBUG_DBPRINT */
			dbprintInt((int) ( data.ICM_20948_gyro[0]*100) );
			dbprint(",");
			dbprintInt((int) (data.ICM_20948_gyro[1] *100) );
			dbprint(",");
			dbprintInt((int) (data.ICM_20948_gyro[2] *100) );
			dbprint(",");
			dbprintInt((int) ( data.ICM_20948_accel[0]*100) );
			dbprint(",");
			dbprintInt((int) (data.ICM_20948_accel[1] *100) );
			dbprint(",");
			dbprintlnInt((int) (data.ICM_20948_accel[2] *100) );
#endif /* DEBUG_DBPRINT */

#if DEBUG_DBPRINTs == 1 /* DEBUG_DBPRINT */
			//dbprint("	roll: ");
			dbprintInt((int) (ICM_20948_euler_angles[0] *100) );
			//dbprint("	pitch: ");
			dbprint("\t");
			dbprintInt((int) (ICM_20948_euler_angles[1] *100) );
			//dbprint("	yaw: ");
			dbprint("\t");
			dbprintlnInt((int) (ICM_20948_euler_angles[2] *100) );
#endif /* DEBUG_DBPRINT */

#if DEBUG_DBPRINTs == 1 /* DEBUG_DBPRINT */
			dbprintInt((int) ICM_20948_magn[0] );
			dbprint(",");
			dbprintInt((int) ICM_20948_magn[1] );
			dbprint(",");
			dbprintlnInt((int) ICM_20948_magn[2] );
#endif /* DEBUG_DBPRINT */


			/* If imu is idle, give it the chance to go to sleep */
			if(!_sleep)
			{
				appState = SYS_IDLE;
			}
		}
			break;
			/***************************/
		case BATT_READ:
		{
			ADC_Batt_Read();
			ADC_get_batt(data.batt);

#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
			dbprintln("BATT_READ");
#endif /* DEBUG_DBPRINT */

			appState = SYS_IDLE;
		}
			break;
			/***************************/
		case SLEEP:
		{
			//RTCDRV_StartTimer( IMU_Idle_Timer, rtcdrvTimerTypeOneshot, 2000, test, NULL);

			RTCDRV_StopTimer( IMU_Idle_Timer );
			RTCDRV_DeInit();

			BLE_disconnect();
			delay(100);
			BLE_power( false);
			BLE_rxtx_enable( false );

			bleConnected = false;

			CMU_ClockEnable(cmuClock_ADC0, false);
//			GPIO_PinModeSet(gpioPortD, 4, gpioModeDisabled, 0);

			GPIO_PinModeSet(gpioPortE, 11, gpioModeDisabled, 0);


			/* Shut down magnetometer */
		    ICM_20948_set_mag_mode(AK09916_BIT_MODE_POWER_DOWN);
		    delay(100);

		    uint8_t temp1[8];
			ICM_20948_read_mag_register(0x31, 1, temp1);
			/* Update data by reading through till ST2 register */
			ICM_20948_read_mag_register(0x11, 8, temp1);


			/* Wake on motion: 50 mg's (0.05 G) */
			ICM_20948_wakeOnMotionITEnable(true, 20, 2.2);
//			ICM_20948_sleepModeEnable(true);
			delay(400);

			IIC_Enable(false);


			/* Disable Systicks before going to sleep */
			EMU_EnterEM2(true);


			///////////////////////////////////////////
			/* When waking up, initialize everything */
			///////////////////////////////////////////

			CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

			IIC_Enable(true);
			BLE_power(true);
			BLE_rxtx_enable( true );
			CMU_ClockEnable(cmuClock_ADC0, true);

			_sleep = false;

			/* Setup IMU */
			ICM_20948_lowPowerModeEnter(false, false, false);
			ICM_20948_Init2();


			/* Timer for checking if IMU is idle */
			RTCDRV_Init();
			RTCDRV_AllocateTimer(&IMU_Idle_Timer);
			RTCDRV_StartTimer( IMU_Idle_Timer, rtcdrvTimerTypePeriodic, 2000, (RTCDRV_Callback_t)CheckIMUidle, NULL);

			/* Set interrupt to trigger every 100ms when the data is ready */
			IMU_MEASURING = true;
			ICM_20948_interruptEnable(true, false);

			GPIO_PinModeSet(gpioPortE, 11, gpioModePushPull, 0);

			appState = SYS_IDLE;

#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
			dbprintln("SLEEP");
#endif /* DEBUG_DBPRINT */
		}
			break;
			/***************************/
		case CALLIBRATE:
		{
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
			dbprintln("CALLIBRATE");
#endif /* DEBUG_DBPRINT */

			ICM_20948_accelGyroCalibrate(data.accelCal, data.gyroCal);
			ICM_20948_calibrate_mag(data.magOffset, data.magScale);

			appState = SYS_IDLE;
		}
			break;
			/***************************/
		} //switch

	} //while
} //main








/**************************************************************************//**
 * @brief GPIO Even IRQ for pushbuttons on even-numbered pins
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
	// Clear all even pin interrupt flags
	GPIO_IntClear(0x5555);
#if DEBUG_DBPRINTs == 1 /* DEBUG_DBPRINT */
	dbprint("Interrupt fired! 1");
#endif /* DEBUG_DBPRINT */

//	measure_send();

//	ICM_20948_interruptStatusRead(interruptStatus);
//
//	/* AND with mask to check only bit 4 of byte 1 */
//	if( (interruptStatus[0] & 8) == 8 ) /* If interrupt is bit WOM interrupt */
//	{
//		appState = SLEEP;
//	}else{

	/* If sensor is not trying to sleep, read sensors
	 * Otherwise bug when trying to sleep but still last interrupts occurring, sensor won't go to sleep
	 */
	if(appState != SLEEP)
	{
		appState = SENSORS_READ;
	}


//	}
}

/**************************************************************************//**
 * @brief GPIO Odd IRQ for pushbuttons on odd-numbered pins
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
	// Clear all odd pin interrupt flags
	GPIO_IntClear(0xAAAA);

#if DEBUG_DBPRINTs == 1 /* DEBUG_DBPRINT */
	dbprint("Interrupt fired! 2");
#endif /* DEBUG_DBPRINT */

//	appState = SENSORS_READ;
}




