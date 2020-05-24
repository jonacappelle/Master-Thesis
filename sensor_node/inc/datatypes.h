/***************************************************************************//**
 * @file datatypes.h
 * @brief Datatypes
 * @version 1.0
 * @author Jona Cappelle
 * *****************************************************************************/


/*
 * datatypes.h
 *
 *  Created on: Mar 11, 2020
 *      Author: jonac
 */

#ifndef INC_DATATYPES_H_
#define INC_DATATYPES_H_

#include "stdint.h"

#define M_PI		3.14159265358979323846

typedef enum app_states {
	INIT,
	SENSORS_READ,
	BATT_READ,
	SLEEP,
	CALLIBRATE,
	SYS_IDLE
} APP_State_t;

typedef struct
{
	// IMU data
	float ICM_20948_gyro[3];
	float ICM_20948_accel[3];
	float ICM_20948_magn[3];
	float ICM_20948_euler_angles[3];

	// Bluetooth data
	uint8_t BLE_euler_angles[sizeof(float) * 3];
	uint8_t BLE_data[sizeof(float) * 3 + 5];

	// Calibration
	float accelCal[3];
	float gyroCal[3];
	float magOffset[3];
	float magScale[3];

	// Battery
	uint8_t batt[1];
	// float ICM_20948_magn_angle[1];

}MeasurementData_t;


#endif /* INC_DATATYPES_H_ */
