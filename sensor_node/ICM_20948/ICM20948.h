/***************************************************************************//**
 * @file ICM20948.h
 * @brief Advanced funcions to control and read data from the ICM-20948
 * @details Partly copied from silabs thunderboard code for ICM-20689, partly self written
 * @version 1.0
 * @author Jona Cappelle
 * *****************************************************************************/


/*
 * ICM20948.h
 *
 *  Created on: Nov 12, 2019
 *      Author: jonac
 */

#ifndef ICM_20948_ICM20948_H_
#define ICM_20948_ICM20948_H_

#include <stdint.h>
#include <stdbool.h>
/*********************************/

/*********************************/

void ICM_20948_power (bool enable);

void ICM_20948_Init ();
void ICM_20948_Init2();
void ICM_20948_Init_SPI ();
void ICM_20948_enable_SPI(bool enable);

void ICM_20948_chipSelectSet ( bool enable );
void ICM_20948_bankSelect ( uint8_t bank );

uint8_t ICM_20948_read ( uint16_t addr );
void ICM_20948_registerRead(uint16_t addr, int numBytes, uint8_t *data);


void ICM_20948_printAllData ();

void ICM_20948_write ( uint16_t addr, uint8_t data );

uint32_t ICM_20948_sleepModeEnable ( bool enable );
uint32_t ICM_20948_reset(void);


uint32_t ICM_20948_gyroDataRead(float *gyro);
uint32_t ICM_20948_gyroResolutionGet(float *gyroRes);
uint32_t ICM_20948_gyroFullscaleSet(uint8_t gyroFs);
uint32_t ICM_20948_gyroBandwidthSet(uint8_t gyroBw);
float ICM_20948_gyroSampleRateSet(float sampleRate);

uint32_t ICM_20948_accelDataRead(float *accel);
uint32_t ICM_20948_accelResolutionGet(float *accelRes);
uint32_t ICM_20948_accelFullscaleSet(uint8_t accelFs);
uint32_t ICM_20948_accelBandwidthSet(uint8_t accelBw);
float ICM_20948_accelSampleRateSet(float sampleRate);


uint32_t ICM_20948_interruptEnable(bool dataReadyEnable, bool womEnable);
uint32_t ICM_20948_interruptStatusRead(uint32_t *intStatus);
uint32_t ICM_20948_wakeOnMotionITEnable(bool enable, uint8_t womThreshold, float sampleRate);
uint32_t ICM_20948_latchEnable(bool enable);


uint32_t ICM_20948_sampleRateSet(float sampleRate);

uint32_t ICM_20948_lowPowerModeEnter(bool enAccel, bool enGyro, bool enTemp);
uint32_t ICM_20948_sensorEnable(bool accel, bool gyro, bool temp);
uint32_t ICM_20948_cycleModeEnable(bool enable);

/* Magnetometer functions */
void ICM_20948_set_mag_transfer(bool read);
void ICM_20948_read_mag_register(uint8_t addr, uint8_t numBytes, uint8_t *data);
void ICM_20948_write_mag_register(uint8_t addr, uint8_t data);
uint32_t ICM_20948_set_mag_mode(uint8_t magMode);
void ICM_20948_magRawDataRead(float *raw_magn);
void ICM_20948_magDataRead(float *magn);
uint32_t ICM_20948_reset_mag(void);

void ICM_20948_registerWrite(uint16_t addr, uint8_t data);

/* Embedded 2 test */
void ICM_20948_magn_to_angle(float *magn, float *angle);

///////////////////////////////////////////////////////////////////////////////
/////////////////             CALIBRATION        //////////////////////////////
///////////////////////////////////////////////////////////////////////////////
uint32_t ICM_20948_accelGyroCalibrate(float *accelBiasScaled, float *gyroBiasScaled);
uint32_t ICM_20948_gyroCalibrate( float *gyroBiasScaled );
uint32_t ICM_20948_min_max_mag( int16_t *minMag, int16_t *maxMag );
bool ICM_20948_calibrate_mag( float *offset, float *scale );



#endif /* ICM_20948_ICM20948_H_ */
