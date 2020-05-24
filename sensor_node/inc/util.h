/***************************************************************************//**
 * @file util.h
 * @brief Handy functions
 * @version 1.0
 * @author Jona Cappelle
 * *****************************************************************************/

/*
 * util.h
 *
 *  Created on: Mar 11, 2020
 *      Author: jonac
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include "stdint.h"
#include "stdbool.h"

bool Check_Equal(uint8_t *arr1, uint8_t *arr2, uint8_t length);
void blink(uint8_t times);

#endif /* INC_UTIL_H_ */
