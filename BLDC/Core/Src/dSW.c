/*
 * dSW.c
 *
 *  Created on: Jul 3, 2021
 *      Author: seven
 */
#include "dSW.h"

int16_t dSW_state(void){
	int16_t result = 0;

	result |= HAL_GPIO_ReadPin(dSW4_GPIO_Port,dSW4_Pin);
	result <<= 1;
	result |= HAL_GPIO_ReadPin(dSW3_GPIO_Port,dSW3_Pin);
	result <<= 1;
	result |= HAL_GPIO_ReadPin(dSW2_GPIO_Port,dSW2_Pin);
	result <<= 1;
	result |= HAL_GPIO_ReadPin(dSW1_GPIO_Port,dSW1_Pin);
	result <<= 1;

	return result;
}
