/*
 * AS5600.c
 *
 *  Created on: Mar 28, 2021
 *      Author: seven
 */

#include "AS5600.h"

void AS5600_Write(uint16_t x){
	uint8_t Txbuffer[2] = {(x>>8)&0x00FF, x&0x00FF};

	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)AS5600_addr, Txbuffer, 2, 1000) != HAL_OK){
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF){
			Error_Handler();
		}
	}
}

uint16_t AS5600_mem_read(uint16_t mem_addr){
	uint8_t Rxbuffer[2] = {};
	uint16_t result = 0;

	while(HAL_I2C_Mem_Read(&hi2c1, (uint16_t)AS5600_addr +1, mem_addr, 1, Rxbuffer, 2, 1000) != HAL_OK){
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF){
			Error_Handler();
		}
	}

	result = Rxbuffer[0] & 0xFF;
	result <<= 8;
	result |= Rxbuffer[1];

	return (result);
}
