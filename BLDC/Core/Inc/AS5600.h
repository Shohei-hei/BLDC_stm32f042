/*
 * AS5600.h
 *
 *  Created on: Mar 28, 2021
 *      Author: seven
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

/* include */
#include <stdint.h>
#include <stdio.h>
#include "main.h"

/* extern */
extern I2C_HandleTypeDef hi2c1;

/* typedef */

/* function */
void AS5600_Write(uint16_t x);
uint16_t AS5600_mem_read(uint16_t mem_addr);

/* define */
#define AS5600_addr				(0x36<<1)
#define AS5600_ZMCO				0x00
#define AS5600_ZPOS_U			0x01
#define AS5600_ZPOS_L			0x02
#define AS5600_MPOS_U			0x03
#define AS5600_MPOS_L			0x04
#define AS5600_MANG_U			0x05
#define AS5600_MANG_L			0x06
#define AS5600_CONF_U			0x07
#define AS5600_CONF_L			0x08
#define AS5600_RAWANGLE_U		0x0C
#define AS5600_RAWANGLE_L		0x0D
#define AS5600_ANGLE_U			0x0E
#define AS5600_ANGLE_L			0x0F
#define AS5600_STATUS			0x0B
#define AS5600_AGC				0x1A
#define AS5600_MAGNITUDE_U		0x1B
#define AS5600_MAGNITUDE_L		0X1C
#define AS5600_BURN				0xFF

#define def_AS5600_Read_Raw		AS5600_mem_read(AS5600_RAWANGLE_U);

#endif /* INC_AS5600_H_ */
