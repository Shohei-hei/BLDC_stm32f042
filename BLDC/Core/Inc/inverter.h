/*
 * inverter.h
 *
 *  Created on: May 6, 2021
 *      Author: seven
 */

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

/* include */
#include "main.h"
#include "mylib.h"

/* define */
#define MD_EN_OFF  	HAL_GPIO_WritePin(MD_EN_GPIO_Port,MD_EN_Pin,GPIO_PIN_RESET)
#define MD_EN_ON  	HAL_GPIO_WritePin(MD_EN_GPIO_Port,MD_EN_Pin,GPIO_PIN_SET)
#define MD_EN(x)  	HAL_GPIO_WritePin(MD_EN_GPIO_Port,MD_EN_Pin,(GPIO_PinState)x)

/* extern */
extern TIM_HandleTypeDef htim1;

/* typedef */

/* function */
void Set_inverter(m_carrier_t value, uint32_t max);


#endif /* INC_INVERTER_H_ */
