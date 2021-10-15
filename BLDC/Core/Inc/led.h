/*
 * led.h
 *
 *  Created on: Oct 7, 2019
 *      Author: sh0wh3y
 */

#ifndef INC_LED_H_
#define INC_LED_H_

//include
#include <stdint.h>
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_conf.h"
#include "stm32f0xx_it.h"
#include "main.h"

//typedef

//define
#define ch_LED1 TIM_CHANNEL_1

#define LED1_off  	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET)
#define LED1_on  	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET)
#define LED1(x)  	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,(GPIO_PinState)!x)

#define LED1PWM(x)	LED_pwm(ch_LED1, x);

//macro

//export variables
extern TIM_HandleTypeDef htim16;

//function
void LEDpwm_Init(TIM_HandleTypeDef htim);
void LED_pwm(uint32_t ledch, uint32_t pluse);

#endif /* INC_LED_H_ */
