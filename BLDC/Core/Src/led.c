/*
 * led.c
 *
 *  Created on: Oct 7, 2019
 *      Author: 翔平
 */

#include "led.h"

static uint32_t g_st_period = 1000;

void LEDpwm_Init(TIM_HandleTypeDef htim){
	g_st_period = htim.Init.Period;
}

void LED_pwm(uint32_t ledch, uint32_t pulse){
	pulse *= g_st_period/1000+1;
	__HAL_TIM_SET_COMPARE(&htim16, ledch, pulse);
}
