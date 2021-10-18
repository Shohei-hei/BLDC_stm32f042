/*
 * inverter.c
 *
 *  Created on: May 6, 2021
 *      Author: seven
 */

#include "inverter.h"

void Set_inverter(m_carrier_t value, uint32_t max){
	uint32_t u, v, w;

	u = (uint32_t)((value.Vu)*max/2) + max/2;
	v = (uint32_t)((value.Vv)*max/2) + max/2;
	w = (uint32_t)((value.Vw)*max/2) + max/2;

/*	u = max*9/10;
	v = max*5/10;
	w = max*1/10;
*/
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, u);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, v);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, w);
}
