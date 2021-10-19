/*
 * mylib.c
 *
 *  Created on: 2021/05/04
 *      Author: seven
 */

#include "mylib.h"
#include "stdint.h"

float my_sin(float x){
	float result = x, t = x;
	int8_t i = 0;

	for(i=1;i<11;i++){
		t *= -(x*x)/((i*2)*(i*2 + 1));
		result += t;
	}
	if(result>1) result = 1;
	else if(result<-1) result = -1;
	else;
	return result;
}

float my_cos(float x){		// -PI =< x =< PI
	float result = 1, t = 1;
	int i = 0;

	if(x >= PI){
		x -= TWOPI;
	}

	for(i=1;i<11;i++){
		t *= -(x*x)/((i*2)*(i*2 - 1));
		result += t;
	}

	if(result>1) result = 1;
	else if(result<-1) result = -1;
	else;
	return result;
}

float my_sqrt(float x){
	float y=1, z=0;
	int i;

	if(x == 0){
		y = 0;
	}else{
		for(i=0;i<20;i++){
			z = x/y;
			y = (y+z)/2;
		}
	}
	return y;
}

//HIP変調
void HIP(m_carrier_t *a, float *Vmax, float *Vmin){
}

//ts エンコーダ値更新周期[s], vm モータ電圧[mV], Fe1 PWMキャリア周波数[Hz]
void dq2uvw(m_carrier_t *a, int16_t angle){
	float rad, Va, Vb, sin0, cos0;

	rad = (float)angle*TWOPI/3600;

	sin0 = my_sin(rad);
	cos0 = my_cos(rad);

	Va = cos0*a->Vd - sin0*a->Vq;
	Vb = sin0*a->Vd + cos0*a->Vq;

	a->Vu = SQRT_2p3*Va;
	a->Vv = SQRT_2p3*((-1)*Va/2 + SQRT_3*Vb/2);
	a->Vw = SQRT_2p3*((-1)*Va/2 - SQRT_3*Vb/2);
}

int16_t pi_control(pi_param_t *a, int16_t meas){
	int16_t result;
	int16_t p_val;
	int16_t i_val;

	a->deff = a->target - meas;		//偏差=目標値-計測値
	p_val = a->deff*a->kp;
	i_val = a->integral + a->deff*a->ki;
	a->integral = i_val;

	result = p_val + i_val;

	return result;
}

void uvw2dq(m_carrier_t *a){

}
