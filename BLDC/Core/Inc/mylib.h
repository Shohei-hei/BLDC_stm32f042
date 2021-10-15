/*
 * mylib.h
 *
 *  Created on: 2021/05/04
 *      Author: seven
 */

#ifndef INC_MYLIB_H_
#define INC_MYLIB_H_

//#include <math.h>
#include <stdint.h>

#define PI			(3.1415926535)
#define TWOPI		(2*3.1415926535)
#define SQRT_2p3	0.81649658092
#define SQRT_2		1.41421356237
#define SQRT_3		1.73205080757
#define SQRT_6		2.44948974278
#define SQRT_2p3	0.81649658092			//sqrt(2/3)

typedef struct{
	float SIN0;				//sinθの値
	float COS0;				//cosθの値

	int16_t Vu;			//U相電圧指令
	int16_t Vv;			//V相電圧指令
	int16_t Vw;			//W相電圧指令

	int16_t Va;			//α軸電圧指令
	int16_t Vb;			//β軸電圧指令

	int16_t Vd;			//d軸電圧指令
	int16_t Vq;			//q軸電圧指令
} m_carrier_t;

typedef struct{
	int16_t target;		//目標値
	int16_t deff;		//偏差
	int16_t kp;			//比例定数
	int16_t ki;			//積分定数
	int16_t integral;	//前回積分量
}pi_param_t;

float my_sin(float x);
float my_cos(float x);
float my_sqrt(float x);
void dq2uvw(m_carrier_t *a, int16_t angle);
int16_t pi_control(pi_param_t *a, int16_t meas);
void uvw2dq(m_carrier_t *a);

#endif /* INC_MYLIB_H_ */
