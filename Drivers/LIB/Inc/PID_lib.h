/*
 * PID_lib.h
 *
 *  Created on: Dec 19, 2023
 *      Author: munir
 */

#ifndef LIB_INC_PID_LIB_H_
#define LIB_INC_PID_LIB_H_

#include "stm32f4xx_hal.h"

typedef struct
{
	double 	P, I, D, kp, ki, kd,
			error, last_error, int_error, max_mv, max_int_error,
			mv, last_us;
	_Bool aw;
}PID_HandleTypeDef;

void PID_set_konstanta (PID_HandleTypeDef *hpid, double kp, double ki, double kd);
void PID_set_max_value (PID_HandleTypeDef *hpid, double max_mv, double max_int_error);
double PID_calculate (PID_HandleTypeDef *hpid, double sp, double pv);
double PI_calculate (PID_HandleTypeDef *hpid, double sp, double pv);
double PD_calculate (PID_HandleTypeDef *hpid, double sp, double pv);
void counting_time (void);
double get_us (void);

#endif /* LIB_INC_PID_LIB_H_ */
