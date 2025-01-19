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
	float 	P, I, D, kp, ki, kd,
			error, last_error, int_error, max_mv, max_int_error,
			mv, last_us, ts;
}PID_HandleTypeDef;

void PID_set_time_sampling (PID_HandleTypeDef *hpid, float us);
void PID_set_konstanta (PID_HandleTypeDef *hpid, float kp, float ki, float kd);
void PID_set_max_value (PID_HandleTypeDef *hpid, float max_mv);
float PID_calculate (PID_HandleTypeDef *hpid, float sp, float pv);
float PI_calculate (PID_HandleTypeDef *hpid, float sp, float pv);
float PD_calculate (PID_HandleTypeDef *hpid, float sp, float pv);
void counting_time (void);
float get_us (void);

#endif /* LIB_INC_PID_LIB_H_ */
