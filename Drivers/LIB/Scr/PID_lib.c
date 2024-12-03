/*
 * PID_lib.c
 *
 *  Created on: Dec 19, 2023
 *      Author: munir
 */

#include "PID_lib.h"

uint32_t t_count;
double ts;

//============================================================================================

/**
  * @brief  Add value to PID constants
  * @param  hpid 	pointer to PID_HandleTypeDef structure that contains
  * 		kp 	  	proportional constant
  * 		ki 	  	integral constant
  * 		kd 	  	differential constant
  * @retval none
  */
void PID_set_konstanta (PID_HandleTypeDef *hpid, double kp, double ki, double kd)
{
	hpid->kp = kp;
	hpid->ki = ki;
	hpid->kd = kd;
}

//============================================================================================

/**
  * @brief  Set maximum value
  * @param  hpid 	pointer to PID_HandleTypeDef structure that contains
  * 		max_mv	maximum manipulated value / output from PID, PI, or PD control
  * @retval none
  */
void PID_set_max_value (PID_HandleTypeDef *hpid, double max_mv)
{
	hpid->max_mv = max_mv;
}

//============================================================================================

/**
  * @brief  PID Control calculation
  * @param  hpid 	pointer to PID_HandleTypeDef structure that contains
  * 		sp		set point
  * 		pv		previous value
  * @retval mv		manipulated value / output PID Control
  */
double PID_calculate (PID_HandleTypeDef *hpid, double sp, double pv)
{
	double result = 0, integral_temp, dt = 0.0001;

	double us = get_us ();
	hpid->ts = us - hpid->last_us;
	hpid->last_us = us;

	hpid->error = sp - pv;
	hpid->P = hpid->error * hpid->kp;
	hpid->D = ((hpid->error - hpid->last_error) / dt) * hpid->kd;

	integral_temp = hpid->int_error + (hpid->error * dt);
	hpid->I = integral_temp * hpid->ki;

	result = hpid->P + hpid->I + hpid->D;

	if (result >= -hpid->max_mv && result <= hpid->max_mv){
		hpid->int_error = integral_temp;
	}
	else{
		if (result > hpid->max_mv){
			result = hpid->max_mv;
		}
		else if (result < -hpid->max_mv){
			result = -hpid->max_mv;
		}
	}

	hpid->mv = result;
	hpid->last_error = hpid->error;

	return result;
}

//============================================================================================

/**
  * @brief  PI Control calculation
  * @param  hpid 	pointer to PID_HandleTypeDef structure that contains
  * 		sp		set point
  * 		pv		previous value
  * @retval mv		manipulated value / output PI Control
  */
double PI_calculate (PID_HandleTypeDef *hpid, double sp, double pv)
{
	double result;
	hpid->error = sp - pv;
	hpid->P = hpid->error * hpid->kp;
	hpid->int_error += hpid->error;
	if (hpid->int_error > hpid->max_int_error) hpid->int_error = hpid->max_int_error;
	else if (hpid->int_error < -hpid->max_int_error) hpid->int_error = -hpid->max_int_error;
	hpid->I = hpid->int_error * hpid->ki;
	hpid->last_error = hpid->error;
	result = hpid->P + hpid->I;
	if (result > hpid->max_mv) result = hpid->max_mv;
	else if (result < -hpid->max_mv) result = -hpid->max_mv;
	hpid->mv = result;
	return result;
}

//============================================================================================

/**
  * @brief  PD Control calculation
  * @param  hpid 	pointer to PID_HandleTypeDef structure that contains
  * 		sp		set point
  * 		pv		previous value
  * @retval mv		manipulated value / output PD Control
  */
double PD_calculate (PID_HandleTypeDef *hpid, double sp, double pv)
{
	double result;
	hpid->error = sp - pv;
	hpid->P = hpid->error * hpid->kp;
	hpid->D = (hpid->error - hpid->last_error) * hpid->kd;
	hpid->last_error = hpid->error;
	result = hpid->P + hpid->D;
	if (result > hpid->max_mv) result = hpid->max_mv;
	else if (result < -hpid->max_mv) result = -hpid->max_mv;
	hpid->mv = result;
	return result;
}

//============================================================================================

void counting_time (void)
{
	t_count++;
	if (t_count > 999999) t_count = 0;
}

double get_us (void)
{
	double us;
	us = (double)(TIM4->CNT + t_count*10000);// * 0.00595238095238095;
	return us;
}

//============================================================================================

