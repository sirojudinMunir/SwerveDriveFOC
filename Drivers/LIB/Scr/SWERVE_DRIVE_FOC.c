/*
 * FOC.c
 *
 *  Created on: Nov 22, 2024
 *      Author: munir
 */

#include "SWERVE_DRIVE_FOC.h"

extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

float LUT_SIN[360], LUT_COS[360];

void FOC_init (void){
	for (uint16_t i = 0; i < 360; i++){
		float rad = (float)i * DEG_2_RAD;
		LUT_SIN[i] = sin (rad);
		LUT_COS[i] = cos (rad);
	}
}

void BLDC_set_freq (BLDC_HandleTypeDef *hbldc, uint32_t freq)
{
	uint32_t prescaller = 1;
	prescaller = (float)TIMER_FREQ / (float)(freq * BLDC_PERIODE*1.5);
	if (prescaller > 0) prescaller--;
	if (hbldc->channel == BLDC_WHEELED){
		WHEELED_TIM->ARR = BLDC_PERIODE;
		WHEELED_TIM->PSC = prescaller;
	}
	else if (hbldc->channel == BLDC_STEERING){
		STEERING_TIM->ARR = BLDC_PERIODE;
		STEERING_TIM->PSC = prescaller;
	}
}

void BLDC_init (BLDC_HandleTypeDef *hbldc)
{
	hbldc->p_shift = 90;
	if (hbldc->channel == BLDC_WHEELED)
	{
		BLDC_set_freq (hbldc, 10000);
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_u], 0.9998);//0.9998
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_v], 0.9998);
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_w], 0.9998);
		dlpf_set_alpha(&hbldc->hdlpf_cmps, 0.002);

		PID_set_max_value(&hbldc->hpid_id, 4.0);//4 4000
		PID_set_max_value(&hbldc->hpid_iq, 4.0);
		PID_set_max_value(&hbldc->hpid_omega, hbldc->max_current);

		HAL_TIMEx_HallSensor_Start_IT (&htim5);
		HAL_TIM_PWM_Start(WHEELED_htim, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(WHEELED_htim, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(WHEELED_htim, TIM_CHANNEL_3);

		HAL_TIM_PWM_Start(WHEELED_htim, TIM_CHANNEL_4);
		WHEELED_TIM->CCR4 = BLDC_PWM_ADC_TRIG;

		HAL_ADCEx_InjectedStart_IT(WHEELED_hadc);

		WHEELED_IR2104 (ENABLE);
	}
	else if (hbldc->channel == BLDC_STEERING)
	{
		BLDC_set_freq (hbldc, 10000);
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_u], 0.95);//0.99925
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_v], 0.95);
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_w], 0.95);

		PID_set_max_value(&hbldc->hpid_id, 4);
		PID_set_max_value(&hbldc->hpid_iq, 4);
		PID_set_max_value(&hbldc->hpid_theta, hbldc->max_current);

		HAL_TIM_PWM_Start(STEERING_htim, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(STEERING_htim, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(STEERING_htim, TIM_CHANNEL_3);

		HAL_TIM_PWM_Start(STEERING_htim, TIM_CHANNEL_4);
		STEERING_TIM->CCR4 = BLDC_PWM_ADC_TRIG;

		HAL_ADCEx_InjectedStart_IT(STEERING_hadc);

		STEERING_IR2104 (ENABLE);
	}
}

void BLDC_get_current (BLDC_HandleTypeDef *hbldc)
{
	double iu, iv, iw;
#if 0
	uint32_t current_pwm[3];

	if (hbldc->channel == BLDC_WHEELED)
	{
		current_pwm[_u] = WHEELED_TIM->CCR1;
		current_pwm[_v] = WHEELED_TIM->CCR2;
		current_pwm[_w] = WHEELED_TIM->CCR3;
	}
	else if (hbldc->channel == BLDC_STEERING)
	{
		current_pwm[_u] = STEERING_TIM->CCR1;
		current_pwm[_v] = STEERING_TIM->CCR2;
		current_pwm[_w] = STEERING_TIM->CCR3;
	}
	iu = (double)hbldc->adc_buff[_u]*0.0040293040293;// * 0.5
	iv = (double)hbldc->adc_buff[_v]*0.0040293040293;
	iw = (double)hbldc->adc_buff[_w]*0.0040293040293;

	if (current_pwm[_u] == hbldc->peak_pwm)
	{
		hbldc->raw_current[_u] = iv+iw;
		hbldc->raw_current[_v] = -iv;
		hbldc->raw_current[_w] = -iw;
	}
	else if (current_pwm[_v] == hbldc->peak_pwm)
	{
		hbldc->raw_current[_u] = -iu;
		hbldc->raw_current[_v] = iu+iw;
		hbldc->raw_current[_w] = -iw;
	}
	else if (current_pwm[_w] == hbldc->peak_pwm)
	{
		hbldc->raw_current[_u] = -iu;
		hbldc->raw_current[_v] = -iv;
		hbldc->raw_current[_w] = iu+iv;
	}
#else
	if (hbldc->channel == BLDC_WHEELED){
		hbldc->adc_buff[_u] = HAL_ADCEx_InjectedGetValue(WHEELED_hadc, ADC_INJECTED_RANK_1);
		hbldc->adc_buff[_v] = HAL_ADCEx_InjectedGetValue(WHEELED_hadc, ADC_INJECTED_RANK_2);
		hbldc->adc_buff[_w] = HAL_ADCEx_InjectedGetValue(WHEELED_hadc, ADC_INJECTED_RANK_3);
	}
	else if (hbldc->channel == BLDC_STEERING){
		hbldc->adc_buff[_u] = HAL_ADCEx_InjectedGetValue(STEERING_hadc, ADC_INJECTED_RANK_1);
		hbldc->adc_buff[_v] = HAL_ADCEx_InjectedGetValue(STEERING_hadc, ADC_INJECTED_RANK_2);
		hbldc->adc_buff[_w] = HAL_ADCEx_InjectedGetValue(STEERING_hadc, ADC_INJECTED_RANK_3);
	}
	switch (hbldc->peak_volt){
	case _u:
		iv = (double)hbldc->adc_buff[_v]*ADC_2_CURRENT;
		iw = (double)hbldc->adc_buff[_w]*ADC_2_CURRENT;
		hbldc->raw_current[_u] = iv+iw;
		hbldc->raw_current[_v] = -iv;
		hbldc->raw_current[_w] = -iw;
		break;
	case _v:
		iu = (double)hbldc->adc_buff[_u]*ADC_2_CURRENT;
		iw = (double)hbldc->adc_buff[_w]*ADC_2_CURRENT;
		hbldc->raw_current[_u] = -iu;
		hbldc->raw_current[_v] = iu+iw;
		hbldc->raw_current[_w] = -iw;
		break;
	case _w:
		iu = (double)hbldc->adc_buff[_u]*ADC_2_CURRENT;
		iv = (double)hbldc->adc_buff[_v]*ADC_2_CURRENT;
		hbldc->raw_current[_u] = -iu;
		hbldc->raw_current[_v] = -iv;
		hbldc->raw_current[_w] = iu+iv;
		break;
	}
#endif
}

void BLDC_get_current_filter (BLDC_HandleTypeDef *hbldc)
{
	dlpf_get_result (&hbldc->hdlpf_current_filt[_u], hbldc->raw_current[_u]);
	dlpf_get_result (&hbldc->hdlpf_current_filt[_v], hbldc->raw_current[_v]);
	dlpf_get_result (&hbldc->hdlpf_current_filt[_w], hbldc->raw_current[_w]);
}

void BLDC_clark_park_trans (BLDC_HandleTypeDef *hbldc, float deg, float ia, float ib, float ic)
{
	float alpha, beta;
	if (deg < 0) deg = deg + 360.0;
	uint16_t deg_index = (int16_t)deg % 360;

	alpha = ia;
	beta = (ib - ic) * ONE_OVER_SQRT3;

	hbldc->id_result = LUT_COS[deg_index] * alpha + LUT_SIN[deg_index] * beta;
	hbldc->iq_result = LUT_COS[deg_index] * beta - LUT_SIN[deg_index] * alpha;
}

void BLDC_inv_clark_park_trans (BLDC_HandleTypeDef *hbldc, float deg, float d, float q)
{
	float alpha, beta;
	if (deg < 0) deg = deg + 360.0;
	uint16_t deg_index = (int16_t)deg % 360;

	alpha = d * LUT_COS[deg_index] - q * LUT_SIN[deg_index];
	beta = d * LUT_SIN[deg_index] + q * LUT_COS[deg_index];

	hbldc->ia = alpha;
	hbldc->ib = -0.5 * alpha + SQRT3_OVER_TWO * beta;
	hbldc->ic = -0.5 * alpha - SQRT3_OVER_TWO * beta;
}

//============================================================================================

void BLDC_spwm (BLDC_HandleTypeDef *hbldc)
{
	uint32_t peak_pwm_temp = 0;

	hbldc->spwm[_u] = (hbldc->ia * BLDC_PWM_GAIN) + BLDC_PWM_CENTER;
	hbldc->spwm[_v] = (hbldc->ib * BLDC_PWM_GAIN) + BLDC_PWM_CENTER;
	hbldc->spwm[_w] = (hbldc->ic * BLDC_PWM_GAIN) + BLDC_PWM_CENTER;

	for (uint8_t i = 0; i < 3; i++)
	{
		if (hbldc->spwm[i] > BLDC_PWM_MAX) hbldc->spwm[i] = BLDC_PWM_MAX;
		if (hbldc->spwm[i] < BLDC_PWM_MIN) hbldc->spwm[i] = BLDC_PWM_MIN;
		if (peak_pwm_temp < hbldc->spwm[i]) {
			peak_pwm_temp = hbldc->spwm[i];
			hbldc->peak_volt = i;
		}
	}
	hbldc->peak_pwm = peak_pwm_temp;

	if (hbldc->channel == BLDC_WHEELED)
	{
		WHEELED_TIM->CCR1 = hbldc->spwm[_u];
		WHEELED_TIM->CCR2 = hbldc->spwm[_v];
		WHEELED_TIM->CCR3 = hbldc->spwm[_w];
	}
	else if (hbldc->channel == BLDC_STEERING)
	{
		STEERING_TIM->CCR1 = hbldc->spwm[_u];
		STEERING_TIM->CCR2 = hbldc->spwm[_v];
		STEERING_TIM->CCR3 = hbldc->spwm[_w];
	}
}

void BLDC_current_control (BLDC_HandleTypeDef *hbldc, double id, double iq, double theta)
{
	BLDC_get_current_filter(hbldc);
	BLDC_clark_park_trans(hbldc, theta,
			hbldc->hdlpf_current_filt[_u].result,
			hbldc->hdlpf_current_filt[_v].result,
			hbldc->hdlpf_current_filt[_w].result);
	PID_calculate(&hbldc->hpid_id, id, hbldc->id_result);
	PID_calculate(&hbldc->hpid_iq, iq, hbldc->iq_result);
	BLDC_inv_clark_park_trans(hbldc, theta, hbldc->hpid_id.mv, hbldc->hpid_iq.mv);
	BLDC_spwm(hbldc);
}


