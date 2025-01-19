/*
 * SWERVE_DRIVE_FOC.h
 *
 *  Created on: Nov 22, 2024
 *      Author: munir
 */

#ifndef LIB_INC_FOC_H_
#define LIB_INC_FOC_H_

#include "main.h"
#include "PID_lib.h"
#include "DLPF_lib.h"
#include "math.h"

#define DEG_2_RAD 				0.01745329251994f
#define RAD_2_DEG				57.2957795130823208f
#define ONE_OVER_SQRT3 			0.5773502691896257f
#define SQRT3_OVER_TWO 			0.8660254037844386f

#define ADC_2_CURRENT 			0.0040293040293

#define TIMER_FREQ				168000000
#define BLDC_PERIODE			4200

#define BLDC_PWM_FREQ 			10000.0f
#define BLDC_CURRENT_CTRL_LOOP	10
#define BLDC_CURRENT_CTRL_TS	(1000000.0f/BLDC_PWM_FREQ)
#define BLDC_CTRL_TS			(BLDC_CURRENT_CTRL_TS*BLDC_CURRENT_CTRL_LOOP)

#define BLDC_PWM_CENTER 		2047
#define BLDC_PWM_MAX 			4100
#define BLDC_PWM_MIN 			100
#define BLDC_PWM_GAIN 			500
#define BLDC_PWM_ADC_TRIG 		4199

#define NORMAL 	1
#define SWAP 	0

#define ENABLE 	1
#define DISABLE 0

#define BLDC_CHANNEL NORMAL /*NORAML / SWAP*/

#if BLDC_CHANNEL

#define STEERING_TIM TIM1
#define STEERING_htim (TIM_HandleTypeDef*)&htim1
#define STEERING_ADC ADC3
#define STEERING_hadc (ADC_HandleTypeDef*)&hadc3
#define STEERING_IR2104(x) \
		M1_EN_U_GPIO_Port->BSRR = M1_EN_U_Pin << ((x==ENABLE)?0U:16U); \
		M1_EN_V_GPIO_Port->BSRR = M1_EN_V_Pin << ((x==ENABLE)?0U:16U); \
		M1_EN_W_GPIO_Port->BSRR = M1_EN_W_Pin << ((x==ENABLE)?0U:16U)

#define WHEELED_TIM TIM8
#define WHEELED_htim (TIM_HandleTypeDef*)&htim8
#define WHEELED_ADC ADC2
#define WHEELED_hadc (ADC_HandleTypeDef*)&hadc2
#define WHEELED_IR2104(x) \
		M2_EN_U_GPIO_Port->BSRR = M2_EN_U_Pin << ((x==ENABLE)?0U:16U); \
		M2_EN_V_GPIO_Port->BSRR = M2_EN_V_Pin << ((x==ENABLE)?0U:16U); \
		M2_EN_W_GPIO_Port->BSRR = M2_EN_W_Pin << ((x==ENABLE)?0U:16U)

#else

#define STEERING_TIM TIM8
#define STEERING_htim (TIM_HandleTypeDef*)&htim8
#define STEERING_ADC ADC2
#define STEERING_hadc (ADC_HandleTypeDef*)&hadc2
#define STEERING_IR2104(x) \
		M2_EN_U_GPIO_Port->BSRR = M2_EN_U_Pin << ((x==ENABLE)?0U:16U); \
		M2_EN_V_GPIO_Port->BSRR = M2_EN_V_Pin << ((x==ENABLE)?0U:16U); \
		M2_EN_W_GPIO_Port->BSRR = M2_EN_W_Pin << ((x==ENABLE)?0U:16U)

#define WHEELED_TIM TIM1
#define WHEELED_htim (TIM_HandleTypeDef*)&htim1
#define WHEELED_ADC ADC3
#define WHEELED_hadc (ADC_HandleTypeDef*)&hadc3
#define WHEELED_IR2104(x) \
		M1_EN_U_GPIO_Port->BSRR = M1_EN_U_Pin << ((x==ENABLE)?0U:16U); \
		M1_EN_V_GPIO_Port->BSRR = M1_EN_V_Pin << ((x==ENABLE)?0U:16U); \
		M1_EN_W_GPIO_Port->BSRR = M1_EN_W_Pin << ((x==ENABLE)?0U:16U)

#endif

typedef enum
{
	BLDC_WHEELED = 0, BLDC_STEERING = 1
}bldc_channel_t;

typedef enum
{
	_u, _v, _w
}bldc_output_t;

typedef struct
{
	bldc_channel_t channel;
	bldc_output_t peak_volt;
	PID_HandleTypeDef hpid_id, hpid_iq, hpid_omega, hpid_theta;
	DLPF_HandleTypeDef hdlpf_current_filt[3], hdlpf_cmps;
	_Bool 	state;
	uint32_t adc_buff[3], spwm[3], peak_pwm, c_loop, zero_det_t;
	int8_t 	dir, last_dir;
	uint8_t hall_sector;
	float 	sector_theta, last_sector_theta, new_sector_theta,
	 	 	theta, raw_current[3], id_result, iq_result,
			p_shift, rotor_offset, angle_offset,
			ia, ib, ic, max_current,
			rpm_abs, rpm, cmps,
			angle_estimation, last_angle_estimation;
}BLDC_HandleTypeDef;


void FOC_init (void);
void BLDC_init (BLDC_HandleTypeDef *hbldc);
void BLDC_get_current_test (BLDC_HandleTypeDef *hbldc);
void BLDC_get_current (BLDC_HandleTypeDef *hbldc);
void BLDC_get_current_filter (BLDC_HandleTypeDef *hbldc);
void BLDC_clark_park_trans (BLDC_HandleTypeDef *hbldc, float deg, float ia, float ib, float ic);
void BLDC_inv_clark_park_trans (BLDC_HandleTypeDef *hbldc, float deg, float d, float q);
void BLDC_spwm (BLDC_HandleTypeDef *hbldc);
void BLDC_current_control (BLDC_HandleTypeDef *hbldc, double id, double iq, double theta);

#endif /* LIB_INC_FOC_H_ */
