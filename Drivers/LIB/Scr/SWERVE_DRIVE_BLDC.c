/*
 * SWERVE_DRIVE_BLDC.c
 *
 *  Created on: Dec 2, 2024
 *      Author: munir
 */

#include "SWERVE_DRIVE_BLDC.h"
#include "SWERVE_DRIVE_FOC.h"
#include "FLASH_lib.h"
#include "TASK_COMMAND.h"
#include "MAGNETIC_SENSOR_AS5048A.h"

extern ADC_HandleTypeDef hadc1;
extern float raw_angle;
extern BLDC_HandleTypeDef WHEELED_handler, STEERING_handler;
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

extern float data_angle, data_rpm;

extern _Bool 	read_ready,
				wheel_state,
				steer_call_ready;

extern _Bool 	read_ready;

DLPF_HandleTypeDef battery_read;

int16_t 	hall_sens_count = 0;

uint32_t 	usb_tx_t,
			adc_battery;

float 		angle_sector,
			angle_sens,
			steer_angle_offset = 0;



void wheeled_motor_set_speed (float speed){
	data_rpm = speed;
}

void steering_motor_set_angle (float angle){
	data_angle = angle;
}

float wheeled_motor_get_speed_setpoint (){
	return data_rpm;
}

float steering_motor_get_angle_set_point (){
	return data_angle;
}

void BLDC_beep (BLDC_HandleTypeDef *hbldc, uint32_t freq, uint32_t time_delay)
{
	uint32_t temp_psc;
	if (hbldc->channel == BLDC_STEERING) temp_psc = STEERING_TIM->PSC;
	else temp_psc = WHEELED_TIM->PSC;
	uint32_t prescaller;
	if (freq != 0)
	{
		if (freq < 100) freq = 100;
		else if (freq > 20000) freq = 20000;
		prescaller = TIMER_FREQ / (freq * BLDC_PERIODE * 2);
		if (hbldc->channel == BLDC_STEERING)
		{
			STEERING_TIM->PSC = prescaller;
		}
		else
		{
			WHEELED_TIM->PSC = prescaller;
		}
		BLDC_inv_clark_park_trans (hbldc, 0, 0, 0.5);
		BLDC_spwm (hbldc);
	}
	else
	{
		BLDC_inv_clark_park_trans (hbldc, 0, 0, 0);
		BLDC_spwm (hbldc);
	}
	osDelay(time_delay);
	BLDC_inv_clark_park_trans (hbldc, 0, 0, 0);
	BLDC_spwm (hbldc);
	if (hbldc->channel == BLDC_STEERING) STEERING_TIM->PSC = temp_psc;
	else WHEELED_TIM->PSC = temp_psc;
}

//============================================================================================

void BLDC_get_sector (BLDC_HandleTypeDef *hbldc)
{
	uint8_t raw_hall_data;
//	raw_hall_data = ((GPIOA->IDR & GPIO_PIN_15)>>15) | ((GPIOB->IDR & GPIO_PIN_3)>>2) | ((GPIOB->IDR & GPIO_PIN_10)>>8);
	raw_hall_data = GPIOA->IDR & 0x07;
	switch (raw_hall_data)
	{
	case 1:
		if (hbldc->sector_theta == 120) 	 hbldc->dir = 1;
		else if (hbldc->sector_theta == 240) hbldc->dir = -1;
		else hbldc->dir = 0;
		hbldc->sector_theta = 180;
	break;
	case 5:
		if (hbldc->sector_theta == 60) 	 	 hbldc->dir = 1;
		else if (hbldc->sector_theta == 180) hbldc->dir = -1;
		else hbldc->dir = 0;
		hbldc->sector_theta = 120;
	break;
	case 4:
		if (hbldc->sector_theta == 0) 	 	 hbldc->dir = 1;
		else if (hbldc->sector_theta == 120) hbldc->dir = -1;
		else hbldc->dir = 0;
		hbldc->sector_theta = 60;
	break;
	case 6:
		if (hbldc->sector_theta == 300) 	 hbldc->dir = 1;
		else if (hbldc->sector_theta == 60)  hbldc->dir = -1;
		else hbldc->dir = 0;
		hbldc->sector_theta = 0;
	break;
	case 2:
		if (hbldc->sector_theta == 240) 	 hbldc->dir = 1;
		else if (hbldc->sector_theta == 0) 	 hbldc->dir = -1;
		else hbldc->dir = 0;
		hbldc->sector_theta = 300;
	break;
	case 3:
		if (hbldc->sector_theta == 180) 	 hbldc->dir = 1;
		else if (hbldc->sector_theta == 300) hbldc->dir = -1;
		else hbldc->dir = 0;
		hbldc->sector_theta = 240;
	break;
	}
}

void BLDC_rpm_sens (BLDC_HandleTypeDef *hbldc)
{
	if (hbldc->dir == 1)
	{
		hbldc->rpm = hbldc->rpm_abs;
		hall_sens_count++;
	}
	else if (hbldc->dir == -1)
	{
		hbldc->rpm = -hbldc->rpm_abs;
		hall_sens_count--;
	}
}

//============================================================================================

void BLDC_get_rpm (BLDC_HandleTypeDef *hbldc)
{
	if (hbldc->channel == BLDC_WHEELED)
	{
		hbldc->rpm_abs = 60.0 / ((float)TIM5->CCR1 * 0.00012);
	}
	else if (hbldc->channel == BLDC_STEERING)
	{
		//----------------------
	}
}

//============================================================================================

void BLDC_speed_control (void)
{
	float id_sp = 0;
	float  t_hall = 0, rpm_abs = fabs(data_rpm);
	uint32_t tcnt = TIM5->CNT;

	if (WHEELED_handler.rpm != 0) t_hall = (60.0/WHEELED_handler.rpm)/60.0 * 2000000.0;//2000000.0
#if 0
	if (tcnt < fabs(t_hall))
	{
		WHEELED_handler.angle_estimation = (float)tcnt / t_hall * 60.0;
		WHEELED_handler.new_sector_theta = WHEELED_handler.sector_theta + WHEELED_handler.angle_estimation;
	}
	else
	{
//		WHEELED_handler.rpm = 0;
		WHEELED_handler.new_sector_theta = WHEELED_handler.sector_theta;
	}
#else
	WHEELED_handler.new_sector_theta = WHEELED_handler.sector_theta;
#endif
	if (rpm_abs < 0.001)
	{
		if (tcnt > fabs(t_hall))
		{
			WHEELED_handler.rpm = 0;
		}
		WHEELED_handler.hpid_omega.int_error = 0;
	}

	WHEELED_handler.c_loop++;
	if (WHEELED_handler.c_loop >= BLDC_CURRENT_CTRL_LOOP)
	{
		WHEELED_handler.c_loop = 0;
		PI_calculate(&WHEELED_handler.hpid_omega, data_rpm, WHEELED_handler.rpm);
	}
	BLDC_get_current_filter(&WHEELED_handler);
	BLDC_clark_park_trans(&WHEELED_handler, WHEELED_handler.new_sector_theta+90,
		  WHEELED_handler.hdlpf_current_filt[_u].result,
		  WHEELED_handler.hdlpf_current_filt[_v].result,
		  WHEELED_handler.hdlpf_current_filt[_w].result);
	PI_calculate(&WHEELED_handler.hpid_id, id_sp, WHEELED_handler.id_result);
	PI_calculate(&WHEELED_handler.hpid_iq, WHEELED_handler.hpid_omega.mv, WHEELED_handler.iq_result);
	BLDC_inv_clark_park_trans(&WHEELED_handler, WHEELED_handler.new_sector_theta+90, WHEELED_handler.hpid_id.mv, WHEELED_handler.hpid_iq.mv);
	BLDC_spwm(&WHEELED_handler);
}

void BLDC_angle_control (void)
{
	STEERING_handler.c_loop++;
	if (STEERING_handler.c_loop >= BLDC_CURRENT_CTRL_LOOP)
	{
		STEERING_handler.c_loop = 0;
		PD_calculate(&STEERING_handler.hpid_theta, data_angle, angle_sens);
	}
	angle_sector = (raw_angle + STEERING_handler.rotor_offset) * STEERING_POLE;
	BLDC_get_current_filter(&STEERING_handler);
	BLDC_clark_park_trans(&STEERING_handler, angle_sector+90,
		  STEERING_handler.hdlpf_current_filt[_u].result,
		  STEERING_handler.hdlpf_current_filt[_v].result,
		  STEERING_handler.hdlpf_current_filt[_w].result);
	PI_calculate(&STEERING_handler.hpid_id, 0, STEERING_handler.id_result);
	PI_calculate(&STEERING_handler.hpid_iq, -STEERING_handler.hpid_theta.mv, STEERING_handler.iq_result);
	BLDC_inv_clark_park_trans(&STEERING_handler, angle_sector+90, STEERING_handler.hpid_id.mv, STEERING_handler.hpid_iq.mv);
	BLDC_spwm(&STEERING_handler);
}

void zero_steer (void)
{
	float angle_steer_call = angle_sens;
	uint32_t t_steer_call = HAL_GetTick(), t_validate;
	t_validate = HAL_GetTick();
	wheel_state = 1;
	while (wheel_state)
	{
		if (HAL_GetTick() - t_steer_call >= 1)
		{
		  t_steer_call = HAL_GetTick();
		  angle_steer_call+=0.01;
		}
		if (!steer_call_ready)
		{
		  if (HAL_GetTick() - t_validate > 1000) steer_call_ready = 1;
		}
		data_angle = angle_steer_call;
	}
	data_angle = 0;
}

//pengujian1:
void BLDC_calibrate (void)
{
	uint32_t a_t, a_test = 0;
	float a_sp = 0;
	HAL_Delay(5000);
	while (1)
	{
		if (HAL_GetTick() - a_t >= 1000)
		{
			a_t = HAL_GetTick();
			a_test++;
			if (a_test > 4) a_test = 0;
			switch (a_test)
			{
			case 0: a_sp = 0; break;
			case 1: a_sp = 90; break;
			case 2: a_sp = -45; break;
			case 3: a_sp = 45; break;
			case 4: a_sp = -10; break;
			}
		}
		steering_motor_set_angle (a_sp);
		if (HAL_GetTick() != usb_tx_t)
		{
		  usb_tx_t = HAL_GetTick();
//		  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f %.3f\n",
//				  STEERING_handler.hdlpf_current_filt[_u].result*1000,
//				  STEERING_handler.hdlpf_current_filt[_v].result*1000,
//				  STEERING_handler.hdlpf_current_filt[_w].result*1000
//				  );
		  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f\n",
				  a_sp,angle_sens
//				  STEERING_handler.id_result*1000, STEERING_handler.iq_result*1000
				  );
		  CDC_Transmit_FS (usb_tx_buff, ln);
		  ////
		}
	}
}

void zero_mosfet (void)
{
	WHEELED_handler.ia = 0;
	WHEELED_handler.ib = 0;
	WHEELED_handler.ic = 0;
	STEERING_handler.ia = 0;
	STEERING_handler.ib = 0;
	STEERING_handler.ic = 0;
	BLDC_spwm(&WHEELED_handler);
	BLDC_spwm(&STEERING_handler);
}

void BLDC2_zero_cal (void)
{
	zero_mosfet ();
	uint32_t time = HAL_GetTick();
	while (HAL_GetTick() - time < 1000)
	{
		BLDC_inv_clark_park_trans(&STEERING_handler, 0, 0, 0.4);
		BLDC_spwm(&STEERING_handler);
		STEERING_handler.rotor_offset = raw_angle;
	}
	zero_mosfet ();
	flash_save_data ();

	uint32_t ln = sprintf ((char*)usb_tx_buff,
			"motor calibration successful\nplease reset the device!\n");
	CDC_Transmit_FS (usb_tx_buff, ln);
	while (1) ;
}

/*
 * @brief	battery_read_init
 * 			inisialisasi pembacaan baterai
 * @param	None
 * @retval	None
 */
void battery_read_init (void)
{
//	HAL_ADC_Start_DMA(&hadc1, &adc_battery, 1);
	dlpf_set_alpha(&battery_read, 0.01);
}

/*
 * @brief	read_battery
 * 			fungsi ini digunakan untuk kalkulasi pembacaan baterai dalam satuan Volt
 * @param	None
 * @retval	tegangan baterai
 */
float read_battery (void)
{
	float v_adc, v_bat;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	adc_battery = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	dlpf_get_result(&battery_read, adc_battery);
	v_adc = (float)battery_read.result / 4095.0 * 3.3;
	v_bat = v_adc * 11.0;
	return v_bat;
}

/*
 * CALLBACK ADC
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == WHEELED_ADC)
	{
		BLDC_get_current (&WHEELED_handler);
		BLDC_speed_control ();
	}
	if (hadc->Instance == STEERING_ADC)
	{
		BLDC_get_current (&STEERING_handler);
		BLDC_angle_control ();

		if (read_ready)
		{
		  if (AS5048A_send_data (1, 1, 0x3fff) == HAL_OK)
			  read_ready = 0;
		}
	}
}

/*
 * CALLBACK HALL SENSOR
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			BLDC_get_rpm (&WHEELED_handler);
			BLDC_get_sector (&WHEELED_handler);
			BLDC_rpm_sens (&WHEELED_handler);
			WHEELED_handler.zero_det_t = HAL_GetTick();
		}
	}
}

/*
 * CALLBACK EXTI
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Z_STEER_DET_Pin)
	{
		if (wheel_state && steer_call_ready)
		{
			steer_angle_offset = angle_sens + STEERING_handler.angle_offset;
			wheel_state = 0;
			data_angle = 0;
		}
	}
}

