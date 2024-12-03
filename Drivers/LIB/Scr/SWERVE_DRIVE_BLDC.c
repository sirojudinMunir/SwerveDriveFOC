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

extern float raw_angle;
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

uint32_t usb_tx_t;
float angle_sector;
double angle_sens, zero_offset = 0,
		mag_angle_offset;
int16_t hall_sens_count = 0;

BLDC_HandleTypeDef hbldc1, hbldc2;

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
	if (hbldc->channel == _BLDC1)
	{
		hbldc->rpm_abs = 60.0 / ((float)TIM5->CCR1 * 0.00012);
	}
	else if (hbldc->channel == _BLDC2)
	{
		//----------------------
	}
}

//============================================================================================

void BLDC_set_speed (double rpm)
{
	float phase_shft = 90, id_sp = 0;
	double  t_hall = 0, rpm_abs = fabs(rpm);
	uint32_t tcnt = TIM5->CNT;

	if (hbldc1.rpm != 0) t_hall = (60.0/hbldc1.rpm)/60.0 * 2000000.0;//2000000.0
	if (tcnt < fabs(t_hall))
	{
		hbldc1.angle_estimation = (double)tcnt / t_hall * 60.0;
//		hbldc1.new_sector_theta = hbldc1.sector_theta + hbldc1.angle_estimation;
		hbldc1.new_sector_theta = hbldc1.sector_theta;
	}
	else
	{
//		hbldc1.rpm = 0;
		hbldc1.new_sector_theta = hbldc1.sector_theta;
	}
	if (rpm_abs < 0.001)
	{
		if (tcnt > fabs(t_hall))
		{
			hbldc1.rpm = 0;
		}
		PID_set_konstanta(&hbldc1.hpid_omega, 0.0002, 0.00000002, 0);
		PID_set_max_value(&hbldc1.hpid_omega, 6.0, 100000000);
		hbldc1.hpid_omega.int_error = 0;
	}
//	else
//	{
//		if (tcnt > (fabs(t_hall) + 10))//100000
//		{
//			hbldc1.rpm = 0;
//		}
//		if (rpm_abs <= 100) PID_set_konstanta(&hbldc1.hpid_omega, 0.005, 0.0000001, 0);
//		else if (rpm_abs > 100 && rpm_abs <= 200) PID_set_konstanta(&hbldc1.hpid_omega, 0.01, 0.0000001, 0);
//		else if (rpm_abs > 200 && rpm_abs <= 300) PID_set_konstanta(&hbldc1.hpid_omega, 0.01, 0.0000001, 0);
//		else if (rpm_abs > 300 && rpm_abs <= 400) PID_set_konstanta(&hbldc1.hpid_omega, 0.015, 0.0000001, 0);
//		else if (rpm_abs > 400 && rpm_abs <= 500) PID_set_konstanta(&hbldc1.hpid_omega, 0.02, 0.0000001, 0);
//		else
//		{
//			PID_set_konstanta(&hbldc1.hpid_omega, 0.02, 0.0000001, 0);
////			id_sp = -2.0;
//		}
//		PID_set_max_value(&hbldc1.hpid_omega, 6.0, 60000000);
//	}

	hbldc1.c_loop++;
	if (hbldc1.c_loop >= 10)
	{
		hbldc1.c_loop = 0;
		PID_calculate(&hbldc1.hpid_omega, rpm, hbldc1.rpm);
	}
	if (hbldc1.hpid_omega.mv < 0) phase_shft = -90;
	BLDC_get_current_filter(&hbldc1);
	BLDC_clark_park_trans(&hbldc1, hbldc1.new_sector_theta+phase_shft,
		  hbldc1.hdlpf_current_filt[_u].result,
		  hbldc1.hdlpf_current_filt[_v].result,
		  hbldc1.hdlpf_current_filt[_w].result);
	PID_calculate(&hbldc1.hpid_id, id_sp, hbldc1.id_result);
	PID_calculate(&hbldc1.hpid_iq, fabs(hbldc1.hpid_omega.mv), hbldc1.iq_result);
	BLDC_inv_clark_park_trans(&hbldc1, hbldc1.new_sector_theta+phase_shft, hbldc1.hpid_id.mv, hbldc1.hpid_iq.mv);
	BLDC_spwm(&hbldc1);
}

void BLDC_set_angle (float deg)
{
	float phase_shift = -90;
	hbldc2.c_loop++;
	if (hbldc2.c_loop >= 10)
	{
		hbldc2.c_loop = 0;
		PID_calculate(&hbldc2.hpid_theta, deg, angle_sens);
	}
	if (hbldc2.hpid_theta.mv < 0) phase_shift = 90.0;
	angle_sector = (raw_angle+mag_angle_offset) * 11.0;
	BLDC_get_current_filter(&hbldc2);
	BLDC_clark_park_trans(&hbldc2, angle_sector+phase_shift,
		  hbldc2.hdlpf_current_filt[_u].result,
		  hbldc2.hdlpf_current_filt[_v].result,
		  hbldc2.hdlpf_current_filt[_w].result);
	PID_calculate(&hbldc2.hpid_id, 0, hbldc2.id_result);
	PID_calculate(&hbldc2.hpid_iq, fabs(hbldc2.hpid_theta.mv), hbldc2.iq_result);
	BLDC_inv_clark_park_trans(&hbldc2, angle_sector+phase_shift, hbldc2.hpid_id.mv, hbldc2.hpid_iq.mv);
	BLDC_spwm(&hbldc2);
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
		BLDC_set_angle (a_sp);
		if (HAL_GetTick() != usb_tx_t)
		{
		  usb_tx_t = HAL_GetTick();
//		  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f %.3f\n",
//				  hbldc2.hdlpf_current_filt[_u].result*1000,
//				  hbldc2.hdlpf_current_filt[_v].result*1000,
//				  hbldc2.hdlpf_current_filt[_w].result*1000
//				  );
		  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f\n",
				  a_sp,angle_sens
//				  hbldc2.id_result*1000, hbldc2.iq_result*1000
				  );
		  CDC_Transmit_FS (usb_tx_buff, ln);
		  ////
		}
	}
}

void zero_mosfet (void)
{
	hbldc1.ia = 0;
	hbldc1.ib = 0;
	hbldc1.ic = 0;
	hbldc2.ia = 0;
	hbldc2.ib = 0;
	hbldc2.ic = 0;
	BLDC_spwm(&hbldc1);
	BLDC_spwm(&hbldc2);
}

void BLDC2_zero_cal (void)
{
	zero_mosfet ();
	uint32_t time = HAL_GetTick();
	while (HAL_GetTick() - time < 1000)
	{
		BLDC_inv_clark_park_trans(&hbldc2, 0, 0, 0.4);
		BLDC_spwm(&hbldc2);
		mag_angle_offset = raw_angle;
	}
	zero_mosfet ();
	flash_save_data ();

	uint32_t ln = sprintf ((char*)usb_tx_buff,
			"motor calibration successful\nplease reset the device!\n");
	CDC_Transmit_FS (usb_tx_buff, ln);
	while (1) ;
}
