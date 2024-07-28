/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "PID_lib.h"
#include "DLPF_lib.h"
#include "FLASH_lib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_SINE_WAVE 360
#define DEG_2_RAD 	0.01745329251994
#define RAD_2_DEG	57.2957795130823208
#define WHEEL_1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

enum swerve_register
{
	_SET_WHEEL			= 0xA0,
	_SET_BEEP			= 0xA3,
	_SET_ZERO_OFFSET 	= 0xB0,
	_SET_PID_M1 		= 0xC1,
	_SET_PID_M2 		= 0xC2,
	_READ_ANGLE			= 0xD0,
	_READ_HALL_SECTOR	= 0xD1,
	_READ_RPM			= 0xD2
};

enum bldc_channel
{
	_BLDC1, _BLDC2
};

enum bldc_output
{
	_u, _v, _w
};

typedef enum
{
	_dq_test, _current_test, _torque_test, _speed_test, _angle_test
}BLDC_TestTypedef;

typedef struct
{
	uint8_t channel;
	PID_HandleTypeDef hpid_id, hpid_iq, hpid_omega, hpid_theta;
	DLPF_HandleTypeDef hdlpf_current_filt[3], hdlpf_cmps;
	uint32_t adc_buff[3], spwm[3], t_pwm, c_loop, zero_det_t;
	uint8_t hall_sector;
	double sector_theta, last_sector_theta, new_sector_theta;
	double theta, raw_current[3], id_result, iq_result,
			p_shift,
			ia, ib, ic, max_current,
			rpm_abs, rpm, cmps,
			angle_estimation, last_angle_estimation;
	int8_t dir, last_dir;
	_Bool state;
}BLDC_HandleTypeDef;

CAN_TxHeaderTypeDef   	TxHeader;
CAN_RxHeaderTypeDef   	RxHeader;
BLDC_HandleTypeDef hbldc1, hbldc2;
DLPF_HandleTypeDef i_test[3], battery_read;

uint32_t flash_data_buff[2];
uint32_t TxMailbox, cnt = 0, t_test, t_delay = 1000, cnt_ovf = 0,
		mag_zero_cal_t,
		led_blink_time, led_blink_delay, usb_tx_t,
		adc_battery;
uint16_t encd_data_rx;
uint8_t spi_rx[2], usb_tx_buff[100], can_rx_buff[8], can_tx_buff[8],
		led_can_respone = 0, led_blink_count = 0;
int16_t hall_sens_count = 0;
float raw_angle, last_raw_angle, m2_angle = 0, angle_sector, hrpm,
		z_lift = 0, z_offset = 0, z_cm = 0,
		a = 0, b = 0, cm_test = 0, sector_test = 0, rpm_trsh = 25;

double angle_sens, raw_angle_ovf, steer_angle_offset = 0, zero_offset = 0,
		mag_angle_offset;

_Bool error_flag, read_ready = 0, flag = 0, wheel_state = 0, update_rpm_flag = 0,
		mag_zero_set_flag = 0, mag_cal_mode = 0,
		led_blink_flag = 1, steer_call_ready = 0, null_rpm = 0,
		lifter_ok_flag = 0;

int32_t count_hall;
uint16_t wheel_addr;
float data_angle = 0, data_rpm = 0;
double x_kp, x_ki, x_kd;
_Bool next_change_param = 0, ctrl_h= 0, start_checking_mosfet = 0, bldc2_start_calibrate = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//==========================================================================================

/*
 * @brief	CAN_filter_config
 * 			fungsi untuk konfigurasi filter pada CAN
 * @param	None
 * @retval	None
 */
void CAN_filter_config (void)
{
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 18;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = wheel_addr<<5;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = wheel_addr<<5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 0;

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
}

/*
 * @brief	CAN_get_wheel
 * 			fungsi ini digunakan untuk mendapatkan nilai kecepatan dan sudut roda dari
 * 			perangkat lain melalui CAN bus
 * @param	None
 * @retval	None
 */
void CAN_get_wheel (void)
{
	int16_t data_rx[2];
	float angle_temp, rpm_temp, angle_diff;

	/* menerima data */
	data_rx[0] = (int16_t)(can_rx_buff[1] | can_rx_buff[2]<<8);
	data_rx[1] = (int16_t)(can_rx_buff[3] | can_rx_buff[4]<<8);
	angle_temp = (float)data_rx[0] / 10.0;
	rpm_temp = (float)data_rx[1] / 10.0;

	/* optimalisasi gerakan */
	while ((angle_sens - angle_temp) > 180)
	{
		angle_temp += 360;
	}
	while ((angle_sens - angle_temp) < -180)
	{
		angle_temp -=360;
	}
	angle_diff = angle_sens - angle_temp;
	if (fabs(angle_diff) > 90)
	{
		if (angle_diff > 0) data_angle = angle_temp + 180.00;
		else data_angle = angle_temp - 180.00;
		data_rpm = -rpm_temp;
	}
	else
	{
		data_angle = angle_temp;
		data_rpm = rpm_temp;
	}
}

void optimalisasi_gerakan (double angle_temp, double rpm_temp)
{
	double angle_diff;
	/* optimalisasi gerakan */
	while ((angle_sens - angle_temp) > 180)
	{
		angle_temp += 360;
	}
	while ((angle_sens - angle_temp) < -180)
	{
		angle_temp -=360;
	}
	angle_diff = angle_sens - angle_temp;
	if (fabs(angle_diff) > 90)
	{
		if (angle_diff > 0) data_angle = angle_temp + 180.00;
		else data_angle = angle_temp - 180.00;
		data_rpm = -rpm_temp;
	}
	else
	{
		data_angle = angle_temp;
		data_rpm = rpm_temp;
	}
}

/*
 * @brief	CAN_get_zero_offset
 * 			fungsi ini digunakan untuk mendapatkan nilai offset sudut roda dari
 * 			perangkat lain melalui CAN bus
 * @param	None
 * @retval	None
 */
void CAN_get_zero_offset (void)
{
	int16_t offset_temp;

	offset_temp = (int16_t)(can_rx_buff[1] | can_rx_buff[2]<<8);
	zero_offset = (float)offset_temp;
}

/*
 * @brief	CAN_send_wheel
 * 			fungsi ini digunakan untuk mengirim sudut roda dan perubahan pulse hall sensor
 * 			ke perangkat lain melalui CAN bus
 * @param	addr		alamat penerima
 * 			angle 		sudut roda saat ini
 * 			hall_cnt	perubahan pulse hall sensor roda
 * @retval	None
 */
void CAN_send_wheel (uint32_t addr, float angle, int16_t hall_cnt)
{
	int16_t angle_temp;

	while (angle < 0)
	{
		angle += 360.0;
	}
	while (angle > 360)
	{
		angle -= 360.0;
	}
	angle_temp = angle * 10.0;

	TxHeader.DLC = 5;
	TxHeader.StdId = addr;
	can_tx_buff[0] = _SET_WHEEL;
	can_tx_buff[1] = angle_temp & 0xFF;
	can_tx_buff[2] = (angle_temp >> 8) & 0xFF;
	can_tx_buff[3] = hall_cnt & 0xFF;
	can_tx_buff[4] = (hall_cnt >> 8) & 0xFF;

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, can_tx_buff, &TxMailbox);
}

/*
 * @brief	CAN_send_ok
 * 			fungsi ini digunakan mengirim respon ke perangkat lain melalui CAN bus
 * @param	addr	alamat penerima
 * @retval	None
 */
void CAN_send_ok (uint32_t addr)
{
	TxHeader.DLC = 2;
	TxHeader.StdId = addr;
	can_tx_buff[0] = (uint8_t)'O';
	can_tx_buff[1] = (uint8_t)'K';

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, can_tx_buff, &TxMailbox);
}

//==========================================================================================

/*
 * @brief	blink_respone
 * 			indikator LED jika perangkat menerima data yang sesuai
 * @param	None
 * @retval	None
 */
void blink_respone (void)
{
	if (led_can_respone != 0)
	{
		if (led_blink_flag)
		{
			led_blink_flag = 0;
			led_blink_time = HAL_GetTick();
			switch (led_can_respone)
			{
			case _SET_WHEEL:
				led_blink_delay = 30;
				led_blink_count = 2;
				break;
			case _SET_ZERO_OFFSET:
				led_blink_delay = 100;
				led_blink_count = 4;
				break;
			case _SET_PID_M1:
				led_blink_delay = 150;
				led_blink_count = 6;
				break;
			case _SET_PID_M2:
				led_blink_delay = 150;
				led_blink_count = 6;
				break;
			}
		}
		if (HAL_GetTick() - led_blink_time >= led_blink_delay)
		{
			led_blink_time = HAL_GetTick();
			if (led_blink_count > 0)
			{
				if (led_blink_count % 2 == 0)
					LED_BUILTIN_GPIO_Port->BSRR = LED_BUILTIN_Pin<<16;
				else
					LED_BUILTIN_GPIO_Port->BSRR = LED_BUILTIN_Pin;
				led_blink_count--;
			}
			else
			{
				led_can_respone = 0;
				led_blink_flag = 1;
			}
		}
	}
}

//============================================================================================

/*
 * @brief	AS5048A_send_data
 * 			fungsi untuk mengirim comamnd ke sensor magnetic encoder AS5048A via SPI
 * @param	par		parity
 * 			rw		read/write
 * 			addr	alamat
 * @retval	HAL status
 */
HAL_StatusTypeDef AS5048A_send_data (_Bool par, _Bool rw, uint16_t addr)
{
	HAL_StatusTypeDef status;
	uint8_t data[2];
	data[0] = (par << 7) | (rw << 6) | (addr >> 8);
	data[1] = addr & 0xff;
	SPI_CS_GPIO_Port->BSRR = SPI_CS_Pin<<16;
	status = HAL_SPI_TransmitReceive_DMA (&hspi1, data, spi_rx, 2);
	return status;
}

//============================================================================================

/*
 * @brief	battery_read_init
 * 			inisialisasi pembacaan baterai
 * @param	None
 * @retval	None
 */
void battery_read_init (void)
{
	HAL_ADC_Start_DMA(&hadc1, &adc_battery, 1);
	dlpf_set_alpha(&battery_read, 0.01);
}

/*
 * @brief	read_battery
 * 			fungsi ini digunakan untuk kalkulasi pembacaan baterai dalam satuan Volt
 * @param	None
 * @retval	tegangan baterai
 */
double read_battery (void)
{
	double v_adc, v_bat;
	dlpf_get_result(&battery_read, adc_battery);
	v_adc = (double)battery_read.result / 4095.0 * 3.3;
	v_bat = v_adc * 11.0;
	return v_bat;
}

//============================================================================================

void BLDC_init (BLDC_HandleTypeDef *hbldc)
{
	hbldc->p_shift = 90;
	if (hbldc->channel == _BLDC1)
	{
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_u], 0.9998);//0.9998
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_v], 0.9998);
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_w], 0.9998);
		dlpf_set_alpha(&hbldc->hdlpf_cmps, 0.002);
		PID_set_konstanta(&hbldc->hpid_id, 0.02, 0.0001, 0);//0.02, 0.0001, 0
		PID_set_max_value(&hbldc->hpid_id, 4.0, 40000.0);//4 4000
		PID_set_konstanta(&hbldc->hpid_iq, 0.02, 0.0002, 0);//0.02, 0.0005, 0
		PID_set_max_value(&hbldc->hpid_iq, 4.0, 20000.0);
		PID_set_konstanta(&hbldc->hpid_omega, 0.002, 0.0000001, 0);//0.002, 0.0000001, 0
		PID_set_max_value(&hbldc->hpid_omega, 4.0, 40000000);
		//M1+KI0.00000001

		HAL_TIMEx_HallSensor_Start_IT (&htim5);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
		HAL_ADCEx_InjectedStart_IT(&hadc2);
		M2_EN_U_GPIO_Port->BSRR = M2_EN_U_Pin;
		M2_EN_V_GPIO_Port->BSRR = M2_EN_V_Pin;
		M2_EN_W_GPIO_Port->BSRR = M2_EN_W_Pin;
	}
	else if (hbldc->channel == _BLDC2)
	{

#ifdef WHEEL_Z
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_u], 0.95);
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_v], 0.95);
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_w], 0.95);
		PID_set_konstanta(&hbldc->hpid_id, 0.0005, 0.000005, 0);//0.00005
		PID_set_max_value(&hbldc->hpid_id, 3.0, 400000);
		PID_set_konstanta(&hbldc->hpid_iq, 0.0005, 0.000005, 0);//0.00004
		PID_set_max_value(&hbldc->hpid_iq, 3.0, 400000);
		PID_set_konstanta(&hbldc->hpid_theta, 25, 0, 1.2);
		PID_set_max_value(&hbldc->hpid_theta, 3.0, 0);
#else
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_u], 0.99998);//0.99925
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_v], 0.99998);
		dlpf_set_alpha(&hbldc->hdlpf_current_filt[_w], 0.99998);
		PID_set_konstanta(&hbldc->hpid_id, 0.001, 0.00002, 0);//0.001, 0.000025, 0
		PID_set_max_value(&hbldc->hpid_id, 3.8, 200000);
		PID_set_konstanta(&hbldc->hpid_iq, 0.001, 0.00004, 0);//0.001, 0.00002, 0
		PID_set_max_value(&hbldc->hpid_iq, 3.8, 100000);
		PID_set_konstanta(&hbldc->hpid_theta, 1.2, 0, 0.0001);//1.2, 0, 0.007
		PID_set_max_value(&hbldc->hpid_theta, 9.0, 0);
#endif

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
		HAL_ADCEx_InjectedStart_IT(&hadc3);
		M1_EN_U_GPIO_Port->BSRR = M1_EN_U_Pin;
		M1_EN_V_GPIO_Port->BSRR = M1_EN_V_Pin;
		M1_EN_W_GPIO_Port->BSRR = M1_EN_W_Pin;
	}
	dlpf_set_alpha(&i_test[0], 0.01);
	dlpf_set_alpha(&i_test[1], 0.01);
	dlpf_set_alpha(&i_test[2], 0.01);
}

//============================================================================================

void BLDC_get_current_test (BLDC_HandleTypeDef *hbldc)
{
	hbldc->raw_current[_u] = (double)hbldc->adc_buff[0]*0.002014652014652;//0.002014652014652
	hbldc->raw_current[_v] = (double)hbldc->adc_buff[1]*0.002014652014652;
	hbldc->raw_current[_w] = (double)hbldc->adc_buff[2]*0.002014652014652;
	dlpf_get_result (&hbldc->hdlpf_current_filt[_u], hbldc->raw_current[_u]);
	dlpf_get_result (&hbldc->hdlpf_current_filt[_v], hbldc->raw_current[_v]);
	dlpf_get_result (&hbldc->hdlpf_current_filt[_w], hbldc->raw_current[_w]);
}

void BLDC_get_current (BLDC_HandleTypeDef *hbldc)
{
	float iu, iv, iw;
	uint32_t current_pwm[3];

	if (hbldc->channel == _BLDC1)
	{
		current_pwm[_u] = TIM8->CCR1;
		current_pwm[_v] = TIM8->CCR2;
		current_pwm[_w] = TIM8->CCR3;
	}
	else if (hbldc->channel == _BLDC2)
	{
		current_pwm[_u] = TIM1->CCR1;
		current_pwm[_v] = TIM1->CCR2;
		current_pwm[_w] = TIM1->CCR3;
	}
	iu = (double)hbldc->adc_buff[0]*0.0040293040293;
	iv = (double)hbldc->adc_buff[1]*0.0040293040293;
	iw = (double)hbldc->adc_buff[2]*0.0040293040293;

	if (current_pwm[_u] == hbldc->t_pwm)
	{
		hbldc->raw_current[_u] = iv+iw;
		hbldc->raw_current[_v] = -iv;
		hbldc->raw_current[_w] = -iw;
	}
	else if (current_pwm[_v] == hbldc->t_pwm)
	{
		hbldc->raw_current[_u] = -iu;
		hbldc->raw_current[_v] = iu+iw;
		hbldc->raw_current[_w] = -iw;
	}
	else if (current_pwm[_w] == hbldc->t_pwm)
	{
		hbldc->raw_current[_u] = -iu;
		hbldc->raw_current[_v] = -iv;
		hbldc->raw_current[_w] = iu+iv;
	}
}

void BLDC_get_current_filter (BLDC_HandleTypeDef *hbldc)
{
	dlpf_get_result (&hbldc->hdlpf_current_filt[_u], hbldc->raw_current[_u]);
	dlpf_get_result (&hbldc->hdlpf_current_filt[_v], hbldc->raw_current[_v]);
	dlpf_get_result (&hbldc->hdlpf_current_filt[_w], hbldc->raw_current[_w]);
}

//============================================================================================

void BLDC_clark_park_trans (BLDC_HandleTypeDef *hbldc, float deg, float ia, float ib, float ic)
{
	float alpha, beta;
	deg *= DEG_2_RAD;

	alpha = ia - 0.5 * (ib + ic);
	beta = 0.8660254037844386 * (ib - ic);

	hbldc->id_result = cos (deg) * alpha + sin (deg) * beta;
	hbldc->iq_result = cos (deg) * beta - sin (deg) * alpha;
}

//============================================================================================

void BLDC_inv_clark_park_trans (BLDC_HandleTypeDef *hbldc, float deg, float d, float q)
{
	float alpha, beta;
	deg *= DEG_2_RAD;

	alpha = d * cos (deg) - q * sin (deg);
	beta = d * sin (deg) + q * cos (deg);
	hbldc->ia = alpha;
	hbldc->ib = -0.5 * alpha + 0.8660254037844386 * beta;
	hbldc->ic = -0.5 * alpha - 0.8660254037844386 * beta;
}


//============================================================================================

void BLDC_spwm (BLDC_HandleTypeDef *hbldc)
{
	uint32_t pwm_h_temp = 0;

	hbldc->spwm[_u] = (hbldc->ia * 500.0)+2047.0;
	hbldc->spwm[_v] = (hbldc->ib * 500.0)+2047.0;
	hbldc->spwm[_w] = (hbldc->ic * 500.0)+2047.0;
	if (hbldc->spwm[_u] > 4000) hbldc->spwm[_u] = 4000;
	if (hbldc->spwm[_v] > 4000) hbldc->spwm[_v] = 4000;
	if (hbldc->spwm[_w] > 4000) hbldc->spwm[_w] = 4000;
	for (uint8_t i = 0; i < 3; i++)
	{
		if (pwm_h_temp < hbldc->spwm[i]) pwm_h_temp = hbldc->spwm[i];
	}
	hbldc->t_pwm = pwm_h_temp;

	if (hbldc->channel == _BLDC1)
	{
		TIM8->CCR1 = hbldc->spwm[_u];
		TIM8->CCR2 = hbldc->spwm[_v];
		TIM8->CCR3 = hbldc->spwm[_w];
		TIM8->CCR4 = 4000;
	}
	else if (hbldc->channel == _BLDC2)
	{
		TIM1->CCR1 = hbldc->spwm[_u];
		TIM1->CCR2 = hbldc->spwm[_v];
		TIM1->CCR3 = hbldc->spwm[_w];
		TIM1->CCR4 = 4000;
	}
}

void BLDC_beep (BLDC_HandleTypeDef *hbldc, uint32_t freq, uint32_t time_delay)
{
	uint32_t prescaller;
	if (freq != 0)
	{
		if (freq < 100) freq = 100;
		else if (freq > 20000) freq = 20000;
		if (hbldc->channel == _BLDC1)
		{
			prescaller = 168000000 / (freq * 4095);
			TIM8->PSC = prescaller;
		}
		else
		{
			prescaller = 168000000 / (freq * 4095);
			TIM1->PSC = prescaller;
		}
		BLDC_inv_clark_park_trans (hbldc, 0, 0, 0.5);
		BLDC_spwm (hbldc);
	}
	else
	{
		BLDC_inv_clark_park_trans (hbldc, 0, 0, 0);
		BLDC_spwm (hbldc);
	}
	HAL_Delay(time_delay);
	BLDC_inv_clark_park_trans (hbldc, 0, 0, 0);
	BLDC_spwm (hbldc);
	if (hbldc->channel == _BLDC1) TIM8->PSC = 1;
	else TIM1->PSC = 1;
}

void BLDC_current_control (BLDC_HandleTypeDef *hbldc, double id, double iq, double theta)
{
	BLDC_get_current(hbldc);
	BLDC_clark_park_trans(hbldc, theta,
			hbldc->hdlpf_current_filt[_u].result,
			hbldc->hdlpf_current_filt[_v].result,
			hbldc->hdlpf_current_filt[_w].result);
	PID_calculate(&hbldc->hpid_id, id, hbldc->id_result);
	PID_calculate(&hbldc->hpid_iq, iq, hbldc->iq_result);
	BLDC_inv_clark_park_trans(hbldc, theta, hbldc->hpid_id.mv, hbldc->hpid_iq.mv);
	BLDC_spwm(hbldc);
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
	else
	{
		if (tcnt > (fabs(t_hall) + 10))//100000
		{
			hbldc1.rpm = 0;
		}
		if (rpm_abs <= 100) PID_set_konstanta(&hbldc1.hpid_omega, 0.005, 0.0000001, 0);
		else if (rpm_abs > 100 && rpm_abs <= 200) PID_set_konstanta(&hbldc1.hpid_omega, 0.01, 0.0000001, 0);
		else if (rpm_abs > 200 && rpm_abs <= 300) PID_set_konstanta(&hbldc1.hpid_omega, 0.01, 0.0000001, 0);
		else if (rpm_abs > 300 && rpm_abs <= 400) PID_set_konstanta(&hbldc1.hpid_omega, 0.015, 0.0000001, 0);
		else if (rpm_abs > 400 && rpm_abs <= 500) PID_set_konstanta(&hbldc1.hpid_omega, 0.02, 0.0000001, 0);
		else
		{
			PID_set_konstanta(&hbldc1.hpid_omega, 0.02, 0.0000001, 0);
//			id_sp = -2.0;
		}
		PID_set_max_value(&hbldc1.hpid_omega, 6.0, 60000000);
	}

	hbldc1.c_loop++;
	if (hbldc1.c_loop >= 2)
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
//	if (hbldc2.hpid_theta.mv < 0) phase_shift = 90.0;
	angle_sector = (raw_angle+mag_angle_offset) * 11.0;
	BLDC_get_current_filter(&hbldc2);
	BLDC_clark_park_trans(&hbldc2, angle_sector+phase_shift,
		  hbldc2.hdlpf_current_filt[_u].result,
		  hbldc2.hdlpf_current_filt[_v].result,
		  hbldc2.hdlpf_current_filt[_w].result);
	PID_calculate(&hbldc2.hpid_id, 0, hbldc2.id_result);
	PID_calculate(&hbldc2.hpid_iq, (hbldc2.hpid_theta.mv), hbldc2.iq_result);
	BLDC_inv_clark_park_trans(&hbldc2, angle_sector+phase_shift, hbldc2.hpid_id.mv, hbldc2.hpid_iq.mv);
	BLDC_spwm(&hbldc2);
}

/******************************************************************************************************/
uint32_t step = 0;
void BLDC_set_z (float z)
{
	float phase_shift = -90;
	hbldc2.c_loop++;
	if (hbldc2.c_loop >= 10)
	{
		hbldc2.c_loop = 0;
		PID_calculate(&hbldc2.hpid_theta, z, z_lift);
	}
	angle_sector = (raw_angle+mag_angle_offset) * 7.0;
	BLDC_get_current_filter(&hbldc2);
	BLDC_clark_park_trans(&hbldc2, angle_sector-phase_shift,
		  hbldc2.hdlpf_current_filt[_u].result,
		  hbldc2.hdlpf_current_filt[_v].result,
		  hbldc2.hdlpf_current_filt[_w].result);
	PID_calculate(&hbldc2.hpid_id, 0, hbldc2.id_result);
	PID_calculate(&hbldc2.hpid_iq, hbldc2.hpid_theta.mv, hbldc2.iq_result);
	BLDC_inv_clark_park_trans(&hbldc2, angle_sector-phase_shift, hbldc2.hpid_id.mv, hbldc2.hpid_iq.mv);
	BLDC_spwm(&hbldc2);
}

void lifter_goto_zero ()
{
	uint32_t z_timer = HAL_GetTick();
	float z = z_lift;
	while (HAL_GPIO_ReadPin(Z_LIMIT_GPIO_Port, Z_LIMIT_Pin))
	{
		if (HAL_GetTick() - z_timer >= 1)
		{
			z_timer = HAL_GetTick();
			z-=0.01;
		}
		BLDC_set_z (z);
	}
	z_offset = z_lift;
	BLDC_set_z (0);
}

void CAN_get_z (void)
{
	int16_t z_temp;

	z_temp = (int16_t)(can_rx_buff[1] | can_rx_buff[2]<<8);
	z_cm = (float)z_temp / 10.0;
}

void CAN_send_z (uint32_t addr, float z)
{
	int16_t z_temp = z * 10.0;
	TxHeader.DLC = 3;
	TxHeader.StdId = addr;
	can_tx_buff[0] = 0x12;
	can_tx_buff[1] = (uint8_t)(z_temp && 0xff);
	can_tx_buff[2] = (uint8_t)(z_temp>>8 && 0xff);

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, can_tx_buff, &TxMailbox);
}

/******************************************************************************************************/

//pengujian1:
void BLDC_calibrate ()
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

void zero_mosfet ();

void BLDC2_zero_cal ()
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
	flash_data_buff[0] = (int32_t)(mag_angle_offset*100000);
	FLASH_write(0, flash_data_buff, 2, DATA_TYPE_32);

	uint32_t ln = sprintf ((char*)usb_tx_buff,
			"motor calibration successful\nplease reset the device!\n");
	CDC_Transmit_FS (usb_tx_buff, ln);
	while (1) ;
}

double str2float (char *str, uint8_t ln)
{
	double result;
	double num[2]={0, 0}, num2_dev = 10;
	uint8_t start_num2;
	_Bool coma_det = 0;
	for (uint8_t i = 0; i < ln; i++)
	{
		if (!coma_det)
		{
			if (str[i] >= '0' && str[i] <= '9')
			{
				if (i != 0) num[0] *= 10;
				num[0] += (str[i] - '0');
			}
			else if (str[i] == '.' || str[i] == ',')
			{
				coma_det = 1;
				start_num2 = i+1;
			}
		}
		else
		{
			if (str[i] >= '0' && str[i] <= '9')
			{
				if (i != start_num2)
				{
					num[1] *= 10;
					num2_dev *= 10;
				}
				num[1] += (str[i] - '0');
			}
		}
	}
	result = num[0] + num[1]/num2_dev;
	return result;
}

_Bool str_compare (char *str1, char *str2, uint32_t ln)
{
	_Bool result = 1;
	for (uint32_t i = 0; i < ln; i++)
	{
		if (str1[i] != str2[i]){
			result = 0;
			break;
		}
	}
	return result;
}

void usb_motor_calibrate (char *cmd)
{
	if (str_compare (cmd, "M2+CAL", 6))
	{
		mag_zero_set_flag = 1;
	}
}

void pid_setting (char *cmd)
{
	uint8_t ln_num = 0;
	if (cmd[0] == 'M')
	{
		for (uint8_t n = 2; cmd[n] != 0; n++)
		{
			if (cmd[n] == '+')
			{
				if (cmd[n+1] == 'K')
				{
					ln_num = 0;
					for (uint8_t i = n+3; (cmd[i] >= '0' && cmd[i] <= '9') || cmd[i]=='.' || cmd[i]==','; i++)
					{
						ln_num++;
					}
					switch (cmd[n+2])
					{
					case 'P':
						x_kp = str2float(cmd+n+3, ln_num);
						break;
					case 'I':
						x_ki = str2float(cmd+n+3, ln_num);
						break;
					case 'D':
						x_kd = str2float(cmd+n+3, ln_num);
						break;
					}
					if (cmd[1] == '1')
					{

						PID_set_konstanta(&hbldc1.hpid_omega, x_kp, x_ki, x_kd);
//						PID_set_konstanta(&hbldc1.hpid_id, x_kp, x_ki, 0);
//						PID_set_konstanta(&hbldc1.hpid_iq, x_kp, x_ki, 0);
					}
					else if (cmd[1] == '2')
					{
						PID_set_konstanta(&hbldc2.hpid_theta, x_kp, x_ki, x_kd);
					}
				}
			}
		}
	}
	else if (cmd[0] == 'N' || cmd[0] == 'n')
	{
		next_change_param = 1;
	}
	else if (cmd[0] == '0')
	{
		start_checking_mosfet = 1;
	}
}

void zero_mosfet ()
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

void mosfet_check ()
{
	_Bool error[6] = {0, 0, 0, 0, 0, 0};
	uint32_t time_error_tolerant[6], time_test, loading_t;
	uint8_t system_failed = 0;
	double current_tolerant = 0.05;

	hbldc1.ia = 0;
	hbldc1.ib = 0;
	hbldc1.ic = 0;
	hbldc2.ia = 0;
	hbldc2.ib = 0;
	hbldc2.ic = 0;
	M2_EN_U_GPIO_Port->BSRR = M2_EN_U_Pin<<16;
	M2_EN_V_GPIO_Port->BSRR = M2_EN_V_Pin<<16;
	M2_EN_W_GPIO_Port->BSRR = M2_EN_W_Pin<<16;
	M1_EN_U_GPIO_Port->BSRR = M1_EN_U_Pin<<16;
	M1_EN_V_GPIO_Port->BSRR = M1_EN_V_Pin<<16;
	M1_EN_W_GPIO_Port->BSRR = M1_EN_W_Pin<<16;

	LED_BUILTIN_GPIO_Port->BSRR = LED_BUILTIN_Pin;
	while (!start_checking_mosfet)
	{
		BLDC_get_current(&hbldc1);
		BLDC_get_current(&hbldc2);
	}
	LED_BUILTIN_GPIO_Port->BSRR = LED_BUILTIN_Pin<<16;
	CDC_Transmit_FS ((uint8_t *)"\nStart Checking\n\n", 17);
	time_test = HAL_GetTick();
	loading_t = HAL_GetTick();
	while (HAL_GetTick() - time_test < 1000)
	{
		BLDC_spwm(&hbldc1);
		BLDC_spwm(&hbldc2);
		BLDC_get_current_test(&hbldc1);
		BLDC_get_current_test(&hbldc2);
		error[0] = (hbldc1.hdlpf_current_filt[_u].result > current_tolerant);
		error[1] = (hbldc1.hdlpf_current_filt[_v].result > current_tolerant);
		error[2] = (hbldc1.hdlpf_current_filt[_w].result > current_tolerant);
		error[3] = (hbldc2.hdlpf_current_filt[_u].result > current_tolerant);
		error[4] = (hbldc2.hdlpf_current_filt[_v].result > current_tolerant);
		error[5] = (hbldc2.hdlpf_current_filt[_w].result > current_tolerant);
		for (uint8_t i = 0; i < 6; i++)
		{
			if (!error[i]) time_error_tolerant[i] = HAL_GetTick();
			else
			{
				if (HAL_GetTick() - time_error_tolerant[i] >= 1)
				{
					system_failed |= 1 << i;
					switch (i)
					{
					case 0: M2_EN_U_GPIO_Port->BSRR = M2_EN_U_Pin<<16; break;
					case 1: M2_EN_V_GPIO_Port->BSRR = M2_EN_V_Pin<<16; break;
					case 2: M2_EN_W_GPIO_Port->BSRR = M2_EN_W_Pin<<16; break;
					case 3: M1_EN_U_GPIO_Port->BSRR = M1_EN_U_Pin<<16; break;
					case 4: M1_EN_V_GPIO_Port->BSRR = M1_EN_V_Pin<<16; break;
					case 5: M1_EN_W_GPIO_Port->BSRR = M1_EN_W_Pin<<16; break;
					}
				}
			}
		}
		if (HAL_GetTick() - loading_t >= 100)
		{
			loading_t = HAL_GetTick();
			CDC_Transmit_FS ((uint8_t *)"* ", 2);
		}
	}
	HAL_Delay(1);
	CDC_Transmit_FS ((uint8_t *)"\n\n", 2);
	HAL_Delay(1);
	if (system_failed == 0)
	{
		CDC_Transmit_FS ((uint8_t *)"System OK!\n", 11);
	}
	else
	{
		CDC_Transmit_FS ((uint8_t *)
				"!!FAILUR DETECTED!!\nPlease Check Your MOSFETs: ", 47);
		HAL_Delay(10);
		for (uint8_t i = 0; i < 6; i++)
		{
			if ((system_failed >> i) & 1)
			{
				uint32_t ln = sprintf ((char*)usb_tx_buff, "%d ", i+1);
				CDC_Transmit_FS (usb_tx_buff, ln);
				HAL_Delay(1);
			}
		}
	}
	CDC_Transmit_FS ((uint8_t *)"\n\n", 2);
	start_checking_mosfet = 0;
}

void BLDC1_testing (BLDC_TestTypedef mode)
{
	float iq_sp = 1, rpm_sp, q = 0;
	_Bool pulse_state = 0;
	uint32_t param_time = HAL_GetTick(), time_delay = 100, param_change = 0;
	_Bool stop_send = 1;
	while (1)
	{
		switch (mode)
		{
		case _dq_test:
			if (!stop_send)
			{
				if (HAL_GetTick() - param_time >= time_delay)
				{
					param_time = HAL_GetTick();
					if (pulse_state == 0)
					{
						pulse_state = 1;
						time_delay = 2000;
						q = iq_sp;
					}
					else
					{
						stop_send = 1;
						q = 0;
					}
				}
				if (HAL_GetTick() != usb_tx_t)
				{
				  usb_tx_t = HAL_GetTick();
				  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f %.3f\n",
						  q*1000, i_test[0].result*1000, i_test[1].result*1000);
				  CDC_Transmit_FS (usb_tx_buff, ln);
				}
			}
			else
			{
				if (next_change_param)
				{
					next_change_param = 0;
					stop_send = 0;
					pulse_state = 0;
					time_delay = 100;
//					switch (param_change)
//					{
//					case 0: PID_set_konstanta(&hbldc1.hpid_id, 0.3, 0.00001, 0); break;
//					case 1: PID_set_konstanta(&hbldc1.hpid_id, 0.3, 0.00002, 0); break;
//					case 2: PID_set_konstanta(&hbldc1.hpid_id, 0.3, 0.00003, 0); break;
//					case 3: PID_set_konstanta(&hbldc1.hpid_id, 0.3, 0.00004, 0); break;
//					case 4: PID_set_konstanta(&hbldc1.hpid_id, 0.3, 0.00005, 0); break;
//					case 5: PID_set_konstanta(&hbldc1.hpid_id, 0.3, 0.00006, 0); break;
//					case 6: PID_set_konstanta(&hbldc1.hpid_id, 0.3, 0.00007, 0); break;
//					case 7: PID_set_konstanta(&hbldc1.hpid_id, 0.3, 0.00008, 0); break;
//					case 8: PID_set_konstanta(&hbldc1.hpid_id, 0.3, 0.00009, 0); break;
//					case 9: PID_set_konstanta(&hbldc1.hpid_id, 0.3, 0.0001, 0); break;
//					}
					param_change++;
					param_time = HAL_GetTick();
				}
			}
			BLDC_get_sector (&hbldc1);
			BLDC_current_control(&hbldc1, 0, q, hbldc1.sector_theta + 90);
			dlpf_get_result(&i_test[0], hbldc1.id_result);
			dlpf_get_result(&i_test[1], hbldc1.iq_result);
			break;
		case _current_test:
			BLDC_set_speed (100);
			dlpf_get_result(&i_test[0], hbldc1.hdlpf_current_filt[_u].result);
			dlpf_get_result(&i_test[1], hbldc1.hdlpf_current_filt[_v].result);
			dlpf_get_result(&i_test[2], hbldc1.hdlpf_current_filt[_w].result);
			if (HAL_GetTick() != usb_tx_t)
			{
			  usb_tx_t = HAL_GetTick();
			  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f %.3f\n",
					  i_test[0].result * 1000, i_test[1].result * 1000, i_test[2].result * 1000);
			  CDC_Transmit_FS (usb_tx_buff, ln);
			}
			break;
		case _speed_test:
//			if (HAL_GetTick() - usb_tx_t >= 20)
//			{
//			  usb_tx_t = HAL_GetTick();
//			  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f %.3f\n",
//					  rpm_sp, i_test[0].result, hbldc1.hpid_omega.error);
//			  CDC_Transmit_FS (usb_tx_buff, ln);
//			}
			if (!stop_send)
			{
				if (HAL_GetTick() - param_time >= time_delay)
				{
					param_time = HAL_GetTick();
					if (pulse_state == 0)
					{
						pulse_state = 1;
						time_delay = 10000;
//						rpm_sp = 100;
					}
					else
					{
						stop_send = 1;
						rpm_sp = 0;
					}
				}
			}
			else
			{
				if (next_change_param)
				{
					next_change_param = 0;
					stop_send = 0;
					pulse_state = 0;
					time_delay = 100;
//					switch (param_change)
//					{
//					case 0: PID_set_konstanta(&hbldc1.hpid_omega, 0.006, 0.0000002, 0); break;
//					case 1: PID_set_konstanta(&hbldc1.hpid_omega, 0.006, 0.0000004, 0); break;
//					case 2: PID_set_konstanta(&hbldc1.hpid_omega, 0.006, 0.0000006, 0); break;
//					case 3: PID_set_konstanta(&hbldc1.hpid_omega, 0.006, 0.0000008, 0); break;
//					case 4: PID_set_konstanta(&hbldc1.hpid_omega, 0.006, 0.000001, 0); break;
//					case 5: PID_set_konstanta(&hbldc1.hpid_omega, 0.006, 0.0000012, 0); break;
//					case 6: PID_set_konstanta(&hbldc1.hpid_omega, 0.006, 0.0000014, 0); break;
//					case 7: PID_set_konstanta(&hbldc1.hpid_omega, 0.006, 0.0000016, 0); break;
//					case 8: PID_set_konstanta(&hbldc1.hpid_omega, 0.006, 0.0000018, 0); break;
//					case 9: PID_set_konstanta(&hbldc1.hpid_omega, 0.006, 0.000002, 0); break;
//					}
					switch (param_change)
					{
					case 0: rpm_sp = 10; break;
					case 1: rpm_sp = 20; break;
					case 2: rpm_sp = 30; break;
					case 3: rpm_sp = 40; break;
					case 4: rpm_sp = 50; break;
					case 5: rpm_sp = 60; break;
					case 6: rpm_sp = 70; break;
					case 7: rpm_sp = 80; break;
					case 8: rpm_sp = 90; break;
					case 9: rpm_sp = 100; break;
					case 10: rpm_sp = 200; break;
					case 11: rpm_sp = 300; break;
					case 12: rpm_sp = 400; break;
					case 13: rpm_sp = 500; break;
					case 14: rpm_sp = 0; break;
					}
					param_change++;
					if (param_change > 14) param_change = 0;
					param_time = HAL_GetTick();
				}
			}
			BLDC_set_angle (0);
			BLDC_set_speed (rpm_sp);
			dlpf_get_result(&i_test[0], hbldc1.rpm);
			break;
		case _torque_test: break;
		case _angle_test: break;
		}
	}
}

void BLDC2_testing (BLDC_TestTypedef mode)
{
	float iq_sp = 0.5, q = 0;
	_Bool pulse_state = 0;
	uint32_t param_time = HAL_GetTick(), time_delay = 100, param_change = 0;
	float angle_sp = 0;
	_Bool stop_send = 1;
	while (1)
	{
		switch (mode)
		{
		case _dq_test:
			if (!stop_send)
			{
				if (HAL_GetTick() - param_time >= time_delay)
				{
					param_time = HAL_GetTick();
					if (pulse_state == 0)
					{
						pulse_state = 1;
						time_delay = 2000;
						q = iq_sp;
					}
					else
					{
						stop_send = 1;
						q = 0;
					}
				}
				if (HAL_GetTick() != usb_tx_t)
				{
				  usb_tx_t = HAL_GetTick();
				  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f %.3f\n",
						  q, i_test[0].result, i_test[1].result);
				  CDC_Transmit_FS (usb_tx_buff, ln);
				}
			}
			else
			{
				if (next_change_param)
				{
					next_change_param = 0;
					stop_send = 0;
					pulse_state = 0;
					time_delay = 100;
//					switch (param_change)
//					{
//					case 0: PID_set_konstanta(&hbldc2.hpid_id, 0.002, 0.0001, 0); break;
//					case 1: PID_set_konstanta(&hbldc2.hpid_id, 0.002, 0.0002, 0); break;
//					case 2: PID_set_konstanta(&hbldc2.hpid_id, 0.002, 0.0003, 0); break;
//					case 3: PID_set_konstanta(&hbldc2.hpid_id, 0.002, 0.0004, 0); break;
//					case 4: PID_set_konstanta(&hbldc2.hpid_id, 0.002, 0.0005, 0); break;
//					case 5: PID_set_konstanta(&hbldc2.hpid_id, 0.002, 0.0006, 0); break;
//					case 6: PID_set_konstanta(&hbldc2.hpid_id, 0.002, 0.0007, 0); break;
//					case 7: PID_set_konstanta(&hbldc2.hpid_id, 0.002, 0.0008, 0); break;
//					case 8: PID_set_konstanta(&hbldc2.hpid_id, 0.002, 0.0009, 0); break;
//					case 9: PID_set_konstanta(&hbldc2.hpid_id, 0.002, 0.001, 0); break;
//					}
					param_change++;
					param_time = HAL_GetTick();
				}
			}
			angle_sector = (raw_angle+mag_angle_offset) * 11.0;
			BLDC_current_control (&hbldc2, 0, q, angle_sector-90);
			dlpf_get_result(&i_test[0], hbldc2.id_result);
			dlpf_get_result(&i_test[1], hbldc2.iq_result);
			break;

		case _current_test:
			angle_sector = (raw_angle+mag_angle_offset) * 11.0;
			BLDC_current_control (&hbldc2, 0, 0.3, angle_sector-90);
			dlpf_get_result(&i_test[0], hbldc2.hdlpf_current_filt[_u].result);
			dlpf_get_result(&i_test[1], hbldc2.hdlpf_current_filt[_v].result);
			dlpf_get_result(&i_test[2], hbldc2.hdlpf_current_filt[_w].result);
			if (HAL_GetTick() != usb_tx_t)
			{
			  usb_tx_t = HAL_GetTick();
			  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f %.3f\n",
					  i_test[0].result * 1000, i_test[1].result * 1000, i_test[2].result * 1000);
			  CDC_Transmit_FS (usb_tx_buff, ln);
			}
			break;
		case _speed_test:
			break;
		case _torque_test:
			break;
		case _angle_test:
			if (!stop_send)
			{
				if (HAL_GetTick() - param_time >= time_delay)
				{
					param_time = HAL_GetTick();
					if (pulse_state == 0)
					{
						pulse_state = 1;
						time_delay = 2000;
						angle_sp = 45;
					}
					else
					{
						stop_send = 1;
						angle_sp = 0;
					}
				}
				if (HAL_GetTick() - usb_tx_t >= 20)
				{
				  usb_tx_t = HAL_GetTick();
				  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f\n",
						  angle_sp, angle_sens);
				  CDC_Transmit_FS (usb_tx_buff, ln);
				}
			}
			else
			{
				if (next_change_param)
				{
					next_change_param = 0;
					stop_send = 0;
					pulse_state = 0;
					time_delay = 100;
					switch (param_change)
					{
					case 0: PID_set_konstanta(&hbldc2.hpid_theta, 1.5, 0, 0.001); break;
					case 1: PID_set_konstanta(&hbldc2.hpid_theta, 1.5, 0, 0.002); break;
					case 2: PID_set_konstanta(&hbldc2.hpid_theta, 1.5, 0, 0.003); break;
					case 3: PID_set_konstanta(&hbldc2.hpid_theta, 1.5, 0, 0.004); break;
					case 4: PID_set_konstanta(&hbldc2.hpid_theta, 1.5, 0, 0.005); break;
					case 5: PID_set_konstanta(&hbldc2.hpid_theta, 1.5, 0, 0.006); break;
					case 6: PID_set_konstanta(&hbldc2.hpid_theta, 1.5, 0, 0.007); break;
					case 7: PID_set_konstanta(&hbldc2.hpid_theta, 1.5, 0, 0.008); break;
					case 8: PID_set_konstanta(&hbldc2.hpid_theta, 1.5, 0, 0.009); break;
					case 9: PID_set_konstanta(&hbldc2.hpid_theta, 1.5, 0, 0.01); break;
					}
					param_change++;
//					if (param_change > 5) param_change = 0;
					param_time = HAL_GetTick();
				}
			}
			BLDC_set_angle (angle_sp);
			BLDC_set_speed (0);
			break;
		}
	}
}

void TA_percobaan (BLDC_TestTypedef mode, BLDC_HandleTypeDef *hbldc)
{
	float iq_sp = 0, rpm_sp = 0, angle_sp = 0;
	uint32_t param_time = HAL_GetTick(), time_delay = 100, param_change = 0;
	_Bool stop_send = 1;
	while (1)
	{
		switch (mode)
		{
		case _current_test:
			if (!stop_send)
			{
				if (HAL_GetTick() - param_time >= time_delay)
				{
					param_time = HAL_GetTick();

//					iq_sp+=0.01;
//					if (iq_sp > 3.0)
//					{
//						iq_sp = 0;
//						stop_send = 1;
//					}
					iq_sp = 0;
					stop_send = 1;

//					param_change++;
//					if (param_change > 7)
//					{
//						param_change = 0;
//						iq_sp = 0;
//						stop_send = 1;
//					}
				}
			}
			else
			{
				if (next_change_param)
				{
					next_change_param = 0;
					stop_send = 0;
					time_delay = 5000;
					param_change = 0;
					iq_sp = 1.0;
					param_time = HAL_GetTick();
				}
			}

			if (hbldc->channel == _BLDC1)
			{
				BLDC_get_sector (&hbldc1);
				BLDC_current_control(&hbldc1, 0, iq_sp, hbldc1.sector_theta + 90);
			}
			else
			{
				angle_sector = (raw_angle+mag_angle_offset) * 11.0;//11.0
				BLDC_current_control (&hbldc2, 0, iq_sp, angle_sector+90);
			}
			dlpf_get_result(&i_test[0], hbldc->id_result);//hbldc1.id_result
			dlpf_get_result(&i_test[1], hbldc->iq_result);
			if (HAL_GetTick() - usb_tx_t >= 1)
			{
			  usb_tx_t = HAL_GetTick();
			  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f %.3f\n",
					  iq_sp*1000, hbldc->id_result*1000, hbldc->iq_result*1000);
			  CDC_Transmit_FS (usb_tx_buff, ln);
			}
			break;
		case _speed_test:
			if (!stop_send)
			{
				if (HAL_GetTick() - param_time >= time_delay)
				{
					param_time = HAL_GetTick();
//					rpm_sp--;
//					if (rpm_sp < -800)
//					{
//						rpm_sp = 0;
//						stop_send = 1;
//					}
//					rpm_sp = 0;
//					stop_send = 1;

					switch (param_change)
					{
					case 0:
						rpm_sp = 200;
						angle_sp = 0;
					break;
					case 1:
						rpm_sp = 200;
						angle_sp = -45;
					break;
					case 2:
						rpm_sp = 200;
						angle_sp = 100;
					break;
					case 3:
						rpm_sp = 200;
						angle_sp = -100;
					break;
					case 4:
						rpm_sp = 250;
						angle_sp = 180;
					break;
					case 5:
						rpm_sp = 200;
						angle_sp = -90;
					break;
					}
					optimalisasi_gerakan (angle_sp, rpm_sp);

					param_change++;
					if (param_change > 5)
					{
						param_change = 0;
						rpm_sp = 0;
						angle_sp = 0;
						stop_send = 1;
					}
				}
				if (HAL_GetTick() - usb_tx_t >= 10)
				{
				  usb_tx_t = HAL_GetTick();
				  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f %.3f %.3f\n",
						  rpm_sp, i_test[0].result, angle_sp, angle_sens);
				  CDC_Transmit_FS (usb_tx_buff, ln);
				}
			}
			else
			{
				if (next_change_param)
				{
					next_change_param = 0;
					rpm_sp = 0;
					angle_sp = 0;
					stop_send = 0;
					param_change = 0;
					time_delay = 2000;//10
					param_time = HAL_GetTick();
				}
			}
			BLDC_set_angle (data_angle);
			BLDC_set_speed (data_rpm);
			dlpf_get_result(&i_test[0], hbldc1.rpm);
			break;
		case _angle_test:
			if (!stop_send)
			{
				if (HAL_GetTick() - param_time >= time_delay)
				{
					param_time = HAL_GetTick();

					angle_sp-=180;
//					if (angle_sp >= 1)
//					{
//						angle_sp = 0;
//						stop_send = 1;
//					}

					param_change++;
					if (param_change > 1)
					{
						param_change = 0;
						angle_sp = 0;
						stop_send = 1;
					}
				}
			}
			else
			{
				if (next_change_param)
				{
					next_change_param = 0;
					stop_send = 0;
					time_delay = 1000;
					param_change = 0;
					angle_sp = 0;//0.025
					param_time = HAL_GetTick();
				}
			}

			BLDC_set_angle (angle_sp);
			BLDC_set_speed (0);
			if (HAL_GetTick() - usb_tx_t >= 10)
			{
			  usb_tx_t = HAL_GetTick();
			  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f\n",
					  angle_sp, angle_sens);
			  CDC_Transmit_FS (usb_tx_buff, ln);
			}
			break;
		}
	}
}


//============================================================================================

void zero_steer ()
{
	float angle_steer_call = angle_sens;
	uint32_t t_steer_call = HAL_GetTick(), t_validate;
	t_validate = HAL_GetTick();
	wheel_state = 1;
	while (wheel_state)
	{
		if (HAL_GetTick() != t_steer_call)
		{
		  t_steer_call = HAL_GetTick();
		  angle_steer_call+=0.1;
		}
		if (!steer_call_ready)
		{
		  if (HAL_GetTick() - t_validate > 1000) steer_call_ready = 1;
		}
		BLDC_set_angle (angle_steer_call);
		BLDC_set_speed (0);

	}
}
//		dlpf_get_result(&i_test[0], hbldc2.hdlpf_current_filt[_u].result);
//		dlpf_get_result(&i_test[1], hbldc2.hdlpf_current_filt[_v].result);
//		dlpf_get_result(&i_test[2], hbldc2.hdlpf_current_filt[_w].result);
//		if (HAL_GetTick() != usb_tx_t)
//		{
//		  usb_tx_t = HAL_GetTick();
//		  uint32_t ln = sprintf ((char*)usb_tx_buff, "%.3f %.3f %.3f\n",
//				  i_test[0].result * 1000, i_test[1].result * 1000, i_test[2].result * 1000);
//		  CDC_Transmit_FS (usb_tx_buff, ln);
//		}

//============================================================================================

void wheel_init ()
{
#ifdef WHEEL_Z
	wheel_addr = 0x101;
	zero_offset = 0;
	mag_angle_offset = -29.5601177;
#else
#ifdef WHEEL_X
	wheel_addr = 0x100;
	zero_offset = 0;
	mag_angle_offset = -16.78;
#else
#ifdef WHEEL_0
	wheel_addr = 0x200;
	zero_offset = -55.0;
	mag_angle_offset = -2.4;
#else
#ifdef WHEEL_1
	wheel_addr = 0x211;
	zero_offset = 64.0; //-175.0
	mag_angle_offset = -50.6;//-42.0
#else
#ifdef WHEEL_2
	wheel_addr = 0x222;
	zero_offset = -180.0;
	mag_angle_offset = -16.78;//-26.04
#else
#error "Pilih antara WHEEL_0, WHEEL_1, atau WHEEL_2!"
#endif
#endif
#endif
#endif
#endif

	hbldc1.channel = _BLDC1;
	hbldc2.channel = _BLDC2;
	hbldc1.last_sector_theta = hbldc1.sector_theta;

	CAN_filter_config ();
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	HAL_Delay(200);
	FLASH_set_sector_addrs (FLASH_SECTOR_11, 0x080E0000);
	//  flash_data_buff[0] = (int32_t)(a*100000);
	//  flash_data_buff[1] = (int32_t)(b*100000);
	//  FLASH_write(0, flash_data_buff, 2, DATA_TYPE_32);
	FLASH_read(0, flash_data_buff, 2, DATA_TYPE_32);
//	mag_angle_offset =  (float)(int32_t)flash_data_buff[0] / 100000;
//	b =  (float)(int32_t)flash_data_buff[1] / 100000;
}

//============================================================================================

uint32_t tcnt1, tcnt2;
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC2)
	{
		hbldc1.adc_buff[0] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
		hbldc1.adc_buff[1] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
		hbldc1.adc_buff[2] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
		BLDC_get_current (&hbldc1);
	}
	if (hadc->Instance == ADC3)
	{
		hbldc2.adc_buff[0] = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
		hbldc2.adc_buff[1] = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);
		hbldc2.adc_buff[2] = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3);
		BLDC_get_current (&hbldc2);
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			BLDC_get_rpm (&hbldc1);
			BLDC_get_sector (&hbldc1);
			BLDC_rpm_sens (&hbldc1);
			hbldc1.zero_det_t = HAL_GetTick();
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		cnt_ovf++;
	}
	if (htim->Instance == TIM7)
	{
		if (read_ready)
		{
//			HAL_TIM_Base_Stop_IT(&htim7);
		  if (AS5048A_send_data (1, 1, 0x3fff) == HAL_OK)
			  read_ready = 0;
		}
	}
	if (htim->Instance == TIM4)
	{
		counting_time ();
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
	{
		encd_data_rx = spi_rx[0] << 8 | spi_rx[1];
		error_flag = (encd_data_rx>>14) & 1;
		if (!error_flag)
		{
			uint16_t val = (encd_data_rx & (0x3fff))>>0;//4
//			raw_angle = val * 720.0 / 1023.0;
			raw_angle = (double)val * 720.0 / (double)0x3fff;//1023.0
			if (raw_angle - last_raw_angle < -300)
			{
				raw_angle_ovf++;
			}
			else if (raw_angle - last_raw_angle > 300)
			{
				raw_angle_ovf--;
			}
			angle_sens = (raw_angle + raw_angle_ovf * 360.0)*-0.1172108178559791463 - steer_angle_offset;
			z_lift = (raw_angle + raw_angle_ovf * 360.0)*0.00222222222222222222222222222222 - z_offset;
			last_raw_angle = raw_angle;
		}
		TIM7->CNT = 0;
		read_ready = 1;
		SPI_CS_GPIO_Port->BSRR = SPI_CS_Pin;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Z_STEER_DET_Pin)
	{
		if (wheel_state && steer_call_ready)
		{
			steer_angle_offset = angle_sens + zero_offset;
			wheel_state = 0;
		}
	}
}

//==================================================================================

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, can_rx_buff);
	if (RxHeader.StdId == wheel_addr)
	{
		switch (can_rx_buff[0])
		{
		case _SET_WHEEL:
			CAN_get_wheel ();
			CAN_send_wheel (wheel_addr, angle_sens, hall_sens_count);
			hall_sens_count = 0;
			break;
		case _SET_ZERO_OFFSET:
			CAN_get_zero_offset ();
			CAN_send_ok (wheel_addr);
			break;
		case 0x12:
			CAN_get_z ();
			break;
		}
		led_can_respone = can_rx_buff[0];
	}
}

//235.77713
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  wheel_init ();

  BLDC_init (&hbldc1);
  BLDC_init (&hbldc2);
  BLDC_get_sector (&hbldc1);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  HAL_TIM_Base_Start_IT(&htim7);
  AS5048A_send_data (1, 1, 0x3fff);
  BLDC_beep(&hbldc2, 3136, 100);
  HAL_Delay(100);
  BLDC_beep(&hbldc2, 3136, 100);
  HAL_Delay(100);
  BLDC_beep(&hbldc2, 3136, 100);
  HAL_Delay(100);
//  TA_percobaan (_angle_test, &hbldc2);
//  TA_percobaan (_speed_test, &hbldc1);
//  BLDC1_testing (_speed_test);
//  BLDC2_testing (_angle_test);//_dq_test
#ifdef WHEEL_Z
  lifter_goto_zero ();
#else
  zero_steer ();
#endif
//  data_rpm = 100;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  zero_mosfet ();
	  blink_respone ();

	  if (mag_zero_set_flag)
	  {
		  mag_zero_set_flag = 0;
		  BLDC2_zero_cal ();
	  }

#ifdef WHEEL_Z
//	  if (HAL_GetTick() - timer_test0 >= 5000)
//	  {
//		  timer_test0 = HAL_GetTick();
//		  switch (test_state)
//		  {
//		  case 0: cm = 10; break;
//		  case 1: cm = 20; break;
//		  case 2: cm = 30; break;
//		  case 3: cm = 0; break;
//		  }
//		  test_state++;
//		  if (test_state > 3) test_state = 0;
//	  }
	  BLDC_set_z (z_cm);
	  if (fabs (z_cm - z_lift) <= 0.2)
	  {
		  if (!lifter_ok_flag)
		  {
			  lifter_ok_flag = 1;
			  CAN_send_z (wheel_addr, z_lift);
		  }
	  }
	  else if (fabs (z_cm - z_lift) > 0.5)
	  {
		  if (lifter_ok_flag)
			  lifter_ok_flag = 0;
	  }
#else
//	  angle_sector = (raw_angle+mag_angle_offset) * 11.0;
//	  BLDC_current_control (&hbldc2, 0, 1.0, 0);

	  BLDC_set_angle (data_angle);//data_angle
	  BLDC_set_speed (data_rpm);//data_rpm
#endif
//	  angle_sector = (raw_angle+mag_angle_offset) * 7;
//	  BLDC_current_control (&hbldc2, 0, 0.1, angle_sector+120);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_13;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_FALLING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = 3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_10;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_FALLING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = 3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim1.Init.Period = 4095;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 167;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 167;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 167;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 199;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim8.Init.Period = 4095;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin|M2_EN_U_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M2_EN_V_Pin|M2_EN_W_Pin|M1_EN_U_Pin|M1_EN_V_Pin
                          |M1_EN_W_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin M2_EN_U_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|M2_EN_U_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M2_EN_V_Pin M2_EN_W_Pin M1_EN_U_Pin M1_EN_V_Pin
                           M1_EN_W_Pin */
  GPIO_InitStruct.Pin = M2_EN_V_Pin|M2_EN_W_Pin|M1_EN_U_Pin|M1_EN_V_Pin
                          |M1_EN_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Z_STEER_DET_Pin */
  GPIO_InitStruct.Pin = Z_STEER_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Z_STEER_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Z_LIMIT_Pin */
  GPIO_InitStruct.Pin = Z_LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Z_LIMIT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
