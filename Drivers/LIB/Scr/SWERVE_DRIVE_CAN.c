/*
 * SWERVE_DRIVE_CAN.c
 *
 *  Created on: Dec 2, 2024
 *      Author: munir
 */

#include "SWERVE_DRIVE_CAN.h"
#include "math.h"

extern CAN_HandleTypeDef hcan1;

CAN_TxHeaderTypeDef   	TxHeader;
CAN_RxHeaderTypeDef   	RxHeader;

uint32_t TxMailbox;

uint8_t can_rx_buff[8], can_tx_buff[8], led_can_respone = 0;
uint16_t wheel_addr;


float data_angle = 0, data_rpm = 0;

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

/**********************************************************************************/

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
		}
		led_can_respone = can_rx_buff[0];
	}
}

