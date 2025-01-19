/*
 * MAGNETIC_SENSOR_AS5048A.c
 *
 *  Created on: Dec 7, 2024
 *      Author: munir
 */

#include "MAGNETIC_SENSOR_AS5048A.h"

extern SPI_HandleTypeDef hspi1;
extern _Bool 	read_ready;
extern float	steer_angle_offset;

extern float 	angle_sens;

_Bool		error_flag;

uint8_t 	spi_rx[2];

uint16_t 	encd_data_rx;

int32_t 	raw_angle_ovf = 0;

float 		raw_angle,
			last_raw_angle;
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


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
	{
		encd_data_rx = spi_rx[0] << 8 | spi_rx[1];
		error_flag = (encd_data_rx>>14) & 1;
		if (!error_flag)
		{
			uint16_t val = (encd_data_rx & (0x3fff))>>0;//0
//			raw_angle = val * 720.0 / 1023.0;
			raw_angle = (float)val * 720.0 / (float)0x3fff;//1023.0 //0x3fff
			if (raw_angle - last_raw_angle < -300)
			{
				raw_angle_ovf++;
			}
			else if (raw_angle - last_raw_angle > 300)
			{
				raw_angle_ovf--;
			}
			angle_sens = (raw_angle + (double)raw_angle_ovf * 360.0)*-0.1172108178559791463 - steer_angle_offset;
			last_raw_angle = raw_angle;
		}
		TIM7->CNT = 0;
		read_ready = 1;
		SPI_CS_GPIO_Port->BSRR = SPI_CS_Pin;
	}
}
