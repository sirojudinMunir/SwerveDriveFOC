/*
 * MAGNETIC_SENSOR_AS5048A.h
 *
 *  Created on: Dec 7, 2024
 *      Author: munir
 */

#ifndef LIB_INC_MAGNETIC_SENSOR_AS5048A_H_
#define LIB_INC_MAGNETIC_SENSOR_AS5048A_H_

#include "main.h"

HAL_StatusTypeDef AS5048A_send_data (_Bool par, _Bool rw, uint16_t addr);

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

#endif /* LIB_INC_MAGNETIC_SENSOR_AS5048A_H_ */
