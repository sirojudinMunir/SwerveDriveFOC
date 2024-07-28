/*
 * FLASH_lib.h
 *
 *  Created on: Dec 19, 2023
 *      Author: munir
 */

#ifndef LIB_INC_FLASH_LIB_H_
#define LIB_INC_FLASH_LIB_H_

#include "stm32f4xx_hal.h"

//Typedefs
//1. data size
typedef enum
{
	DATA_TYPE_8=0,
	DATA_TYPE_16,
	DATA_TYPE_32,
}DataTypeDef;

void FLASH_erase_sector(void);
void FLASH_set_sector_addrs(uint8_t sector, uint32_t addrs);
void FLASH_write(uint32_t idx, void *wrBuf, uint32_t Nsize, DataTypeDef dataType);
void FLASH_read(uint32_t idx, void *rdBuf, uint32_t Nsize, DataTypeDef dataType);

#endif /* LIB_INC_FLASH_LIB_H_ */
