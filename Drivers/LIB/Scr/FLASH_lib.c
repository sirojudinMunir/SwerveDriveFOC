/*
 * FLASH_lib.c
 *
 *  Created on: Dec 19, 2023
 *      Author: munir
 */
#include "FLASH_lib.h"
#include "SWERVE_DRIVE_FOC.h"

extern uint16_t wheel_addr;
extern BLDC_HandleTypeDef WHEELED_handler, STEERING_handler;

static uint32_t flash_sector_addr;
static uint8_t flash_sector_num;

uint32_t flash_data_buff[20];

void FLASH_erase_sector(void)
{
	HAL_FLASH_Unlock();
	//Erase the required Flash sector
	FLASH_Erase_Sector(flash_sector_num, FLASH_VOLTAGE_RANGE_3);
	//Lock the Flash space
	HAL_FLASH_Lock();
}

void FLASH_set_sector_addrs(uint8_t sector, uint32_t addrs)
{
	flash_sector_num = sector;
	flash_sector_addr = addrs;
}

void FLASH_write(uint32_t idx, void *wrBuf, uint32_t Nsize, DataTypeDef dataType)
{
	uint32_t flashAddress = flash_sector_addr + idx;
	
	//Erase sector before write
	FLASH_erase_sector();
	//Unlock Flash
	HAL_FLASH_Unlock();
	//Write to Flash
	switch(dataType)
	{
		case DATA_TYPE_8:
				for(uint32_t i=0; i<Nsize; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flashAddress , ((uint8_t *)wrBuf)[i]);
					flashAddress++;
				}
			break;
		
		case DATA_TYPE_16:
				for(uint32_t i=0; i<Nsize; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flashAddress , ((uint16_t *)wrBuf)[i]);
					flashAddress+=2;
				}
			break;
		
		case DATA_TYPE_32:
				for(uint32_t i=0; i<Nsize; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress , ((uint32_t *)wrBuf)[i]);
					flashAddress+=4;
				}
			break;
	}
	//Lock the Flash space
	HAL_FLASH_Lock();
}

void FLASH_read(uint32_t idx, void *rdBuf, uint32_t Nsize, DataTypeDef dataType)
{
	uint32_t flashAddress = flash_sector_addr + idx;
	
	switch(dataType)
	{
		case DATA_TYPE_8:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint8_t *)rdBuf + i) = *(uint8_t *)flashAddress;
					flashAddress++;
				}
			break;
		
		case DATA_TYPE_16:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint16_t *)rdBuf + i) = *(uint16_t *)flashAddress;
					flashAddress+=2;
				}
			break;
		
		case DATA_TYPE_32:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint32_t *)rdBuf + i) = *(uint32_t *)flashAddress;
					flashAddress+=4;
				}
			break;
	}
}


void flash_save_data (void)
{
	LED_BUILTIN_GPIO_Port->BSRR = LED_BUILTIN_Pin<<16;

	flash_data_buff[0] = wheel_addr;

	flash_data_buff[1] = (uint32_t)(WHEELED_handler.max_current*1000000);
	flash_data_buff[2] = (uint32_t)(WHEELED_handler.hpid_id.kp*100000000);
	flash_data_buff[3] = (uint32_t)(WHEELED_handler.hpid_id.ki*100000000);
	flash_data_buff[4] = (uint32_t)(WHEELED_handler.hpid_iq.kp*100000000);
	flash_data_buff[5] = (uint32_t)(WHEELED_handler.hpid_iq.ki*100000000);
	flash_data_buff[6] = (uint32_t)(WHEELED_handler.hpid_omega.kp*100000000);
	flash_data_buff[7] = (uint32_t)(WHEELED_handler.hpid_omega.ki*100000000);
	flash_data_buff[8] = (int32_t)(WHEELED_handler.rotor_offset*1000000);

	flash_data_buff[9] = (uint32_t)(STEERING_handler.max_current*1000000);
	flash_data_buff[10] = (uint32_t)(STEERING_handler.hpid_id.kp*100000000);
	flash_data_buff[11] = (uint32_t)(STEERING_handler.hpid_id.ki*100000000);
	flash_data_buff[12] = (uint32_t)(STEERING_handler.hpid_iq.kp*100000000);
	flash_data_buff[13] = (uint32_t)(STEERING_handler.hpid_iq.ki*100000000);
	flash_data_buff[14] = (uint32_t)(STEERING_handler.hpid_theta.kp*100000000);
	flash_data_buff[15] = (uint32_t)(STEERING_handler.hpid_theta.kd*100000000);
	flash_data_buff[16] = (int32_t)(STEERING_handler.rotor_offset*1000000);
	flash_data_buff[17] = (int32_t)(STEERING_handler.angle_offset*1000000);

	FLASH_write(0, flash_data_buff, 18, DATA_TYPE_32);

	LED_BUILTIN_GPIO_Port->BSRR = LED_BUILTIN_Pin;
}

void flash_get_data (void)
{
	FLASH_read(0, flash_data_buff, 18, DATA_TYPE_32);

	wheel_addr = (uint8_t)flash_data_buff[0];

	WHEELED_handler.max_current = (double)flash_data_buff[1]/1000000.0;
	WHEELED_handler.hpid_id.kp = (double)flash_data_buff[2]/100000000.0;
	WHEELED_handler.hpid_id.ki = (double)flash_data_buff[3]/100000000.0;
	WHEELED_handler.hpid_iq.kp = (double)flash_data_buff[4]/100000000.0;
	WHEELED_handler.hpid_iq.ki = (double)flash_data_buff[5]/100000000.0;
	WHEELED_handler.hpid_omega.kp = (double)flash_data_buff[6]/100000000.0;
	WHEELED_handler.hpid_omega.ki = (double)flash_data_buff[7]/100000000.0;
	WHEELED_handler.rotor_offset = (double)(int32_t)flash_data_buff[8]/1000000.0;

	STEERING_handler.max_current = (double)flash_data_buff[9]/1000000.0;
	STEERING_handler.hpid_id.kp = (double)flash_data_buff[10]/100000000.0;
	STEERING_handler.hpid_id.ki = (double)flash_data_buff[11]/100000000.0;
	STEERING_handler.hpid_iq.kp = (double)flash_data_buff[12]/100000000.0;
	STEERING_handler.hpid_iq.ki = (double)flash_data_buff[13]/100000000.0;
	STEERING_handler.hpid_theta.kp = (double)flash_data_buff[14]/100000000.0;
	STEERING_handler.hpid_theta.kd = (double)flash_data_buff[15]/100000000.0;
	STEERING_handler.rotor_offset = (double)(int32_t)flash_data_buff[16]/1000000.0;
	STEERING_handler.angle_offset = (double)(int32_t)flash_data_buff[17]/1000000.0;
}

void set_default_motor_param (void)
{
	FLASH_read(0, flash_data_buff, 18, DATA_TYPE_32);

	wheel_addr = (uint8_t)flash_data_buff[0];

	WHEELED_handler.max_current = 5.0;
	PID_set_konstanta(&WHEELED_handler.hpid_id, 0.02, 0.001, 0);
	PID_set_konstanta(&WHEELED_handler.hpid_iq, 0.02, 0.002, 0);
	PID_set_konstanta(&WHEELED_handler.hpid_omega, 0.002, 0.000002, 0);
	WHEELED_handler.rotor_offset = 0.0;

	STEERING_handler.max_current = 5.0;
	PID_set_konstanta(&STEERING_handler.hpid_id, 0.02, 0.002, 0);
	PID_set_konstanta(&STEERING_handler.hpid_iq, 0.02, 0.002, 0);
	PID_set_konstanta(&STEERING_handler.hpid_theta, 1.0, 0, 0);
	STEERING_handler.rotor_offset = 0.0;
	STEERING_handler.angle_offset = 0.0;
}

