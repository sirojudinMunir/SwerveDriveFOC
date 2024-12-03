/*
 * SWERVE_DRIVE_CAN.h
 *
 *  Created on: Dec 2, 2024
 *      Author: munir
 */

#ifndef LIB_INC_SWERVE_DRIVE_CAN_H_
#define LIB_INC_SWERVE_DRIVE_CAN_H_

#include "main.h"

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


extern CAN_TxHeaderTypeDef   	TxHeader;
extern uint8_t can_rx_buff[8], can_tx_buff[8], led_can_respone;
extern uint16_t wheel_addr;
extern int16_t hall_sens_count;
extern double angle_sens, zero_offset;
extern float data_angle, data_rpm;

void CAN_filter_config (void);
void CAN_get_wheel (void);
void CAN_get_zero_offset (void);
void CAN_send_wheel (uint32_t addr, float angle, int16_t hall_cnt);
void CAN_send_ok (uint32_t addr);


#endif /* LIB_INC_SWERVE_DRIVE_CAN_H_ */
