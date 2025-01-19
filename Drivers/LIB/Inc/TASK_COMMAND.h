/*
 * COMMAND.h
 *
 *  Created on: Nov 23, 2024
 *      Author: munir
 */

#ifndef LIB_INC_TASK_COMMAND_H_
#define LIB_INC_TASK_COMMAND_H_

#include "stdio.h"
#include "cmsis_os.h"

#define MAX_LIST_CMD_PARAM 	11
#define MAX_LIST_CMD_MODE 	3
#define MAX_LIST_CMD_MOTOR 	4

typedef enum
{
	 _default, _max_cur, _d_ctrl_pi, _q_ctrl_pi, _speed_ctrl_pi, _angle_ctrl_pd,
	_rotor_angle_offset, _steering_zero_offset, _addr, _steering_angle, _wheeled_speed,

	_cmd_none,
}BLDC_cmdParamTypedef;

typedef enum
{
	_set, _get, _info,

	_mode_none,
}BLDC_cmdModeTypedef;

typedef enum
{
	_steering, _wheeled, _swerve, _help,

	_motor_none,
}BLDC_cmdMotorTypedef;

typedef struct{
	BLDC_cmdMotorTypedef motor;
	BLDC_cmdModeTypedef mode;
	BLDC_cmdParamTypedef cmd;
}USB_settingTypedef;

typedef struct{
	char cmd_str[20];
	uint32_t action;
}CMD_listTypedef;

_Bool get_flash_save_flag (void);
void set_flash_save_flag (_Bool state);
_Bool get_usb_msg_flag (void);
void set_usb_msg_flag (_Bool state);
double str2float (char *str, uint8_t ln);
_Bool str_compare (char *str1, char *str2, uint32_t ln);
void usb_motor_calibrate (char *cmd);
void pid_setting (char *cmd);
void cmd_feedback_message (void);
int cmd_set (char *cmd);

void start_command_task(void *argument);

extern osThreadId_t command_task_handle;
extern const osThreadAttr_t command_task_attributes;

extern uint8_t usb_tx_buff[500];

#endif /* LIB_INC_COMMAND_H_ */
