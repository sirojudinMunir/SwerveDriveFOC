/*
 * COMMAND.c
 *
 *  Created on: Nov 23, 2024
 *      Author: munir
 */

#include "TASK_COMMAND.h"
#include "usb_device.h"
#include "FLASH_lib.h"
#include "PID_lib.h"
#include "SWERVE_DRIVE_FOC.h"

/*
 * extern from main.c
 */
extern _Bool mag_zero_set_flag;
extern uint8_t usb_tx_buff[300];
extern uint32_t usb_tx_lenght;
extern BLDC_HandleTypeDef hbldc1, hbldc2;
extern USB_settingTypedef usb_setting;

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

osThreadId_t command_task_handle;
const osThreadAttr_t command_task_attributes = {
  .name = "command_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};

uint8_t usb_tx_buff[300];

_Bool next_change_param = 0, flash_save_flag = 0, usb_msg_flag = 0;
double x_kp, x_ki, x_kd;
uint32_t offset_index = 0;

_Bool get_flash_save_flag (void){
	return flash_save_flag;
}

void set_flash_save_flag (_Bool state){
	flash_save_flag = state;
}

_Bool get_usb_msg_flag (void){
	return usb_msg_flag;
}

void set_usb_msg_flag (_Bool state){
	usb_msg_flag = state;
}

double str2float (char *str, uint8_t ln)
{
	double result;
	double num[2]={0, 0}, num2_dev = 10;
	uint8_t start_num2;
	_Bool coma_det = 0, minus_det = 0;

	if (str[0] == '-') minus_det = 1;
	for (uint8_t i = minus_det; i < ln; i++){
		if ((str[i] >= '0' && str[i] <= '9') || str[i] == '.' || str[i] == ','){
			if (!coma_det){
				if (str[i] == '.' || str[i] == ','){
					coma_det = 1;
					start_num2 = i+1;
				}
				else{
					if (i != 0) num[0] *= 10;
					num[0] += (str[i] - '0');
				}
			}
			else
			{
				if (str[i] == '.' || str[i] == ','){
					return 0;
				}
				else{
					if (i != start_num2){
						num[1] *= 10;
						num2_dev *= 10;
					}
					num[1] += (str[i] - '0');
				}
			}
		}
		else{
			return 0;
		}
	}
	result = num[0] + num[1]/num2_dev;
	if (minus_det) result = -result;
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
						PID_set_konstanta(&WHEELED_handler.hpid_omega, x_kp, x_ki, x_kd);
//						PID_set_konstanta(&WHEELED_handler.hpid_id, x_kp, x_ki, 0);
//						PID_set_konstanta(&WHEELED_handler.hpid_iq, x_kp, x_ki, 0);
					}
					else if (cmd[1] == '2')
					{
						PID_set_konstanta(&STEERING_handler.hpid_theta, x_kp, x_ki, x_kd);
					}
				}
			}
		}
	}
	else if (cmd[0] == 'N' || cmd[0] == 'n')
	{
		next_change_param = 1;
	}
}

void wrong_cmd_message (){
	uint32_t str_ln;
	uint8_t str[20];
	str_ln = sprintf ((char*)str, "Incorrect Command!\n");
	CDC_Transmit_FS (str, str_ln);
}

uint32_t get_param_info (BLDC_HandleTypeDef *hbldc, uint8_t *str){
	uint32_t str_ln = 0;

	if (hbldc->channel == BLDC_STEERING){
		str_ln = sprintf ((char*)str, "Steering Motor Info:\n");
	}
	else if (hbldc->channel == BLDC_WHEELED){
		str_ln = sprintf ((char*)str, "Wheeled Motor Info:\n");
	}
	str_ln += sprintf ((char*)str+str_ln, "- MAX Current: %fA\n", hbldc->max_current);
	str_ln += sprintf ((char*)str+str_ln, "- Direct Current Control:\n\tKp: %f\n\tKi: %f\n",
			hbldc->hpid_id.kp, hbldc->hpid_id.ki);
	str_ln += sprintf ((char*)str+str_ln, "- Quadrature Current Control:\n\tKp: %f\n\tKi: %f\n",
			hbldc->hpid_iq.kp, hbldc->hpid_iq.ki);
	if (hbldc->channel == BLDC_WHEELED){
		str_ln += sprintf ((char*)str+str_ln, "- Speed Control:\n\tKp: %f\n\tKi: %f\n",
				hbldc->hpid_omega.kp, hbldc->hpid_omega.ki);
	}
	else if (hbldc->channel == BLDC_STEERING){
		str_ln += sprintf ((char*)str+str_ln, "- Position Control:\n\tKp: %f\n\tKd: %f\n",
				hbldc->hpid_theta.kp, hbldc->hpid_theta.kd);
	}
	str_ln += sprintf ((char*)str+str_ln, "- Rotor Offset: %fdeg\n", hbldc->rotor_offset);
	if (hbldc->channel == BLDC_STEERING){
		str_ln += sprintf ((char*)str+str_ln, "- Angle Offset: %fdeg\n", hbldc->angle_offset);
	}
	return str_ln;
}

void cmd_feedback_message (void){
	uint32_t str_ln = 0;
	if (usb_setting.mode == _info){
		if (usb_setting.motor == _steering){
			str_ln = get_param_info (&STEERING_handler, usb_tx_buff);
		}
		else if (usb_setting.motor == _wheeled){
			str_ln = get_param_info (&WHEELED_handler, usb_tx_buff);
		}
	}
	else{
		if (usb_setting.mode == _set) {
			if (usb_setting.cmd != _cmd_none){
				str_ln = sprintf ((char*)usb_tx_buff, "Successfully Entering New Parameters!\n");
			}
		}
		switch (usb_setting.cmd)
		{
		case _max_cur:
			if (usb_setting.motor == _steering)
			{
				str_ln += sprintf ((char*)usb_tx_buff+str_ln, "Steering Motor MAX Current: %fA\n", STEERING_handler.max_current);
			}
			else
			{
				str_ln += sprintf ((char*)usb_tx_buff+str_ln, "Wheeled Motor MAX Current: %fA\n", WHEELED_handler.max_current);
			}
			break;
		case _d_ctrl_pi:
			if (usb_setting.motor == _steering)
			{
				str_ln += sprintf ((char*)usb_tx_buff+str_ln, "Steering Motor Direct Current Control:\n\tKp: %f\n\tKi: %f\n",
						 STEERING_handler.hpid_id.kp, STEERING_handler.hpid_id.ki);
			}
			else
			{
				str_ln += sprintf ((char*)usb_tx_buff+str_ln, "Wheeled Motor Direct Current Control:\n\tKp: %f\n\tKi: %f\n",
						 WHEELED_handler.hpid_id.kp, WHEELED_handler.hpid_id.ki);
			}
			break;
		case _q_ctrl_pi:
			if (usb_setting.motor == _steering)
			{
				str_ln += sprintf ((char*)usb_tx_buff+str_ln, "Steering Motor Quadrature Current Control:\n\tKp: %f\n\tKi: %f\n",
						 STEERING_handler.hpid_iq.kp, STEERING_handler.hpid_iq.ki);
			}
			else
			{
				str_ln += sprintf ((char*)usb_tx_buff+str_ln, "Wheeled Motor Quadrature Current Control:\n\tKp: %f\n\tKi: %f\n",
						 WHEELED_handler.hpid_iq.kp, WHEELED_handler.hpid_iq.ki);
			}
			break;
		case _speed_ctrl_pi:
			if (usb_setting.motor == _wheeled)
			{
				str_ln += sprintf ((char*)usb_tx_buff+str_ln, "Wheeled Motor Speed Control:\n\tKp: %f\n\tKi: %f\n",
						 WHEELED_handler.hpid_omega.kp, WHEELED_handler.hpid_omega.ki);
			}
			break;
		case _angle_ctrl_pd:
			if (usb_setting.motor == _steering)
			{
				str_ln += sprintf ((char*)usb_tx_buff+str_ln, "Steering Motor Position Control:\n\tKp: %f\n\tKd: %f\n",
						 STEERING_handler.hpid_theta.kp, STEERING_handler.hpid_theta.kd);
			}
			break;
		case _rotor_angle_offset:
			if (usb_setting.motor == _steering)
			{
				str_ln += sprintf ((char*)usb_tx_buff+str_ln, "Steering Motor Rotor Offset: %fdeg\n", STEERING_handler.rotor_offset);
			}
			else
			{
				str_ln += sprintf ((char*)usb_tx_buff+str_ln, "Wheeled Motor Rotor Offset: %fdeg\n", WHEELED_handler.rotor_offset);
			}
			break;
		case _steering_zero_offset:
			if (usb_setting.motor == _steering)
			{
				str_ln += sprintf ((char*)usb_tx_buff+str_ln, "Steering Motor Angle Offset: %fdeg\n", STEERING_handler.angle_offset);
			}
			break;
		default:
			wrong_cmd_message ();
			break;
		}
	}
	CDC_Transmit_FS (usb_tx_buff, str_ln);
}

uint32_t count_separator (char *str, char separator_char){
	uint32_t separator = 0;
	for (uint32_t i = 0; str[i] == separator_char; i++){
		separator++;
	}
	return separator;
}

uint32_t find_separator (char *str, char separator_char){
	uint32_t separator = 0;
	for (uint32_t i = 0; str[i] != separator_char; i++){
		separator++;
	}
	return separator;
}

uint32_t count_float_number (char *str){
	uint32_t count = 0;
	for (uint16_t i = 0; (str[i] >= '0' && str[i] <= '9') || str[i] == ',' || str[i] == '.' || str[i] == '-'; i++)
	{
		count++;
	}
	return count;
}
///WM SET DCTR = 0.03 0.00012
int cmd_set (char *cmd)
{
	BLDC_cmdMotorTypedef motor = _cmd_none;
	BLDC_cmdModeTypedef mode = _mode_none;
	BLDC_cmdTypedef cmd_temp = _motor_none;
	int error_result = 0;
	uint8_t param_ln = 1;
	uint32_t val_ln = 0;
	double val[3] = {0, 0, 0};

	offset_index = 0;

	offset_index += count_separator (cmd, ' ');
	if (str_compare (cmd+offset_index, "WM", 2)) motor = _wheeled;
	else if (str_compare (cmd+offset_index, "SM", 2)) motor = _steering;

	if (motor != _motor_none)
	{
		offset_index += find_separator (cmd+offset_index, ' ');
		offset_index += count_separator (cmd+offset_index, ' ');
		if (str_compare (cmd+offset_index, "SET", 3)) mode = _set;
		else if (str_compare (cmd+offset_index, "GET", 3)) mode = _get;
		else if (str_compare (cmd+offset_index, "INFO", 4)) mode = _info;

		if (mode == _set || mode == _get)
		{
			offset_index += find_separator (cmd+offset_index, ' ');
			offset_index += count_separator (cmd+offset_index, ' ');
			if (str_compare (cmd+offset_index, "CLIM", 4)) cmd_temp = _max_cur;
			else if (str_compare (cmd+offset_index, "DCTR", 4)) cmd_temp = _d_ctrl_pi;
			else if (str_compare (cmd+offset_index, "QCTR", 4)) cmd_temp = _q_ctrl_pi;
			else if (str_compare (cmd+offset_index, "SCTR", 4)) cmd_temp = _speed_ctrl_pi;
			else if (str_compare (cmd+offset_index, "ACTR", 4)) cmd_temp = _angle_ctrl_pd;
			else if (str_compare (cmd+offset_index, "RAO", 3)) cmd_temp = _rotor_angle_offset;
			else if (str_compare (cmd+offset_index, "ZO", 2)) cmd_temp = _steering_zero_offset;

			if (cmd_temp != _cmd_none)
			{
				if (mode == _set)
				{
					offset_index += find_separator (cmd+offset_index, '=')+1;
					param_ln = 0;
					while (1){
						if (param_ln > 0){
							offset_index += find_separator (cmd+offset_index, ' ');
						}
						offset_index += count_separator (cmd+offset_index, ' ');
						val_ln = count_float_number (cmd+offset_index);
						if (val_ln == 0) break;
						val[param_ln] = str2float (cmd+offset_index, val_ln);
//						offset_index += val_ln;
						param_ln++;
						if (param_ln > 2) break;
					}

					if (error_result == 0)
					{
						switch (cmd_temp)
						{
						case _max_cur:
							if (motor == _steering) STEERING_handler.max_current = val[0];
							else WHEELED_handler.max_current = val[0];
							break;
						case _d_ctrl_pi:
							if (motor == _steering)
							{
								STEERING_handler.hpid_id.kp = val[0];
								STEERING_handler.hpid_id.ki = val[1];
							}
							else
							{
								WHEELED_handler.hpid_id.kp = val[0];
								WHEELED_handler.hpid_id.ki = val[1];
							}
							break;
						case _q_ctrl_pi:
							if (motor == _steering)
							{
								STEERING_handler.hpid_iq.kp = val[0];
								STEERING_handler.hpid_iq.ki = val[1];
							}
							else
							{
								WHEELED_handler.hpid_iq.kp = val[0];
								WHEELED_handler.hpid_iq.ki = val[1];
							}
							break;
						case _speed_ctrl_pi:
							if (motor == _wheeled)
							{
								WHEELED_handler.hpid_omega.kp = val[0];
								WHEELED_handler.hpid_omega.ki = val[1];
							}
							break;
						case _angle_ctrl_pd:
							if (motor == _steering)
							{
								STEERING_handler.hpid_theta.kp = val[0];
								STEERING_handler.hpid_theta.kd = val[1];
							}
							break;
						case _rotor_angle_offset:
							if (motor == _steering) STEERING_handler.rotor_offset = val[0];
							else WHEELED_handler.rotor_offset = val[0];
							break;
						case _steering_zero_offset:
							if (motor == _steering) STEERING_handler.angle_offset = val[0];
							else return -2;
							break;
						default:
							break;
						}
					}
				}
			}
			else {
				wrong_cmd_message ();
				return -1;
			}
		}
		else if (mode == _mode_none){
			wrong_cmd_message ();
			return -1;
		}
	}
	else {
		wrong_cmd_message ();
		return -1;
	}

	if (mode == _set) {
		if (cmd_temp != _cmd_none){
			set_flash_save_flag (1);
		}
	}

	usb_setting.motor = motor;
	usb_setting.mode = mode;
	usb_setting.cmd = cmd_temp;
	cmd_feedback_message ();
	set_usb_msg_flag (1);
	return 0;
}


/**
  * @brief  Function implementing the start_command_task thread.
  * @param  argument: Not used
  * @retval None
  */
void start_command_task(void *argument)
{
	/* Infinite loop */
	for(;;)
	{
		if (get_flash_save_flag ()){
		  set_flash_save_flag (0);
		  flash_save_data ();
		}
		if (get_usb_msg_flag ()){
		  set_usb_msg_flag (0);
		}
		osDelay(1);
	}
}
