/*
 * SWERVE_DRIVE_BLDC.h
 *
 *  Created on: Dec 2, 2024
 *      Author: munir
 */

#ifndef LIB_INC_SWERVE_DRIVE_BLDC_H_
#define LIB_INC_SWERVE_DRIVE_BLDC_H_

#include "SWERVE_DRIVE_FOC.h"

#define STEERING_POLE 11.0

extern float angle_sens, mag_angle_offset;
extern int16_t hall_sens_count;

void BLDC_beep (BLDC_HandleTypeDef *hbldc, uint32_t freq, uint32_t time_delay);

void BLDC_get_sector (BLDC_HandleTypeDef *hbldc);

void BLDC_rpm_sens (BLDC_HandleTypeDef *hbldc);

void BLDC_get_rpm (BLDC_HandleTypeDef *hbldc);

void BLDC_speed_control (void);

void BLDC_angle_control (void);

void zero_steer (void);

void wheeled_motor_set_speed (float speed);

void steering_motor_set_angle (float angle);

float wheeled_motor_get_speed_setpoint ();

float steering_motor_get_angle_set_point ();

void BLDC_calibrate (void);

void zero_mosfet (void);

void BLDC2_zero_cal (void);

void battery_read_init (void);

float read_battery (void);

#endif /* LIB_INC_SWERVE_DRIVE_BLDC_H_ */
