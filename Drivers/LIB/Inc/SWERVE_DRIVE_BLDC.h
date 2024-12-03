/*
 * SWERVE_DRIVE_BLDC.h
 *
 *  Created on: Dec 2, 2024
 *      Author: munir
 */

#ifndef LIB_INC_SWERVE_DRIVE_BLDC_H_
#define LIB_INC_SWERVE_DRIVE_BLDC_H_

#include "SWERVE_DRIVE_FOC.h"

extern BLDC_HandleTypeDef hbldc1, hbldc2;
extern double angle_sens, zero_offset, mag_angle_offset;
extern int16_t hall_sens_count;

void BLDC_beep (BLDC_HandleTypeDef *hbldc, uint32_t freq, uint32_t time_delay);

void BLDC_get_sector (BLDC_HandleTypeDef *hbldc);

void BLDC_rpm_sens (BLDC_HandleTypeDef *hbldc);

void BLDC_get_rpm (BLDC_HandleTypeDef *hbldc);

void BLDC_set_speed (double rpm);

void BLDC_set_angle (float deg);

void BLDC_calibrate (void);

void zero_mosfet (void);

void BLDC2_zero_cal (void);


#endif /* LIB_INC_SWERVE_DRIVE_BLDC_H_ */
