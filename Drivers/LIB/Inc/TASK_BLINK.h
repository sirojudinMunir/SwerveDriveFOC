/*
 * TASK_BLINK.h
 *
 *  Created on: Dec 2, 2024
 *      Author: munir
 */

#ifndef LIB_INC_TASK_BLINK_H_
#define LIB_INC_TASK_BLINK_H_

#include "cmsis_os.h"

void start_blink_task(void *argument);

extern osThreadId_t blink_task_handle;
extern const osThreadAttr_t blink_task_attributes;


#endif /* LIB_INC_TASK_BLINK_H_ */
