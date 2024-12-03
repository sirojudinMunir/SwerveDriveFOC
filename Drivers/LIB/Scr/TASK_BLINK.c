/*
 * TASK_BLINK.c
 *
 *  Created on: Dec 2, 2024
 *      Author: munir
 */

#include "TASK_BLINK.h"
#include "main.h"
#include "SWERVE_DRIVE_CAN.h"

osThreadId_t blink_task_handle;
const osThreadAttr_t blink_task_attributes = {
  .name = "blink_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

_Bool led_blink_flag = 1;
uint32_t led_blink_time, led_blink_delay;
uint8_t led_blink_count = 0;

/*
 * @brief	blink_respone
 * 			indikator LED jika perangkat menerima data yang sesuai
 * @param	None
 * @retval	None
 */
void blink_respone (void)
{
	if (led_can_respone != 0)
	{
		if (led_blink_flag)
		{
			led_blink_flag = 0;
			led_blink_time = HAL_GetTick();
			switch (led_can_respone)
			{
			case _SET_WHEEL:
				led_blink_delay = 30;
				led_blink_count = 2;
				break;
			case _SET_ZERO_OFFSET:
				led_blink_delay = 100;
				led_blink_count = 4;
				break;
			case _SET_PID_M1:
				led_blink_delay = 150;
				led_blink_count = 6;
				break;
			case _SET_PID_M2:
				led_blink_delay = 150;
				led_blink_count = 6;
				break;
			}
		}
		if (HAL_GetTick() - led_blink_time >= led_blink_delay)
		{
			led_blink_time = HAL_GetTick();
			if (led_blink_count > 0)
			{
				if (led_blink_count % 2 == 0)
					LED_BUILTIN_GPIO_Port->BSRR = LED_BUILTIN_Pin<<16;
				else
					LED_BUILTIN_GPIO_Port->BSRR = LED_BUILTIN_Pin;
				led_blink_count--;
			}
			else
			{
				led_can_respone = 0;
				led_blink_flag = 1;
			}
		}
	}
}

/**
  * @brief  Function implementing the start_blink_task thread.
  * @param  argument: Not used
  * @retval None
  */
void start_blink_task(void *argument)
{
	/* Infinite loop */
	for(;;)
	{
//		blink_respone ();
		HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		osDelay(500);
	}
}
