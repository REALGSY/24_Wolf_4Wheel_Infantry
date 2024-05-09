/**
 * @file Task_RobotControl.c
 * @author Gsy
 * @brief 
 * @version 1
 * @date 2022-07-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Task_RobotControl.h"
#include "Status_Update.h"
#include "FreeRTOSConfig.h"
#include "Robot_control.h"
#include "stdio.h"
#include "tim.h"
#include "DEV_Buzzer.h"
void RobotControl(void const *argument)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //ÿ2����ǿ�ƽ����ܿ���
	for(;;)
	{
		RemoteControl_Update();
		Robot_control();
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}
