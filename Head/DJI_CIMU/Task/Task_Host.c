#include "Task_Host.h"
#include "Status_Update.h"
#include "FreeRTOSConfig.h"
#include "Task_RobotControl.h"
#include "usart.h"
#include <stdio.h>
#include "M2006_Motor.h"
#include "DR16_Remote.h"
#include "shot.h"
#include "Control_Vision.h"
#include "tim.h"
#include "Cloud_control.h"
void Host_Run(void const * argument)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(2);
	for(;;) 
	{
		Vision_classdef_SendToPC(&Send_Msg);
		//printf("pitch:%d,%d\n",test_1,test_2);
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}

