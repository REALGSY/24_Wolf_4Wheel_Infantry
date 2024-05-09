#include "Task_Host.h"
#include "Status_Update.h"
#include "FreeRTOSConfig.h"
#include "Task_RobotControl.h"
#include "usart.h"
#include <stdio.h>
#include "IMU_Compensate.h"
#include "DEV_CIMU.h"
#include "Chassis_control.h"
#include "M6020_Motor.h"
#include "M3508_Motor.h"
#include "Control_Vision.h"
#include "Task_CanMsg.h"
void Host_Run(void const * argument)
{
	for(;;)
	{
//		Vision_classdef_SendToPC(&Send_Msg);
		//printf("begin:%f,%f,\n",kaerman,meika);
	//	printf("cmpu:%f \n",DJI_C_IMU.pitch);
		vTaskDelay(50);
	}
}
