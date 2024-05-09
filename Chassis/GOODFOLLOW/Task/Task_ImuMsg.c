/**
 * @file Task_ImuMsg.c
 * @author Gsy 
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Task_ImuMsg.h"
#include "IMU_Compensate.h"
#include "DJI_IMU.h"
#include "DEV_CIMU.h"
#include "Chassis_control.h"
void IMU_Receive_Run(void const * argument)
{
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(1); //每一毫秒强制进入总控制
	for(;;)
	{
		if(IMU_Init_Condition == 1)
		{
			IMU_CompensateFUN.IMU_GetData_Compensate();
	  	DJI_C_IMUFUN.Updata_Hand_Euler_Gyro_Data();
		}
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}
