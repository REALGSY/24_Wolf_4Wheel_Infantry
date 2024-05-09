/**
  * @brief  上位机测试函数
  * @param	None
  * @retval None
  */
	
#include "Task_Check.h" 
#include "FreeRTOSConfig.h"
#include "typedef.h"
#include "DR16_Remote.h"
#include "DevicesCheck.h"
#include "DEV_CIMU.h"
#include "Robot_control.h"
#include "DEV_Power_Meter.h" 
#include "Dev_SupCap.h"
void DEV_Check(void const *argument)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(200); //每2毫秒强制进入总控制

	for(;;)
	{
		Check_DR16();                     //DR16检测
		DJI_C_IMUFUN.Check_DJI_C_IMU();   //C板陀螺仪检测检测
		if(rc.OffLineFlag == 1)
		{
			DR16_Export_Data.Alldead = 1;
			Dev_Check.Offline_Check();
		}
		else
		{
			DR16_Export_Data.Alldead = 0;
		}
		Check_Power_Meter();
		Check_SupCap();
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}
