#include "Task_CanMsg.h"
#include "typedef.h"
#include "Debug_DataScope.h"
#include "Task_Debug.h"
#include "INS_task.h"
#include "DR16_Remote.h"
#include "Control_Vision.h"
#include "M2006_Motor.h"
#include "M6020_Motor.h"
void Debug(void const * argument)
{
  /* USER CODE BEGIN Debug */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(200);  //每十毫秒强制进入总控制
  /* Infinite loop */
  for(;;)
  {
		Control_Vision_FUN.Check_Vision();
		M2006_FUN.Check_M2006();
		DR16_Fun.Check_DR16();
		Check_M6020();		
//		Debug_addData(INS_angle[0],2);
//		Debug_addData(INS_angle[1],3);
//		Debug_addData(INS_angle[1],4);
//		Debug_show(5);
//    osDelay(15);
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
  /* USER CODE END Debug */
}
