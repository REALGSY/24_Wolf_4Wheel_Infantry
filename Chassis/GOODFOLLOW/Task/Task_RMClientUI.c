#include "Task_RMClientUI.h"
#include "Task_Check.h" 
#include "FreeRTOSConfig.h"
#include "typedef.h"
#include "DR16_Remote.h"
#include "DevicesCheck.h"
#include "DEV_CIMU.h"
#include "Robot_control.h"
#include "RMClient_UI.h"
void RMClientUI_Send(void const *argument)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(20); //每2毫秒强制进入总控制

	for(;;)
	{
		UserDefined_UI(); 
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}
