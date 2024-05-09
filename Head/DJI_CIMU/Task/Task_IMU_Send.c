#include "FreeRTOSConfig.h"
#include "Task_IMU_Send.h"
#include "calibrate_task.h"
#include "INS_task.h"
#include "led_flow_task.h"
#include "Debug_DataScope.h"
#include "DJI_IMU.h"
#include "bsp_buzzer.h"
#include "DR16_Remote.h"
#include "Monitor_RM_CAN.h"
#include "Robot_control.h"
#include "Chassis_control.h"
#include "M6020_Motor.h"
#include "Cloud_control.h"
void IMU_Send(void const * argument)
{
  /* USER CODE BEGIN IMU_Send */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1);  //每十毫秒强制进入总控制
  /* Infinite loop */
  for(;;)
  {
	#if send_way == 0
		//欧拉角 
	if(cali_sensor[0].cali_done == CALIED_FLAG && cali_sensor[0].cali_cmd == 0)
	{
		Euler_Send_Fun(Euler_Send); //更新陀螺仪角度
		#endif
	}
	if(init_mode == false)
	{
		if(Robot.ChassisWorkMode == ChassisWorkMode_Square)
		{
			Yaw_error = ComputeMinOffset(M6020s[0].realAngle,1300);
		}
		else
		{
			Yaw_error = ComputeMinOffset(M6020s[0].realAngle,Chassis_tar);
		}
		
		if(Robot.ControlSource != ControlSource_PC && Robot.Device_FrictMode != FrictWorkMode_Disable)
		{
			DR16_Export_Data.Robot_TargetValue.Omega_Value = 0;
		}
    DR16_0x175_Can1_SendData(DR16_Export_Data.Robot_TargetValue.Left_Right_Value,DR16_Export_Data.Robot_TargetValue.Forward_Back_Value,Yaw_error,(int16_t)Robot.ChassisWorkMode);//第一位为摇杆左右移动标志位 第二位为前后移动标志位 第三位为底盘与yaw轴中心的误差值 第四位为底盘模式的发送
		DR16_0x185_Can1_SendData(DR16_Export_Data.Robot_TargetValue.Omega_Value ,0,Sup_Cap,0); //第一位为小陀螺数值 第二位预留摩擦轮模式 (已在底盘写出) 第三位为超级电容开启关闭标志位 第四位暂时空
	}
    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
  /* USER CODE END IMU_Send */
}
