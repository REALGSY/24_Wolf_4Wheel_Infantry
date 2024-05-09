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
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1);  //ÿʮ����ǿ�ƽ����ܿ���
  /* Infinite loop */
  for(;;)
  {
	#if send_way == 0
		//ŷ���� 
	if(cali_sensor[0].cali_done == CALIED_FLAG && cali_sensor[0].cali_cmd == 0)
	{
		Euler_Send_Fun(Euler_Send); //���������ǽǶ�
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
    DR16_0x175_Can1_SendData(DR16_Export_Data.Robot_TargetValue.Left_Right_Value,DR16_Export_Data.Robot_TargetValue.Forward_Back_Value,Yaw_error,(int16_t)Robot.ChassisWorkMode);//��һλΪҡ�������ƶ���־λ �ڶ�λΪǰ���ƶ���־λ ����λΪ������yaw�����ĵ����ֵ ����λΪ����ģʽ�ķ���
		DR16_0x185_Can1_SendData(DR16_Export_Data.Robot_TargetValue.Omega_Value ,0,Sup_Cap,0); //��һλΪС������ֵ �ڶ�λԤ��Ħ����ģʽ (���ڵ���д��) ����λΪ�������ݿ����رձ�־λ ����λ��ʱ��
	}
    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
  /* USER CODE END IMU_Send */
}
