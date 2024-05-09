/**
 * @file Status_Update.c
 * @author Gsy
 * @brief 
 * @version 1
 * @date 2022-07-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
int stop = 0;
#include "Status_Update.h"
#include "DR16_Remote.h"
#include "Robot_control.h"
#include "Monitor_RM_CAN.h"
#include "Shot.h"
#include "Chassis_control.h"
void RemoteMode_Update(void)
{
	if(DR16_Export_Data.Alldead == 0)
	{
//    switch(DR16_Export_Data.ControlSwitch->Left)
//	{
//	case RemotePole_UP:
//		read_start_imu();//更新当前陀螺仪角度
//		if(DR16_Export_Data.ControlSwitch->Right == RemotePole_MID)
//		{
//			Robot.ChassisWorkMode = ChassisWorkMode_CloudFollow;
//			Robot.Device_FrictMode = FrictWorkMode_HighSpeed;
//		}
//		else if(DR16_Export_Data.ControlSwitch->Right == RemotePole_UP)
//		{
//			Robot.ChassisWorkMode = ChassisWorkMode_CloudOnly;
//			Robot.Device_FrictMode = FrictWorkMode_HighSpeed;
//		}
//		else
//		{
//			Robot_Disable();
//		}
//	break;

//	case RemotePole_MID:
//			read_start_imu();
//			if(DR16_Export_Data.ControlSwitch->Right == RemotePole_MID)
//		{
//      Robot.ChassisWorkMode = ChassisWorkMode_CloudFollow;
//			Robot.Device_FrictMode = FrictWorkMode_Disable;
//		}
//		else if(DR16_Export_Data.ControlSwitch->Right == RemotePole_UP)
//		{
//			Robot.ChassisWorkMode = ChassisWorkMode_CloudOnly;
//			Robot.Device_FrictMode = FrictWorkMode_Disable;
//		}
//		else
//		{
//			Robot_Disable();
//		}
//	break;

//	case RemotePole_DOWM:
//		a_start = 0;
//	  onlineyaw = 0;
//		if(DR16_Export_Data.ControlSwitch->Right == RemotePole_DOWM)
//		{
//			Robot_Disable();
//		}
//		else
//		{
//			Robot_Disable();
//		}
//	break;
//	}
 }
	else 
	Robot_Disable();
}


void RemoteControl_Update (void)
{
	  RemoteControl_Output();
	  RemoteMode_Update();
    
}
