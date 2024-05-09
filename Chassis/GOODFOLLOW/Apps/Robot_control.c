/**
 * @file Robot_control.c
 * @author Gsy 
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "Robot_control.h"
#include "Monitor_RM_CAN.h"
#include "Chassis_control.h"
#include "DR16_Remote.h"
#include "Shot.h"
#include "tim.h"
#include "RM_JudgeSystem.h"
#include "Dev_SupCap.h"

Robot_t Robot;
int Residual_Heat = 0;
int SupCap_Mix = 0;
void Robot_Disable(void)
{
		Robot.ChassisWorkMode = ChassisWorkMode_Disable;
	  Robot.Device_FrictMode=FrictWorkMode_Disable;
}

void Cloud_control(void)
{
	Cloud_processing(DR16_Export_Data.Robot_TargetValue.Left_Right_Value, DR16_Export_Data.Robot_TargetValue.Forward_Back_Value, DR16_Export_Data.Robot_TargetValue.Omega_Value,DR16_Export_Data.Robot_TargetValue.Pitch_Value);
}

void Chassis_Follow_control(void)
{
	Chassis_processing(DR16_Export_Data.Robot_TargetValue.Left_Right_Value, DR16_Export_Data.Robot_TargetValue.Forward_Back_Value, DR16_Export_Data.Robot_TargetValue.Omega_Value,DR16_Export_Data.Robot_TargetValue.Pitch_Value);
}

void Shot_control (void)
{
	roll_shot_go();
}

void Robot_control (void)
{
	//Cloud_control();   				//云台控制
	Chassis_Follow_control(); //底盘控制
	//Shot_control();						//发射控制
	Residual_Heat = ext_game_robot_state.data.shooter_id1_17mm_cooling_limit - ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;
	if(ext_game_robot_state.data.shooter_id1_17mm_cooling_limit == 65535)
	{
		Residual_Heat = 65535;
	}
	
	if(Supcap_Rec.RecvData.Pack.cap_cell >= 90)
	{
		SupCap_Mix = 3 ;
	}
	else if(Supcap_Rec.RecvData.Pack.cap_cell >= 70 && Supcap_Rec.RecvData.Pack.cap_cell < 90)
	{
		SupCap_Mix = 2 ;
	}
	else if(Supcap_Rec.RecvData.Pack.cap_cell >= 40 && Supcap_Rec.RecvData.Pack.cap_cell < 70)
	{
		SupCap_Mix = 1 ;
	}
	else if(Supcap_Rec.RecvData.Pack.cap_cell < 40 )
	{
		SupCap_Mix = 0 ;
	}
	
	Robomaster_0x088_Can1_SendData(ext_game_robot_state.data.robot_id,Residual_Heat,SupCap_Mix,30);
}
