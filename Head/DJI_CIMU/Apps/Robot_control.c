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
#include "WS2812.h"
#include "DR16_Remote.h"
#include "Shot.h"
#include "tim.h"
Robot_t Robot;

void Robot_Disable(void)
{
		Robot.ChassisWorkMode = ChassisWorkMode_Disable;
	  Robot.Device_FrictMode=FrictWorkMode_Disable;
}

void Cloud_control(void)
{
	Cloud_processing(DR16_Export_Data.Robot_TargetValue.Left_Right_Value, DR16_Export_Data.Robot_TargetValue.Forward_Back_Value, DR16_Export_Data.Robot_TargetValue.Yaw_Value,DR16_Export_Data.Robot_TargetValue.Pitch_Value);
}

void Chassis_Follow_control(void)
{
	Chassis_processing(DR16_Export_Data.Robot_TargetValue.Left_Right_Value, DR16_Export_Data.Robot_TargetValue.Forward_Back_Value, DR16_Export_Data.Robot_TargetValue.Yaw_Value,DR16_Export_Data.Robot_TargetValue.Pitch_Value);
}

void Shot_control (void)
{
	roll_shot_go();
}

/**
	* @brief  更改机器人控制来源
 * @param	void
 * @retval None
 */
void Robot_ChangeControlSource(ControlSource_e controlSource)
{
    if (Robot.ControlSource != controlSource) //发生模式跳变，重置。
    {
        Robot_Reset();
    }
    Robot.ControlSource = controlSource;
}

/**
 * @brief  状态复位
 * @param	void
 * @retval None
 */
void Robot_Reset(void)
{
    //重新初始化机器人。
    Robot.Sport_CloudWorkMode = CloudWorkMode_Normal;
    Robot.Device_FrictMode = FrictWorkMode_Disable;
}

void Robot_control (void)
{
	ws2812_init(8);
	RGB_Control();
	Cloud_control();   				//云台控制
	Chassis_Follow_control(); //底盘控制
	Shot_control();						//发射控制
}

/**
 * @brief  	    机器人重启(软重启)
 * @param[in]	None
 * @retval 	    None
 */
void Robot_Soft_Reset(void)
{   
    //--- 芯片复位
    __set_FAULTMASK(1);    //关闭所有中断
    HAL_NVIC_SystemReset();//复位
}
