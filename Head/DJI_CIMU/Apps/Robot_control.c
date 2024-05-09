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
	* @brief  ���Ļ����˿�����Դ
 * @param	void
 * @retval None
 */
void Robot_ChangeControlSource(ControlSource_e controlSource)
{
    if (Robot.ControlSource != controlSource) //����ģʽ���䣬���á�
    {
        Robot_Reset();
    }
    Robot.ControlSource = controlSource;
}

/**
 * @brief  ״̬��λ
 * @param	void
 * @retval None
 */
void Robot_Reset(void)
{
    //���³�ʼ�������ˡ�
    Robot.Sport_CloudWorkMode = CloudWorkMode_Normal;
    Robot.Device_FrictMode = FrictWorkMode_Disable;
}

void Robot_control (void)
{
	ws2812_init(8);
	RGB_Control();
	Cloud_control();   				//��̨����
	Chassis_Follow_control(); //���̿���
	Shot_control();						//�������
}

/**
 * @brief  	    ����������(������)
 * @param[in]	None
 * @retval 	    None
 */
void Robot_Soft_Reset(void)
{   
    //--- оƬ��λ
    __set_FAULTMASK(1);    //�ر������ж�
    HAL_NVIC_SystemReset();//��λ
}
