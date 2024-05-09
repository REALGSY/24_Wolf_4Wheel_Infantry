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
#include "Status_Update.h"
#include "DR16_Remote.h"
#include "Robot_control.h"
#include "Monitor_RM_CAN.h"
#include "Shot.h"
#include "Chassis_control.h"
#include "SpeedRamp.h"
#include "USER_Filter.h"
#include "Control_Vision.h"
#include "Cloud_control.h"
#include "M6020_Motor.h"
#include "calibrate_task.h"
int stop = 0;
int om_gogo = 0;
int Max_Speed = 659;
uint16_t Reset_cnt;
int8_t Energy_Mechanism_Flag = 0;
int Half_Ship_Flag = 0;  //舵全模式
//底盘遥控前后斜坡
SpeedRamp_t ChassisRamp_ForwardBack = ForwardBackGroundInit;
#undef ForwardBackGroundInit

//底盘遥控左右斜坡
SpeedRamp_t ChassisRamp_LeftRight = LeftRightGroundInit;
#undef LeftRightGroundInit

//底盘遥控左右转斜坡
SpeedRamp_t ChassisRamp_Rotate = RotateGroundInit;
#undef RotateGroundInit

void RemoteMode_Update(void)
{
	if(DR16_Export_Data.OffLineFlag == 0)
	{
    switch(DR16_Export_Data.ControlSwitch->Left)
	{
	case RemotePole_UP:  
		read_start_imu();//更新当前陀螺仪角度 避免切换模式的时候疯
		if(DR16_Export_Data.ControlSwitch->Right == RemotePole_MID)  //左上右中 发射模式  拨轮打上打开摩擦轮
		{      
			Robot.ChassisWorkMode = ChassisWorkMode_CloudFollow;
			Robot_ChangeControlSource(ControlSource_RC);
			Robot.Device_FrictMode = FrictWorkMode_HighSpeed; 
			onlineyaw = 1;
			To_mode = 6;
		}
		else if(DR16_Export_Data.ControlSwitch->Right == RemotePole_UP)  //左上右上 电脑控制模式
		{
			if(Energy_Mechanism_Flag == 1)
			{
				Robot.ChassisWorkMode = ChassisWorkMode_Lock;
			}
			else if(Half_Ship_Flag == 1)
			{
				Robot.ChassisWorkMode = ChassisWorkMode_Square;
			}
			else
			{
				Robot.ChassisWorkMode = ChassisWorkMode_CloudFollow;
			}
			Robot_ChangeControlSource(ControlSource_PC);
		}
		else    //左上右下 视觉小陀螺
		{
			Robot.ChassisWorkMode = ChassisWorkMode_CloudFollow;
			Robot_ChangeControlSource(ControlSource_RC);
			Robot.Device_FrictMode = FrictWorkMode_HighSpeed; 
			onlineyaw = 1;
			To_mode = 5;
		}
	break;

	case RemotePole_MID:
			read_start_imu();//更新当前陀螺仪角度 避免切换模式的时候疯
	    Robot_ChangeControlSource(ControlSource_RC);	
			if(DR16_Export_Data.ControlSwitch->Right == RemotePole_MID) //左中右中 正常运动模式
		{
			onlineyaw = 0;
      Robot.ChassisWorkMode = ChassisWorkMode_CloudFollow;
			Robot.Device_FrictMode = FrictWorkMode_Disable;
		}
		else if(DR16_Export_Data.ControlSwitch->Right == RemotePole_UP)  //正常行走模式
		{
			onlineyaw = 0;
			Robot.ChassisWorkMode = ChassisWorkMode_Square;
			Robot.Device_FrictMode = FrictWorkMode_Disable;
		}
		else if(DR16_Export_Data.ControlSwitch->Right == RemotePole_DOWM)     //左中右下 底盘锁住
		{
			if(DR16.rc.ch3 != - 660)
			{
			  onlineyaw = 0;
				Robot.ChassisWorkMode = ChassisWorkMode_Lock;
				Robot.Device_FrictMode = FrictWorkMode_HighSpeed;
//				onlineyaw = 1;
//				Robot.ChassisWorkMode = ChassisWorkMode_Lock;
//				Robot.Device_FrictMode = FrictWorkMode_HighSpeed;
//				To_mode = 2;
			}
			else 
			{
				onlineyaw = 1;
				Robot.ChassisWorkMode = ChassisWorkMode_Lock;
				Robot.Device_FrictMode = FrictWorkMode_HighSpeed;
				To_mode = 2;
			}
		}
	break;

	case RemotePole_DOWM:
		a_start = 0;
	  onlineyaw = 0;
	  Robot_ChangeControlSource(ControlSource_RC);
		if(DR16_Export_Data.ControlSwitch->Right == RemotePole_DOWM)
		{
			Robot_Disable();
			if(DR16.rc.ch4_DW  == -660)
			{
				Reset_cnt++;
        if(Reset_cnt == 500)//--- 拨轮打上2s重启
        {
					Robot_Soft_Reset();
					//cali_sensor[0].cali_cmd = 1;
        }
			}
      else
      {
        Reset_cnt = 0;
      }
		}
		else
		{
			Robot_Disable();
		}
	break;
	}
 }
	else 
	{
		Robot_Disable();
	}
	

}

static int RampRate = 2;              //斜坡函数叠加值
static float RampRate_factor = 10.4f;       //斜坡函数叠加值
float Forward_Back_Value_Direction = 1; //改变前进方向
float Left_Right_Value_Direction = 1;   //改变左右方向
float Omega_Value_Direction = 1; 
static float Mouse_LpfAttFactor = 0.2;  //PC鼠标滤波系数
float Yaw_Value;
float Pitch_Value;
int Turn_Round_Flag = 0;
float Continuous_fire = 0;

void RemoteControl_PC_Update(void)  //键盘控制
{

    /************** 运动控制begin ******************/
	
    /************** 键盘操作 ******************/	

	  /************************** W S **********************/
	
    if (DR16_Fun.GetKeyMouseAction(KEY_W, KeyAction_PRESS)) //W
    {
			ChassisRamp_ForwardBack.rate = RampRate * RampRate_factor;
    }
    else if (DR16_Fun.GetKeyMouseAction(KEY_S, KeyAction_PRESS)) //S
    {
			ChassisRamp_ForwardBack.rate = -RampRate * RampRate_factor;
    }
    else
    {
			CountReset(&ChassisRamp_ForwardBack);
			ChassisRamp_ForwardBack.rate = 0;
    }

    /***************************A D **********************/
    if (DR16_Fun.GetKeyMouseAction(KEY_D, KeyAction_PRESS)) //D
    {
        ChassisRamp_LeftRight.rate = -RampRate * RampRate_factor;
    }
    else if (DR16_Fun.GetKeyMouseAction(KEY_A, KeyAction_PRESS)) //A
    {
        ChassisRamp_LeftRight.rate = RampRate * RampRate_factor;
    }
    else
    {
			  CountReset(&ChassisRamp_LeftRight);
        ChassisRamp_LeftRight.rate = 0;
    }

		if (DR16_Fun.GetKeyMouseAction(KEY_R, KeyAction_CLICK)) //R  开启摩擦轮
    {
			  if(Robot.Device_FrictMode == FrictWorkMode_Disable)
				{
					shot.Key_need = 0;
					Robot.Device_FrictMode = FrictWorkMode_HighSpeed;

				}
        else
				{
					Robot.Device_FrictMode = FrictWorkMode_HighSpeed;
				}
    }
		
		if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_R, KeyAction_CLICK))   //关闭摩擦轮
    {
        shot.Key_need = 0;
				Robot.Device_FrictMode = FrictWorkMode_Disable;
    }       
		
		if (DR16_Fun.GetKeyMouseAction(KEY_G, KeyAction_CLICK)) //G 开启弹舱
    {
			Open_Left = 1;
    }
		
		if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_G, KeyAction_CLICK))   //关闭弹舱
    {
			Open_Left = 0;
    }    
		
		if (DR16_Fun.GetKeyMouseAction(KEY_Q, KeyAction_CLICK)) //Q 舵全模式
    {
			if(Half_Ship_Flag == 0)
			{
				Half_Ship_Flag = 1;
			}
			else if (Half_Ship_Flag == 1)
			{
				Half_Ship_Flag = 0;
			}
    }

		
		if (DR16_Fun.GetKeyMouseAction(KEY_SHIFT, KeyAction_PRESS)) //SHIFT
    {
			DR16_Export_Data.Robot_TargetValue.Omega_Value = 660 * Omega_Value_Direction;
			Forward_Back_Value_Direction = 0.5;
			Left_Right_Value_Direction = 0.5;
    }
		else
		{
			DR16_Export_Data.Robot_TargetValue.Omega_Value = 0;
			Forward_Back_Value_Direction = 1;
			Left_Right_Value_Direction = 1;
		}

		if (DR16_Fun.GetKeyMouseAction(KEY_Z, KeyAction_PRESS)) //Z(打符模式)
    {
			if (DR16_Fun.GetKeyMouseAction(KEY_SHIFT, KeyAction_PRESS)) //SHIFT
			{
				DR16_Export_Data.Robot_TargetValue.Omega_Value = 300 * Omega_Value_Direction;
				Forward_Back_Value_Direction = 0.5;
				Left_Right_Value_Direction = 0.5;
				Energy_Mechanism_Flag = 0;
			}
			else
			{
				Energy_Mechanism_Flag = 1;
			}
    }
		else
		{
			Energy_Mechanism_Flag = 0;
		}
		
		if(Last_Fine != To_mode)
		{
			shot.Key_need = 0;   
		}
		
		Last_Fine = To_mode;
		
		if (DR16_Fun.GetKeyMouseAction(MOUSE_Right, KeyAction_PRESS))
   	{
			onlineyaw = 1;
    }
		else
		{
			onlineyaw = 0;
		}
		
    DR16_Export_Data.Robot_TargetValue.Forward_Back_Value = 1 * Forward_Back_Value_Direction * SpeedRampCalc(&ChassisRamp_ForwardBack);
    DR16_Export_Data.Robot_TargetValue.Left_Right_Value = -1 * Left_Right_Value_Direction * SpeedRampCalc(&ChassisRamp_LeftRight);

		/************** 减速!!! ******************/
		if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_W, KeyAction_PRESS))
    {
        DR16_Export_Data.Robot_TargetValue.Forward_Back_Value = Forward_Back_Value_Direction * 130;
    }
    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_S, KeyAction_PRESS))
    {
        DR16_Export_Data.Robot_TargetValue.Forward_Back_Value = Forward_Back_Value_Direction * -130;
    }
    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_A, KeyAction_PRESS))
    {
        DR16_Export_Data.Robot_TargetValue.Left_Right_Value = -1 *Left_Right_Value_Direction * 130;
    }
    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_D, KeyAction_PRESS))
    {
        DR16_Export_Data.Robot_TargetValue.Left_Right_Value = -1 *Left_Right_Value_Direction * -130;
    }
		/************** 鼠标操作 ******************/
		
	  Filter_IIRLPF(&DR16_Export_Data.mouse.x, &Yaw_Value, Mouse_LpfAttFactor); //陀螺仪在线时底盘跟随云台
    DR16_Export_Data.Robot_TargetValue.Yaw_Value = Yaw_Value *400;
		
    Filter_IIRLPF(&DR16_Export_Data.mouse.y, &Pitch_Value, Mouse_LpfAttFactor);
    DR16_Export_Data.Robot_TargetValue.Pitch_Value = - Pitch_Value *60;
		
		if (DR16_Fun.GetKeyMouseAction(MOUSE_Left, KeyAction_CLICK))
   	{
			shot.Key_need = 1;
    }
	  else if (DR16_Fun.GetKeyMouseAction(MOUSE_Left, KeyAction_LONG_PRESS))
   	{
			shot.Key_need = 1;
			Continuous_fire = 1;
    }
		else if(To_mode == 5)
		{
			Auto_Shot = 1;
		}
		else
		{
			Auto_Shot = 0;
			if(Continuous_fire == 1)
			{
				shot.Key_need = 0 ;
				Continuous_fire = 0;
			}
		}

		
		
		if (DR16_Fun.GetKeyMouseAction(MOUSE_Right, KeyAction_PRESS))
   	{
			if(DR16_Fun.GetKeyMouseAction(KEY_V, KeyAction_PRESS)) //v 小陀螺模式
			{
				onlineyaw = 1;
				To_mode = 5; //击打小陀螺模式
			}
			else if(DR16_Fun.GetKeyMouseAction(KEY_Z, KeyAction_PRESS)) //Z 打符模式
			{
				onlineyaw = 1;
				To_mode = 2; //击打小陀螺模式
			}
			else
			{
				onlineyaw = 1;
				To_mode = 6; //正常
			}
    }
		else
		{
			To_mode = 6; //正常
			onlineyaw = 0;
		}
	
		if (DR16_Fun.GetKeyMouseAction(KEY_C, KeyAction_PRESS)) //C 开超电
    {
			Sup_Cap = 1;
    }
		else
    {
			Sup_Cap = 0;
    }    
		
		if(DR16_Fun.GetKeyMouseAction(KEY_F, KeyAction_LONG_PRESS)) //长按F接触热量限制
		{
			FIRE_FIRE = 1;
		}
		else
		{
			FIRE_FIRE = 0;
		}
		
		if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_B, KeyAction_CLICK))  
    {
			Shoot_Down_Flag ++;
			if(Shoot_Down_Flag>3)
			Shoot_Down_Flag = 3;
    }    
		else if (DR16_Fun.GetKeyMouseAction(KEY_B, KeyAction_CLICK))
   	{
			Shoot_Down_Flag = 0;
    }
}



void RemoteControl_Update (void)
{
	RemoteMode_Update(); 
	Laser_Control();
}
