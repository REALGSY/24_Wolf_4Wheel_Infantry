/**
 * @file Chassis_control.c
 * @author Gsy
 * @brief 
 * @version 1
 * @date 2022-11-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Cloud_control.h"
#include "Chassis_control.h"
#include "M6020_Motor.h"
#include "M3508_Motor.h"
#include "Monitor_RM_CAN.h"
#include "DJI_IMU.h"
#include "Control_Vision.h"
#include "DR16_Remote.h"
#include "SpeedRamp.h"
#include "math.h"
#include "USER_Filter.h"
#include "shot.h"
#include "tim.h"
#include "Status_Update.h"
/*******************************用户数据定义************************************/
int Cloud_Pitch_error;
int Cloud_Yaw_error;
float Last_Yaw_error;
float Last_Pitch_error;
int pitch_start ;
int onlineyaw = 0;
int last_onlineyaw = 0;
int change_online = 0;
int gogodeng = 0;
const float Cloud_Pitch_Min = 493 ;
 float Cloud_Pitch_Max = 1400;
uint16_t init_cnt = 0;
uint8_t init_mode = true;
float Yaw_Imu;
float Pitch_Imu;
int nmzhendesile = 1500;
/*******************************************************************************/

/************云台--陀螺仪PID***********/
cloud_positionpid_t    M6020s_YawOPID    = M6020s_YawOPIDInit;
cloud_positionpid_t    M6020s_YawIPID    = M6020s_YawIPIDInit;
cloud_positionpid_t    M6020s_PitchOPID  = M6020s_PitchOPIDInit;
cloud_positionpid_t    M6020s_PitchIPID  = M6020s_PitchIPIDInit;

cloud_positionpid_t    Vision_YawOPID    = Vision_YawOPIDInit;
cloud_positionpid_t    Vision_YawIPID    = Vision_YawIPIDInit;
cloud_positionpid_t    Vision_PitchOPID  = Vision_PitchOPIDInit;
cloud_positionpid_t    Vision_PitchIPID  = Vision_PitchIPIDInit;


/**********云台--陀螺仪PID END*******/

/**
 * @brief YAW轴电机跟随云台
 * 
 * @param target 
 * @param value 
 * @return int 
 */
int Open_Left = 0;
int Vision_Hold = 0;
int Last_Fine = 0;
int Cloud_Rest = 0;
int test_1;
int test_2;

int duoji1 = 0;
int duoji2 = 0;
int duoji3 = 0;
int duoji4 = 0;

int nmsl = 4000;
void YawWorkMode_Follow(float Vx, float Vy, float VOmega , float VPitch)
{

    //--- 等待IMU初始化完毕
  if(init_mode == true)
  {
		if(DJI_C_IMU.pitch != 0 || DJI_C_IMU.yaw != 0)
		{
    	init_cnt++;
    	if(init_cnt >= 500) //1300*2ms误差采样时间，开机后的陀螺仪是傻乎乎的，必须处理一下
        {
            read_start_imu();  //待测试
            init_mode = false;
					  a_start = 0 ;
        }
    }
    return;
  }
	
	if(M6020s_Yaw.OffLineFlag == 1)
	{
		Cloud_Rest = 1; 
	}
	
	if(M6020s_Yaw.OffLineFlag != 1 && Cloud_Rest == 1)
	{
		M6020s_Pitch.targetAngle =  22.7527f * DJI_C_IMU.pitch ;
		M6020s_Yaw.targetAngle =  22.7527f * DJI_C_IMU.total_yaw ;
		Cloud_Rest = 0;
	}
	
	if(Robot.ChassisWorkMode == ChassisWorkMode_CloudFollow || Robot.ChassisWorkMode == ChassisWorkMode_CloudOnly || Robot.ChassisWorkMode == ChassisWorkMode_Lock || Robot.ChassisWorkMode == ChassisWorkMode_Square)
	{
	if(DR16.rc.ch4_DW  == -660)    //开启弹舱等待
	{
		gogodeng ++;
		if(gogodeng > 200)
		{
			
			Open_Left = !Open_Left ;
			gogodeng = 0;
		}
	}
		
		
		if(Open_Left == 1)   //开启弹舱
		{
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 950);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 4800);
		}
		else
		{
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 4100);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1750);
		}
		
	if(onlineyaw !=last_onlineyaw)  //当切换模式时标记标记位
	{
		change_online = 1;
	}
			if (change_online == 1)     //切换模式的时候重新标记原点
		{
			pitch_start = 22.7527f * DJI_C_IMU.pitch;
			start_imu =  22.7527f * DJI_C_IMU.yaw;
		}
		 
		
	if(onlineyaw == 1)	
	{
		if(VisionData.RawData.enemy != 0 && VisionData.OffLineFlag == 1)
		{
		
//			Filter_IIRLPF(&VisionData.RawData.x, &M6020s_Yaw.rc_targetAngle, 0.2); //利用低通平滑一下视觉传来的数据 (还是算了)
//			Filter_IIRLPF(&VisionData.RawData.y, &M6020s_Pitch.rc_targetAngle, 0.2); //利用低通平滑一下视觉传来的数据
			M6020s_Yaw.rc_targetAngle = VisionData.RawData.x ;
			M6020s_Pitch.rc_targetAngle = VisionData.RawData.y ;

			M6020s_Pitch.targetAngle =  22.7527f * M6020s_Pitch.rc_targetAngle ;
		  M6020s_Yaw.targetAngle =  22.7527f * M6020s_Yaw.rc_targetAngle;
			
		}
		else 
		{
			M6020s_Pitch.targetAngle += - VPitch  *0.02f;
		  M6020s_Yaw.targetAngle += - VOmega  *0.02f;
		}	
	}
	else
	{		
		test_1 = VOmega;
		
		M6020s_Pitch.targetAngle += - VPitch *0.02f;
		M6020s_Yaw.targetAngle += - VOmega * 0.02f;
	}
	

  M6020s_Yaw.imu_angle = 22.7527f * DJI_C_IMU.total_yaw;
	M6020s_Pitch.imu_angle = 22.7527f * DJI_C_IMU.pitch;

	Pit_AngleLimit(&M6020s_Pitch.targetAngle);
	
	if(onlineyaw == 1 )
	{
		M6020s_Yaw.p_outCurrent = Vision_YawOPID.CLOUD_Position_PID(&Vision_YawOPID,M6020s_Yaw.targetAngle,M6020s_Yaw.imu_angle);
    Filter_IIRLPF(&DJI_C_IMU.Gyro_z, &Yaw_Imu, 0.2); //陀螺仪在线时底盘跟随云台
		Filter_IIRLPF(&DJI_C_IMU.Gyro_y, &Pitch_Imu, 0.2); //陀螺仪在线时底盘跟随云台
		M6020s_Yaw.i_outCurrent = Vision_YawIPID.CLOUD_Position_PID(&Vision_YawIPID,M6020s_Yaw.p_outCurrent,Yaw_Imu);
		M6020s_Pitch.p_outCurrent = Vision_PitchOPID.CLOUD_Position_PID(&Vision_PitchOPID,M6020s_Pitch.targetAngle,M6020s_Pitch.imu_angle);
		M6020s_Pitch.i_outCurrent = -Vision_PitchIPID.CLOUD_Position_PID(&Vision_PitchIPID,M6020s_Pitch.p_outCurrent,Pitch_Imu);
	}
	else
	{		
		M6020s_Yaw.p_outCurrent = M6020s_YawOPID.CLOUD_Position_PID(&M6020s_YawOPID,M6020s_Yaw.targetAngle,M6020s_Yaw.imu_angle);
		Filter_IIRLPF(&DJI_C_IMU.Gyro_z, &Yaw_Imu, 0.3); //陀螺仪在线时底盘跟随云台
		Filter_IIRLPF(&DJI_C_IMU.Gyro_y, &Pitch_Imu, 0.3); //陀螺仪在线时底盘跟随云台
		M6020s_Yaw.i_outCurrent = M6020s_YawIPID.CLOUD_Position_PID(&M6020s_YawIPID,M6020s_Yaw.p_outCurrent,Yaw_Imu);
		M6020s_Pitch.p_outCurrent = M6020s_PitchOPID.CLOUD_Position_PID(&M6020s_PitchOPID,M6020s_Pitch.targetAngle,M6020s_Pitch.imu_angle);
		M6020s_Pitch.i_outCurrent = - M6020s_PitchIPID.CLOUD_Position_PID(&M6020s_PitchIPID,M6020s_Pitch.p_outCurrent,Pitch_Imu);
	}
	


	
	Motor_0x1FF_SendData(M6020s_Yaw.i_outCurrent,M6020s_Pitch.i_outCurrent,M6020s_Pitch.i_outCurrent,0);
	last_onlineyaw = onlineyaw;
	change_online = 0;

	}
	else
	Motor_0x1FF_SendData(0,0,0,0);
}



float Last_Max = 0;
float Last_Lim = 0;
float Last_Max_Con = 1180;
 void Pit_AngleLimit(float *PitchAngle)
{
	  static float IMUMode_Limit_L;
    static float IMUMode_Limit_H;
	
		if(Robot.ChassisWorkMode == ChassisWorkMode_Square)
		{
			if(DR16_Export_Data.Robot_TargetValue.Omega_Value > 200)
			{
				Cloud_Pitch_Max = 1400;
			}
			else
			{
			  //Cloud_Pitch_Max = 1480;  头暂时不需要抬这么低了
				Cloud_Pitch_Max = 1400;
			}
		}
		else
		{
			Cloud_Pitch_Max = 1400;
		}
//	  IMUMode_Limit_L = DJI_C_IMU.pitch * 22.7527f + fabs(Cloud_getPitchAngleWithDown());
//	  IMUMode_Limit_H = DJI_C_IMU.pitch * 22.7527f - fabs(Cloud_getPitchAngleWithUp());
	  IMUMode_Limit_L = DJI_C_IMU.pitch * 22.7527f  + fabs(Cloud_getPitchAngleWithUp());
	  IMUMode_Limit_H = DJI_C_IMU.pitch * 22.7527f - fabs(Cloud_getPitchAngleWithDown());  
	
    if (M6020s_Pitch.totalAngle < Cloud_Pitch_Min && Last_Lim != *PitchAngle)
    {
			if(Last_Lim > *PitchAngle || Last_Lim == 0)
			{	}
			else
			{
      *PitchAngle = IMUMode_Limit_L;
			}
			
    }
	
    if (M6020s_Pitch.totalAngle > Cloud_Pitch_Max && (Last_Max != *PitchAngle || Last_Max_Con != Cloud_Pitch_Max))
    {
			if(Last_Max < *PitchAngle)
			{	}
			else
			{
      *PitchAngle = IMUMode_Limit_H;
			}
			
    }
		Last_Lim = *PitchAngle;
		Last_Max = *PitchAngle;
		Last_Max_Con = Cloud_Pitch_Max;
}

static float Cloud_getPitchAngleWithUp(void)
{
    if (M6020s_Pitch.totalAngle < Cloud_Pitch_Max)
    {
        return 0;
    }
    else
    {
        return (M6020s_Pitch.totalAngle - Cloud_Pitch_Max);
    }
}

static float Cloud_getPitchAngleWithDown(void)
{
    if (M6020s_Pitch.totalAngle > Cloud_Pitch_Min)
    {
        return 0;
    }
    else
    {
        return (M6020s_Pitch.totalAngle - Cloud_Pitch_Min);
    }
}
uint8_t Buff_Offled = 0;
void Laser_Control(void)
{
	if(To_mode == 2 && onlineyaw == 1)
	{
		Buff_Offled = 1;
	}
	else
	{
		Buff_Offled = 0;
	}
	
	if(Robot.Device_FrictMode == FrictWorkMode_HighSpeed && Buff_Offled == 0 )
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	}
}
