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
#include "IMU_Compensate.h"
#include "M6020_Motor.h"
#include "M3508_Motor.h"
#include "Monitor_RM_CAN.h"
#include "DEV_CIMU.h"
#include "DJI_IMU.h"
#include "Control_Vision.h"
#include "DR16_Remote.h"
#include "SpeedRamp.h"
#include "math.h"
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
const float Cloud_Pitch_Min = 1000;
const float Cloud_Pitch_Max = 2700;
/*******************************************************************************/

/************云台--陀螺仪PID***********/
cloud_positionpid_t    M6020s_YawOPID    = M6020s_YawOPIDInit;
cloud_incrementalpid_t M6020s_YawIPID    = M6020s_YawIPIDInit;
cloud_positionpid_t    M6020s_PitchOPID  = M6020s_PitchOPIDInit;
cloud_incrementalpid_t M6020s_PitchIPID  = M6020s_PitchIPIDInit;

/**********云台--陀螺仪PID END*******/

/**
 * @brief YAW轴电机跟随云台
 * 
 * @param target 
 * @param value 
 * @return int 
 */

void YawWorkMode_Follow(float Vx, float Vy, float VOmega , float VPitch)
{
	
	if(Robot.ChassisWorkMode == ChassisWorkMode_CloudFollow || Robot.ChassisWorkMode == ChassisWorkMode_CloudOnly || Robot.ChassisWorkMode == ChassisWorkMode_Lock)
	{
	if(rc.roll == -660)
	{
		gogodeng ++;
		if(gogodeng > 100)
		{
			
			onlineyaw = !onlineyaw ;
			if(onlineyaw == 1)
			{
					__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1030);
			}
			else
			{
				__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 630);
			}
			gogodeng = 0;
		}
	}

	if(onlineyaw !=last_onlineyaw)
	{
		change_online = 1;
	}
			if (change_online == 1)
		{
			pitch_start = 22.7527f * DJI_C_IMU.pitch;
			start_imu =  22.7527f * DJI_C_IMU.yaw;
			M6020s_Yaw.rc_targetAngle = 0;
		  M6020s_Pitch.rc_targetAngle = 0;
		}
		
	if(onlineyaw == 1)	
	{
		if(VisionData.RawData.mode != 0 && VisionData.OffLineFlag == 1)
		{
		M6020s_Yaw.rc_targetAngle = VisionData.RawData.x ;
		M6020s_Pitch.rc_targetAngle = VisionData.RawData.y ;

		M6020s_Pitch.targetAngle =  22.7527f * DJI_C_IMU.pitch + M6020s_Pitch.rc_targetAngle *0.128f;
		M6020s_Yaw.targetAngle =  22.7527f * DJI_C_IMU.yaw  + M6020s_Yaw.rc_targetAngle;
		Last_Pitch_error = M6020s_Pitch.targetAngle ;
		Last_Yaw_error = M6020s_Yaw.targetAngle;
		}
		else if(VisionData.RawData.mode == 0 && VisionData.OffLineFlag == 1)
		{
			M6020s_Pitch.targetAngle = Last_Pitch_error;
			M6020s_Yaw.targetAngle = Last_Yaw_error;
		}
		else 
		{
			M6020s_Pitch.targetAngle = pitch_start ;
			M6020s_Yaw.targetAngle =  start_imu ;
		}
	}
	else
	{		
		if (change_online == 1)
		{
			M6020s_Pitch.targetAngle = 22.7527f * DJI_C_IMU.pitch;
		  M6020s_Yaw.targetAngle =  22.7527f * DJI_C_IMU.total_yaw;
		}
		
		
		M6020s_Pitch.targetAngle += - VPitch *0.02f;
		M6020s_Yaw.targetAngle += - VOmega * 0.05f;
	}
	
	Cloud_Yaw_error = ComputeMinOffset(M6020s_Yaw.targetAngle,22.7527f * DJI_C_IMU.yaw );

   M6020s_Yaw.imu_angle = 22.7527f * DJI_C_IMU.total_yaw;
	M6020s_Pitch.imu_angle = 22.7527f * DJI_C_IMU.total_pitch;
	Pit_AngleLimit(&M6020s_Pitch.targetAngle);
	
	if(onlineyaw == 1 )
	M6020s_Yaw.p_outCurrent = M6020s_YawOPID.CLOUD_Position_PID(&M6020s_YawOPID,Cloud_Yaw_error,0);
	else 
	M6020s_Yaw.p_outCurrent = M6020s_YawOPID.CLOUD_Position_PID(&M6020s_YawOPID,M6020s_Yaw.targetAngle,M6020s_Yaw.imu_angle);
	M6020s_Yaw.i_outCurrent = M6020s_YawIPID.CLOUD_Incremental_PID(&M6020s_YawIPID,M6020s_Yaw.p_outCurrent,DJI_C_IMU.Gyro_z);
  M6020s_Pitch.p_outCurrent = M6020s_PitchOPID.CLOUD_Position_PID(&M6020s_PitchOPID,M6020s_Pitch.targetAngle ,M6020s_Pitch.imu_angle);
	M6020s_Pitch.i_outCurrent = M6020s_PitchIPID.CLOUD_Incremental_PID(&M6020s_PitchIPID,M6020s_Pitch.p_outCurrent,DJI_C_IMU.Gyro_y);

	
//	Motor_0x1FF_SendData(M6020s_Yaw.i_outCurrent,M6020s_Pitch.i_outCurrent,0,0);
	
	last_onlineyaw = onlineyaw;
	change_online = 0;

	Last_Pitch_error = 22.7527f * DJI_C_IMU.pitch;
	Last_Yaw_error = 22.7527f * DJI_C_IMU.yaw;
	}
//	else
	//Motor_0x1FF_SendData(0,0,0,0);
}



void Pit_AngleLimit(int *PitchAngle)
{
	  static float IMUMode_Limit_L;
    static float IMUMode_Limit_H;
	
	  IMUMode_Limit_H = DJI_C_IMU.pitch * 22.7527f + fabs(Cloud_getPitchAngleWithUp());
    IMUMode_Limit_L = DJI_C_IMU.pitch * 22.7527f - fabs(Cloud_getPitchAngleWithDown());
	
	
    if (*PitchAngle < IMUMode_Limit_L)
    {
      *PitchAngle = IMUMode_Limit_L;
    }
    else if (*PitchAngle > IMUMode_Limit_H)
    {
      *PitchAngle = IMUMode_Limit_H;
    }
}


static float Cloud_getPitchAngleWithUp(void)
{
    if (M6020s_Pitch.totalAngle > Cloud_Pitch_Max)
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
    if (M6020s_Pitch.totalAngle < Cloud_Pitch_Min)
    {
        return 0;
    }
    else
    {
        return (M6020s_Pitch.totalAngle - Cloud_Pitch_Min);
    }
}
