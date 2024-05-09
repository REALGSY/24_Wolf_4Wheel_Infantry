/**
 * @file Chassis_control.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
  
	
#ifndef __CHIASSIS_CONTROL_H
#define __CHIASSIS_CONTROL_H

#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define WheelMaxSpeed 7000.0f //限制轮子最大速度
#define Pitch_Follow 170*22.7527f //pitch轴跟随角度
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define M6020_mAngleRatio 22.7527f //机械角度与真实角度的比率
#define Cloud_Yaw_Center 963
#define Cloud_Pitch_Center 4132
extern int Chassis_tar ;
void read_start_imu (void); 
extern int a_start;
extern int ComputeMinOffset(int target, int value) ;
extern float start_imu;
void MecanumCalculate(float X_Move,float Y_Move,float Yaw ,int16_t *Speed);
void Chassis_processing(float Vx, float Vy, float VOmega ,float VPitch);
void rc_dead(float *rc_dead);
void ChassisWorkMode_Follow(float Vx, float Vy, float VOmega);
void Angle_conversion (float a_imu );
void Robot_Init(void);
void Cloud_processing(float Vx, float Vy, float VOmega ,float VPitch);
extern int onlineyaw ;
extern int16_t  Yaw_error;
void Chassis_Init(void);
void Chassis_Dead_Init(void);
float Cloud_getYawAngleWithCenter(void);
extern uint8_t Sup_Cap;
#define M3508s_FloowOPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            5.54f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            10000,            \
            0,              \
            1000,              \
            &CLOUD_Position_PID, \
    }

#define LFWHEEL_PID_PARAM  \
    {                       \
            0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            10.0f,           \
            0.8f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            15000,            \
            9000,              \
            &CLOUD_Incremental_PID, \
    }

#define RFWHEEL_PID_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            10.0f,           \
            0.8f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            15000,            \
            9000,              \
            &CLOUD_Incremental_PID, \
    }


#define LBWHEEL_PID_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            10.0f,           \
            0.8f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            15000,            \
            9000,              \
            &CLOUD_Incremental_PID, \
    }

	
#define RBWHEEL_PID_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            10.0f,           \
            0.8f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            15000,            \
            9000,              \
            &CLOUD_Incremental_PID, \
    }

			
		
#define RF_Ship_PID_P_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            1.0f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            1000,            \
            20,              \
            2000,              \
            &CLOUD_Position_PID, \
    }

#define LF_Ship_PID_P_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            1.0f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            1000,            \
            20,              \
            2000,              \
            &CLOUD_Position_PID, \
    }


#define LB_Ship_PID_P_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            1.0f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            1000,            \
            20,              \
            2000,              \
            &CLOUD_Position_PID, \
    }

	
#define RB_Ship_PID_P_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            1.0f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            1000,            \
            20,              \
            2000,              \
            &CLOUD_Position_PID, \
    }
		
#define RF_Ship_I_PID_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            30.3f,           \
            50.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            1000,              \
            1000,              \
            &CLOUD_Position_PID, \
    }

#define LF_Ship_I_PID_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            30.0f,           \
            50.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            1000,              \
            1000,              \
            &CLOUD_Position_PID, \
    }
		
#define LB_Ship_I_PID_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            30.3f,           \
            50.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            1000,              \
            1000,              \
            &CLOUD_Position_PID, \
    }
		
#define RB_Ship_I_PID_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            30.3f,           \
            50.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            1000,              \
            1000,              \
            &CLOUD_Position_PID, \
    }
		
#define Follow_I_PID_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            1.3f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            10000,            \
            1000,              \
            1000,              \
            &CLOUD_Position_PID, \
    }
		
#define Follow_P_PID_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            3.3f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            1000,              \
            1000,              \
            &CLOUD_Position_PID, \
    }
#endif
