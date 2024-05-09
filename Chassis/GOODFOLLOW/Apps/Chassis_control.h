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
#define WheelMaxSpeed 11000.0f //限制轮子最大速度
#define Pitch_Follow 170*22.7527f //pitch轴跟随角度
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define Cloud_Yaw_Center 2750
#define Cloud_Pitch_Center 4095
#define Chassis_tar 2793
#define USE_RM_Referee 1  // 启动裁判系统
void read_start_imu (void); 
extern int a_start;
float ComputeMinOffset(float target, float value) ;
extern float start_imu;
void MecanumCalculate(float X_Move,float Y_Move,float Yaw ,int16_t *Speed);
void Chassis_processing(float Vx, float Vy, float VOmega ,float VPitch);
void rc_dead(float *rc_dead);
void Angle_conversion (float a_imu );
void Robot_Init(void);
void Cloud_processing(float Vx, float Vy, float VOmega ,float VPitch);
extern int onlineyaw ;
void Chassis_Init(void);
void AngleLimit(float *angle);
void RUDTargetAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite);
void RUDTotalAngle_Calc(int i , uint8_t opposite);
extern float Ship_ready_dead[4];
extern int start_go;
void Ship_calc(float X_Move,float Y_Move,float Yaw ,float *Angle) ;
void Drv_Slow(float *rec , float target , float slow_Inc, float Accval, float DecVal);
extern float kaerman;
extern float meika;

void Set_MaxSpeed(void);
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
            19000,            \
            8000,              \
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
            19000,            \
            8000,              \
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
            19000,            \
            8000,              \
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
            19000,            \
            8000,              \
            &CLOUD_Incremental_PID, \
    }

			
		
#define RF_Ship_PID_P_PARAM  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            15.0f,           \
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
            12.3f,           \
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
            15.0f,           \
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
            10.5f,           \
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
            70.0f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            100,              \
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
            50.0f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            100,              \
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
            70.0f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            100,              \
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
            50.0f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            100,              \
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
            1.4f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            2000,            \
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
            3.00f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            5500,            \
            1000,              \
            1000,              \
            &CLOUD_Position_PID, \
    }
		
#endif

		
		
