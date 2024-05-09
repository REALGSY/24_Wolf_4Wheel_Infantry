/**
 * @file Cloud_control.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
 
#ifndef __CLOUD_CONTROL_H
#define __CLOUD_CONTROL_H

#include "PID.h"

void YawWorkMode_Follow(float Vx, float Vy, float VOmega , float VPitch);
static float Cloud_getPitchAngleWithUp(void);
static float Cloud_getPitchAngleWithDown(void);
void Pit_AngleLimit(float *PitchAngle);
extern int pitch_start ;
extern uint8_t init_mode ;
extern int Open_Left ;
extern int Last_Fine ;
extern int test_1;
extern int test_2;
void Laser_Control(void);
//#define Cloud_Pitch_Min  1180
//#define Cloud_Pitch_Max  200
#define M6020s_YawOPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            0.65f,           \
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
	
#define M6020s_YawIPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            350.0f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            29999,            \
            0,              \
            1000,              \
            &CLOUD_Position_PID, \
    }
	

#define M6020s_PitchOPIDInit  \
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
            15000,            \
            500,              \
            2000,              \
            &CLOUD_Position_PID, \
    }

		
#define M6020s_PitchIPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            180.0f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            300,              \
            12000,              \
            &CLOUD_Position_PID, \
    }
	

#define Vision_YawOPIDInit  \
		{                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            0.7f,           \
            0.0f,            \
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
	
#define Vision_YawIPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            250.0f,           \
            1.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            29999,            \
            500,              \
            20000,              \
            &CLOUD_Position_PID, \
    }
	

#define Vision_PitchOPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            0.8f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            15000,            \
            500,              \
            2000,              \
            &CLOUD_Position_PID, \
    }

		
#define Vision_PitchIPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            180.0f,           \
            0.0f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            300,              \
            12000,              \
            &CLOUD_Position_PID, \
    }
	
#endif
