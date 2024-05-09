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
void Pit_AngleLimit(int *PitchAngle);
extern int pitch_start ;

#define M6020s_YawOPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            0.3f,           \
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
            250.0f,           \
            5.77f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            1000,              \
            &CLOUD_Incremental_PID, \
    }
	

#define M6020s_PitchOPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            1.34f,           \
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

		
#define M6020s_PitchIPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            150.0f,           \
            4.57f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            30000,            \
            1000,              \
            &CLOUD_Incremental_PID, \
    }
	

#endif
