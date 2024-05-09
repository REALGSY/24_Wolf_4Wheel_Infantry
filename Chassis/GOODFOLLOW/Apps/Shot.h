
#ifndef __SHOT_H
#define __SHOT_H

#include "can.h"
#include "PID.h"
void shot_IRQHandler(void);
void roll_shot_go (void);
void if_or_no (void);
void Roll_Init (void);
void Roll_Preload (void);
void Roll_Mode (void);
void Shoot_MotorInObstruct (void);
void State_detection (void);
void Off_Shot(void);
void Reload_Reset(void);
void Speed_Reme(void);
#define one_part_roll (8191 * 36 / 9); //转动此角度值发送一个弹丸

typedef struct
{
	int roll_get_time;
	int last_roll_frequency; //上次的射击数目
	int16_t roll_frequency;   
	int16_t shot_one; //需要供弹一次
	int16_t maybe_stop; //判断是否卡弹
	int time_up; //发射判断时间记录位
	int Locked_time;  //记录堵转时间
	int Disability;
}Shot_t;

#define Fric_L_IPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            20.0f,           \
            0.6f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            10000,            \
            3000,              \
            &CLOUD_Incremental_PID, \
    }
	
#define Fric_R_IPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            20.0f,           \
            0.6f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            10000,            \
            3000,              \
            &CLOUD_Incremental_PID, \
    }
	
#define Fric_MotorOPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            0.34f,           \
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
		
#define Fric_MotorIPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            3.4f,           \
            0.11f,           \
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
