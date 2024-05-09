
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
void Reload_Reset(void);
void Speed_Reme(void);
void FricSpeed_Change(void);
extern int16_t FricSpeed;
extern int16_t hold_time;
extern uint16_t Auto_Shot;
#define one_part_roll (8191 * 36 / 9); //ת���˽Ƕ�ֵ����һ������

typedef struct
{
	int roll_get_time;
	int last_roll_frequency; //�ϴε������Ŀ
	int16_t roll_frequency;   
	int16_t shot_one; //��Ҫ����һ��
	int16_t maybe_stop; //�ж��Ƿ񿨵�
	int time_up; //�����ж�ʱ���¼λ
	int Locked_time;  //��¼��תʱ��
	int Disability;
	uint16_t Key_need;
}Shot_t;
extern Shot_t shot;
extern float Shot_RealSpeed ;
extern uint16_t FIRE_FIRE ;
extern uint8_t Shoot_Down_Flag;
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
            15000,            \
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
            15000,            \
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
            0.4f,           \
            0.0f,           \
            1.1f,          \
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
            2.87f,           \
            0.008f,           \
            0.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            9500,            \
            0,              \
            1000,              \
            &CLOUD_Position_PID, \
    }
#endif
