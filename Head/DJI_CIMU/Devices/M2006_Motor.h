/**
 * @file M2006_Motor.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __M2006_MOTOR_H
#define __M2006_MOTOR_H

#include "can.h"
#include "typedef.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "PID.h"
#define M2006_READID_START 0x201
#define M2006_READID_END 0x202
#define M2006_SENDID 0x1FF    //����5-8�ĵ��ID
#define M2006_MaxOutput 10000 //���͸������������ֵ
#define M2006_LOADANGLE 36864
#define M2006_ReductionRatio 36 //������ٱ�
//#define M2006_LOADANGLE		42125			/* �����һ������Ҫת�ĽǶ���  6*8191 ��7�ײ�����*/

//#define M2006_LOADCIRCLE	5			/* �����һ������Ҫת��Ȧ�� */
//#define M2006_LOADSPEED		1800		/* �������ʱ��ת�� */
#define M2006_FIRSTANGLE 3800 /* �����ʼλ�� */

#define M2006_FunGroundInit \
    {                       \
        &M2006_getInfo,     \
            &Check_M2006,   \
    }

typedef struct
{
    uint16_t realAngle; //�������Ļ�е�Ƕ�
    int16_t realSpeed;  //���������ٶ�
    int16_t realTorque; //��������ʵ��ת��

    int16_t targetSpeed; //Ŀ���ٶ�
    int32_t targetAngle; //Ŀ��Ƕ�

    uint16_t lastAngle; //�ϴεĽǶ�
    int32_t totalAngle; //�ۻ��ܹ��Ƕ�
    int16_t turnCount;  //ת����Ȧ��

	  positionpid_t p_pid;		    //���pid
   	incrementalpid_t i_pid;		    //���pid
    int16_t outCurrent;      //�������
    int16_t inneroutCurrent; //�������

    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
} M2006s_t;

typedef struct
{
    void (*M2006_getInfo)(Can_Export_Data_t RxMessage);
    void (*Check_M2006)(void);
} M2006_FUN_t;
void M2006_getInfo(Can_Export_Data_t RxMessage);
extern M2006s_t M2006_Reload_1; //����������
extern M2006s_t M2006_Reload_2; 
extern M2006s_t M2006_Reload_3; 
extern M2006_FUN_t M2006_FUN;

#endif /* __M2006_MOTOR_H */
