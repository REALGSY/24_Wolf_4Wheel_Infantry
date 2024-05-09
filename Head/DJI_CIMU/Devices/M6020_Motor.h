/**
 * @file M6020_Motor.h
 * @author Gsy 
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __M6020_MOTOR_H
#define __M6020_MOTOR_H

#include "can.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "PID.h"
#include "typedef.h"
#define M6020_READID_START 0x205
#define M6020_READID_END 0x207

typedef struct
{
    uint16_t realAngle;  //�������Ļ�е�Ƕ�
    int16_t realSpeed;   //���������ٶ�
    int16_t realCurrent; //��������ʵ��ת�ص���
    uint8_t temperture;  //�������ĵ���¶�

    int16_t targetSpeed; //Ŀ���ٶ�
    float  targetAngle; //Ŀ��Ƕ�
    uint16_t lastAngle;  //�ϴεĽǶ�
    int32_t totalAngle;  //�ۻ��ܹ��Ƕ�
    int16_t turnCount;   //ת����Ȧ��
    float rc_targetAngle;
    int16_t i_outCurrent; //�������
	  int16_t p_outCurrent; //�������
    positionpid_t p_pid;		    //���pid
   	incrementalpid_t i_pid;		    //���pid
    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
	  float imu_angle;
	  int32_t imu_last_angle;
	  float Init_angle;
		float Dead_angle;
    float Ship_error;
} M6020s_t;

extern M6020s_t M6020s_Yaw;
extern M6020s_t M6020s_Pitch;
extern M6020s_t M6020s_Yaw_Follow;
extern M6020s_t M6020s[4];
void M6020_getInfo(Can_Export_Data_t RxMessage);
void Check_M6020(void);
#endif
