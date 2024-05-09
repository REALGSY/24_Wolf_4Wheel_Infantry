/**
 * @file M3508_Motor.h
 * @author Gsy 
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H


#include <stdbool.h>
#include <stdint.h>
#include "can.h"
#include "PID.h"
#include "typedef.h"
#define M3508_READID_START 0x201
#define M3508_READID_END 0x204

void M3508_getInfo(Can_Export_Data_t RxMessage);
 
typedef struct
{
	  uint16_t yaw_realAngle;
    uint16_t realAngle;  //�������Ļ�е�Ƕ�
    int16_t realSpeed;   //���������ٶ�
    int16_t realCurrent; //��������ʵ�ʵ���
    uint8_t temperture;  //�������ĵ���¶�

    int16_t targetSpeed;  //Ŀ���ٶ�
    uint16_t targetAngle; //Ŀ��Ƕ�
    uint16_t lastAngle;   //�ϴεĽǶ�
    int32_t totalAngle;   //�ۻ��ܹ��Ƕ�
    int16_t turnCount;    //ת����Ȧ��

    int16_t i_outCurrent; //�������
    int16_t p_outCurrent; //�������
    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
	
	  positionpid_t p_pid;		    //���pid
	  incrementalpid_t i_pid;		    //���pid
} M3508s_t;

extern M3508s_t M3508s[4];

#endif
