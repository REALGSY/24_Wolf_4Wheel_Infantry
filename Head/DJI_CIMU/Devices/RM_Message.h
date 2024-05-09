/**
 * @file RM_Message.h
 * @author Gsy 
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __RM_MESSAGE_H
#define __RM_MESSAGE_H


#include <stdbool.h>
#include <stdint.h>
#include "can.h"
#include "PID.h"
#include "typedef.h"
#define RM_READID_START 0x201

void RM_Mess_getInfo(Can_Export_Data_t RxMessage);
void RM_2_Mess_getInfo(Can_Export_Data_t RxMessage);
typedef struct
{
	  int16_t RM_ID;
	  int16_t Residual_Heat;  //ǹ��ʣ��
	  int16_t SupCap_Mix;   //���絲λ
	  int8_t FricSpeed;   //ǹ�ڹ涨�ٶ�
    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
	
} RM_ME_t;

typedef struct
{
	  int16_t RM_Shot_Speed;

    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
	
} RM_ME_2_t;

extern RM_ME_t RM_ME[4];
extern RM_ME_2_t RM_ME_2;
#endif
