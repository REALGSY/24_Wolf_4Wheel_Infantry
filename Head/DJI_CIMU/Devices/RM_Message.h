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
	  int16_t Residual_Heat;  //枪管剩余
	  int16_t SupCap_Mix;   //超电挡位
	  int8_t FricSpeed;   //枪口规定速度
    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
	
} RM_ME_t;

typedef struct
{
	  int16_t RM_Shot_Speed;

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
	
} RM_ME_2_t;

extern RM_ME_t RM_ME[4];
extern RM_ME_2_t RM_ME_2;
#endif
