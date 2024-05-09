/**
 * @file M3508_Motor.c
 * @author Gsy 
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "M3508_Motor.h"
#include <stdio.h>
#include "BSP_CAN.h"


M3508s_t M3508s[4];

void M3508_getInfo(Can_Export_Data_t RxMessage)
{
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - M3508_READID_START);
    //������ݣ����ݸ�ʽ���C620���˵����P33
    M3508s[StdId].realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    M3508s[StdId].realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    M3508s[StdId].realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    M3508s[StdId].temperture = RxMessage.CANx_Export_RxMessage[6];

    //֡��ͳ�ƣ����ݸ��±�־λ
    M3508s[StdId].InfoUpdateFrame++;
    M3508s[StdId].InfoUpdateFlag = 1;
}
