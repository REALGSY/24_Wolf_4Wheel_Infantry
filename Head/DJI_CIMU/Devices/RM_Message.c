/**
 * @file RM_Message.c
 * @author Gsy 
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "RM_Message.h"
#include <stdio.h>

RM_ME_t RM_ME[4];
RM_ME_2_t RM_ME_2;

void RM_Mess_getInfo(Can_Export_Data_t RxMessage)
{
    uint32_t StdId;
    StdId = 0;
    //解包数据，数据格式详见C620电调说明书P33
    RM_ME[StdId].RM_ID = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
	  RM_ME[StdId].Residual_Heat = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
	  //RM_ME[StdId].Residual_Heat =100;
    RM_ME[StdId].SupCap_Mix = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
	  RM_ME[StdId].FricSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[6] << 8 | RxMessage.CANx_Export_RxMessage[7]);
    //帧率统计，数据更新标志位
    RM_ME[StdId].InfoUpdateFrame++;
    RM_ME[StdId].InfoUpdateFlag = 1;
}

void RM_2_Mess_getInfo(Can_Export_Data_t RxMessage)
{

    //解包数据，数据格式详见C620电调说明书P33
    RM_ME_2.RM_Shot_Speed = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);

    //帧率统计，数据更新标志位
    RM_ME_2.InfoUpdateFrame++;
    RM_ME_2.InfoUpdateFlag = 1;
}
