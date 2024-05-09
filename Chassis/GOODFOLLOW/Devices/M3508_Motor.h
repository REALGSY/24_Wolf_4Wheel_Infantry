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
    uint16_t realAngle;  //读回来的机械角度
    int16_t realSpeed;   //读回来的速度
    int16_t realCurrent; //读回来的实际电流
    uint8_t temperture;  //读回来的电机温度

    int16_t targetSpeed;  //目标速度
    uint16_t targetAngle; //目标角度
    uint16_t lastAngle;   //上次的角度
    int32_t totalAngle;   //累积总共角度
    int16_t turnCount;    //转过的圈数

    int16_t i_outCurrent; //输出电流
    int16_t p_outCurrent; //输出电流
    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
	
	  positionpid_t p_pid;		    //电机pid
	  incrementalpid_t i_pid;		    //电机pid
} M3508s_t;

extern M3508s_t M3508s[4];

#endif
