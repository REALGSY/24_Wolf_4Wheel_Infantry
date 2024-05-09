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
#define M2006_SENDID 0x1FF    //控制5-8的电机ID
#define M2006_MaxOutput 10000 //发送给电机的最大控制值
#define M2006_LOADANGLE 36864
#define M2006_ReductionRatio 36 //电机减速比
//#define M2006_LOADANGLE		42125			/* 电机拨一个弹需要转的角度数  6*8191 （7孔拨弹）*/

//#define M2006_LOADCIRCLE	5			/* 电机拨一个弹需要转的圈数 */
//#define M2006_LOADSPEED		1800		/* 电机拨弹时的转速 */
#define M2006_FIRSTANGLE 3800 /* 电机初始位置 */

#define M2006_FunGroundInit \
    {                       \
        &M2006_getInfo,     \
            &Check_M2006,   \
    }

typedef struct
{
    uint16_t realAngle; //读回来的机械角度
    int16_t realSpeed;  //读回来的速度
    int16_t realTorque; //读回来的实际转矩

    int16_t targetSpeed; //目标速度
    int32_t targetAngle; //目标角度

    uint16_t lastAngle; //上次的角度
    int32_t totalAngle; //累积总共角度
    int16_t turnCount;  //转过的圈数

	  positionpid_t p_pid;		    //电机pid
   	incrementalpid_t i_pid;		    //电机pid
    int16_t outCurrent;      //输出电流
    int16_t inneroutCurrent; //输出电流

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} M2006s_t;

typedef struct
{
    void (*M2006_getInfo)(Can_Export_Data_t RxMessage);
    void (*Check_M2006)(void);
} M2006_FUN_t;
void M2006_getInfo(Can_Export_Data_t RxMessage);
extern M2006s_t M2006_Reload_1; //拨盘运输电机
extern M2006s_t M2006_Reload_2; 
extern M2006s_t M2006_Reload_3; 
extern M2006_FUN_t M2006_FUN;

#endif /* __M2006_MOTOR_H */
