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
    uint16_t realAngle;  //读回来的机械角度
    int16_t realSpeed;   //读回来的速度
    int16_t realCurrent; //读回来的实际转矩电流
    uint8_t temperture;  //读回来的电机温度

    int16_t targetSpeed; //目标速度
    float  targetAngle; //目标角度
    uint16_t lastAngle;  //上次的角度
    int32_t totalAngle;  //累积总共角度
    int16_t turnCount;   //转过的圈数
    float rc_targetAngle;
    int16_t i_outCurrent; //输出电流
	  int16_t p_outCurrent; //输出电流
    positionpid_t p_pid;		    //电机pid
   	incrementalpid_t i_pid;		    //电机pid
    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
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
