/**
 * @file Monitor_RM_CAN.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __Monitor_RM_CAN_H
#define __Monitor_RM_CAN_H

#include "can.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

void Motor_0x200_SendData (int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
void Motor_0x1FF_SendData (int16_t motor5,int16_t motor6,int16_t motor7,int16_t motor8);
void Motor_0x200_Can2_SendData (int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
void Motor_0x1FF_Can2_SendData(int16_t motor5,int16_t motor6,int16_t motor7,int16_t motor8);
void DR16_0x175_Can1_SendData(int16_t Vx,int16_t Vy,int16_t Vw,int16_t mode);
void DR16_0x185_Can1_SendData(int16_t Vx,int16_t Vy,int16_t Vw,int16_t mode);
void Motor_0x2FF_SendData(int16_t motor5,int16_t motor6,int16_t motor7,int16_t motor8);
#endif
