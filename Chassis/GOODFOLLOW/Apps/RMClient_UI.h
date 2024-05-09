/**
 * @file RMClient_UI.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 裁判系统客户端自定义UI
 * @version 0.1
 * @date 2021-04-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __RMClient_UI_H
#define __RMClient_UI_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "RM_JudgeSystem.h"
#include "Robot_control.h"

#define DISPLAY_INIT        \
    {                       \
        &Show_CrossHair,    \
        &Car_DistanceFrame, \
        &ShootLv_show,      \
        &CapEnergy_Show,    \
        &Yellow_ShowOn,     \
        &Green_ShowOn,      \
        &Orange_ShowOn,     \
        &Purple_ShowOn,     \
        &Pink_ShowOn};

#define DISPLAY_UPDATA         \
    {                          \
        &ShootLv_Updata,       \
        &Cap_EnergyUpdata,     \
        &LEDYellow_MeanUpdate, \
        &LEDOrange_MeanUpdate, \
        &LEDPink_MeanUpdate,   \
    };




typedef struct
{
    void (*(*Display_Init)[9])(void);
    void (*(*Dispaly_Updata)[7])(void);
} RMClient_UI_t;

extern uint8_t UISend_Time;
extern int8_t InitShow_Flag;
extern void(*Dispaly_Init[])(void);


void UserDefined_UI(void);

#endif /* __RMClient_UI_H */
