/**
 * @file BSP_CAN.h
 * @author Gsy 
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _BSP_CAN_H
#define _BSP_CAN_H
void CAN_1_Filter_Config(void);
void CAN_2_Filter_Config(void);
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern uint8_t CAN_RxMessage[8];
extern CAN_RxHeaderTypeDef CAN_Rx_Heade;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;
void CAN_RxMessage_Export_Date(CAN_HandleTypeDef *hcanx, osMessageQId CANx_Handle, uint8_t Can_type);
typedef struct
{
    struct
    {
        CAN_FilterTypeDef CAN_Filter;
    } CAN_FilterTypedef;

    struct
    {
        CAN_RxHeaderTypeDef CANx_RxHeader;
        uint8_t CAN_RxMessage[8];
    } CAN_RxTypedef;

} Can_Data_t;

#define Can_DataGroundInit \
    {                      \
        {0}, {0},          \
    }
		
#define Can1_Type 1
#define Can2_Type 2
		

#endif
