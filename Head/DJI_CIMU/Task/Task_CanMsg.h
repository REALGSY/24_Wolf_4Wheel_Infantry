/**
 * @file Task_CanMsg.h
 * @author Gsy 
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __TASK_CANMSG_H
#define __TASK_CANMSG_H

#include "Handle.h"


typedef union 
{
	struct
    {
        float voltageVal;//mV
        float Shunt_Current;//mA
    }Pack;
		uint8_t data[8];
}PowerMeter_t;

#endif
