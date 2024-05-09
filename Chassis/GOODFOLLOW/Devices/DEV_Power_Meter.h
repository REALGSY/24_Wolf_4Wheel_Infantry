/** 
 * @file DEV_Power_Meter.h
 * @author Gsy 
 * @brief 
 * @version 1
 * @date 2023-04-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __DEV_POWER_METER_H
#define __DEV_POWER_METER_H

#include "Handle.h"
#include "typedef.h"

typedef union 
{
	struct
    {
        float voltageVal;//mV
        float Shunt_Current;//mA
    }Pack;
		uint8_t data[8];
		
}PowerMeter_Rec_t;

typedef struct 
{
		float infoUpdateFrame;
		float OffLineFlag;
		float Power;
	
}PowerMeter_t;

extern PowerMeter_t Power_Meter;
void Check_Power_Meter(void);
void PowerMeter_Update(Can_Export_Data_t RxMessage);
#endif
