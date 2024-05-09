/**
 * @file SpeedRamp.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __SpeedRamp_H
#define __SpeedRamp_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <AddMath.h>


typedef struct
{
	float count;//当前的总速度值
	float rate;//每一次叠加的速度值
	int16_t mincount;
	int16_t maxcount;
}SpeedRamp_t;
void CountReset(SpeedRamp_t *SpeedRamp);
int16_t SpeedRampCalc(SpeedRamp_t *SpeedRamp);
float RAMP_float(float final, float now, float ramp);

#endif 




