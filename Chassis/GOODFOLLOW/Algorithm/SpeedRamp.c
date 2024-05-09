/**
 * @file SpeedRamp.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "SpeedRamp.h"

/**
	* @brief  斜坡函数计算
  * @param	斜坡函数结构体
  * @retval None
  */
int16_t SpeedRampCalc(SpeedRamp_t *SpeedRamp)
{

	SpeedRamp->count += SpeedRamp->rate;
	VAL_LIMIT(SpeedRamp->count,
			  SpeedRamp->mincount,
			  SpeedRamp->maxcount);

	return SpeedRamp->count;
}

/**
	* @brief  斜坡计数值归零
  * @param	斜坡函数结构体
  * @retval None
  */
void CountReset(SpeedRamp_t *SpeedRamp)
{

	if (abs(SpeedRamp->count) < abs(SpeedRamp->rate))
	{
		SpeedRamp->count = 0;
	}
	else
	{

		SpeedRamp->count -= SpeedRamp->count * 0.2f;
	}

	//if (SpeedRamp->count > abs(SpeedRamp->rate))
	//{
	//	SpeedRamp->count -= abs(SpeedRamp->rate);
	//}
	//else if (SpeedRamp->count < -abs(SpeedRamp->rate))
	//{
	//	SpeedRamp->count += abs(SpeedRamp->rate);
	//}
	//else
	//{
	//
	//}
}

/**
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)
  * @retval 当前输出
  * @attention  
  */
float RAMP_float(float final, float now, float ramp)
{
	float buffer = 0;

	buffer = final - now;

	if (buffer > 0)
	{
		if (buffer > ramp)
		{
			now += ramp;
		}
		else
		{
			now += buffer;
		}
	}
	else
	{
		if (buffer < -ramp)
		{
			now += -ramp;
		}
		else
		{
			now += buffer;
		}
	}

	return now;
}
