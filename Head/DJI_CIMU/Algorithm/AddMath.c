/**
 * @file AddMath.c
 * @author Someone
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "AddMath.h"

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int floatEqual_0(float num) {//用于处理判断float变量是否为0 
if (num<(float)flaot0)
{
	return 1;
}
return 0;
}
