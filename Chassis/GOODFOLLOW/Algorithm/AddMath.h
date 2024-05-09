/**
 * @file AddMath.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __USERMATH_H
#define __USERMATH_H
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <Math.h>
#include <string.h>  //机器人的默认配置文件。

#define HIGH 	0x1
#define LOW 	0x0
#define flaot0  1e-6  //当变量的绝对值小于此时，变量float为0 
//#define PI 3.1415926535897932384626433832795f
#define HALF_PI 1.5707963267948966192313216916398f
#define TWO_PI 6.283185307179586476925286766559f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f
#define EULER 2.718281828459045235360287471352f

#ifdef abs
#undef abs
#endif

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))
#define swap(a, b) { uint8_t t = a; a = b; b = t; }

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#define getBit(value, pos) ((value >> pos) & 1)
#define setBit(value, pos) value|(1 << pos)
#define clrBit(value, pos) value&(~(1 << pos))
#define toggleBit(value, pos) value^(1 << pos)

long map(long, long, long, long, long);
int floatEqual_0(float num);

#define  IndexOutofBounds(index,length) (index<0||index>length-1) 

#endif /* __USERMATH_H */
