#ifndef AHRS_MIDDLEWARE_H
#define AHRS_MIDDLEWARE_H
/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       AHRS_MiddleWare.c/h
  * @brief      姿态解算中间层，为姿态解算提供相关函数
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

//定义 NULL
#ifndef NULL
#define NULL 0
#endif

//定义PI 值
#ifndef PI
#define PI 3.14159265358979f
#endif

//定义 角度(度)转换到 弧度的比例
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

//定义 弧度 转换到 角度的比例
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

#define Latitude_At_ShenZhen 22.57025f
#define Latitude_At_GuangZhou 23.16f



extern void AHRS_get_height(float *high);
extern void AHRS_get_latitude(float *latitude);
extern float AHRS_invSqrt(float num);
extern float AHRS_sinf(float angle);
extern float AHRS_cosf(float angle);
extern float AHRS_tanf(float angle);
extern float AHRS_asinf(float sin);
extern float AHRS_acosf(float cos);
extern float AHRS_atan2f(float y, float x);
#endif
