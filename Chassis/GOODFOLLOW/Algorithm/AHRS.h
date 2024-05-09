#ifndef AHRS_H
#define AHRS_H

#include "AHRS_MiddleWare.h"

/**
  * @brief          根据加速度的数据，磁力计的数据进行四元数初始化
  * @author         luopin
  * @param[in]      需要初始化的四元数数组
  * @param[in]      用于初始化的加速度计,(x,y,z)不为空 单位 m/s2 
  * @param[in]      用于初始化的磁力计计,(x,y,z)不为空 单位 uT
  * @retval         返回空
  */
extern void AHRS_init(float quat[4], const float accel[3], const float mag[3]);

/**
  * @brief          根据陀螺仪的数据，加速度的数据，磁力计的数据进行四元数更新
  * @author         luopin  
  * @param[in]      需要更新的四元数数组
  * @param[in]      更新定时时间，固定定时调用，例如1000Hz，传入的数据为0.001f,
  * @param[in]      用于更新的陀螺仪数据,数组顺序(x,y,z) 单位 rad
  * @param[in]      用于初始化的加速度数据,数组顺序(x,y,z) 单位 m/s2 
  * @param[in]      用于初始化的磁力计数据,数组顺序(x,y,z) 单位 uT
  * @retval         返回空
  */
extern char AHRS_update(float quat[4], const float timing_time, const float gyro[3], const float accel[3], const float mag[3]);

/**
  * @brief          根据四元数大小计算对应的欧拉角偏航yaw
  * @author         luopin
  * @param[in]      四元数数组，不为NULL
  * @retval         返回的偏航角yaw 单位 rad
  */
extern float get_yaw(const float quat[4]);

/**
  * @brief          根据四元数大小计算对应的欧拉角俯仰角 pitch
  * @author         luopin
  * @param[in]      四元数数组，不为NULL
  * @retval         返回的俯仰角 pitch 单位 rad
  */
extern float get_pitch(const float quat[4]);
/**
  * @brief          根据四元数大小计算对应的欧拉角横滚角 roll
  * @author         luopin
  * @param[in]      四元数数组，不为NULL
  * @retval         返回的横滚角 roll 单位 rad
  */
extern float get_roll(const float quat[4]);

/**
  * @brief          根据四元数大小计算对应的欧拉角yaw，pitch，roll
  * @author         luopin
  * @param[in]      四元数数组，不为NULL
  * @param[in]      返回的偏航角yaw 单位 rad
  * @param[in]      返回的俯仰角pitch  单位 rad
  * @param[in]      返回的横滚角roll 单位 rad
  */
extern void get_angle(const float quat[4], float *yaw, float *pitch, float *roll);
/**
  * @brief          返回当前的重力加速度
  * @author         luopin
  * @param[in]      空
  * @retval         返回重力加速度 单位 m/s2
  */

extern float get_carrier_gravity(void);

#endif
