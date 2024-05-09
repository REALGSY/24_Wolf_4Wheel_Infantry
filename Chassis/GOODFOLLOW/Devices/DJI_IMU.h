/**
 * @file DJI_IMU.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-04-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _DJI_IMU_H
#define _DJI_IMU_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "typedef.h"

#define MPU_DELAY(x) HAL_Delay(x)

#define INS_DELTA_TICK 1 //任务调用的间隔

#define DMA_RX_NUM 23
#define MPU6500_RX_BUF_DATA_OFFSET 0
#define IST8310_RX_BUF_DATA_OFFSET 15

#define MPU_DATA_READY_BIT 0 //陀螺仪数据准备
#define MPU_MOT_BIT 1        //mpu6500 运动检测

#define DJI_IMUFUNGroundInit        \
    {                               \
        &DJI_IMU_Init,              \
            &mpu_get_data,          \
            &IMU_Cali_Slove,        \
            &IMU_data_read_over,    \
    }

extern	uint8_t IMU_Init_Condition ;
typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t mx;
    int16_t my;
    int16_t mz;

    int16_t temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    int16_t ax_offset;
    int16_t ay_offset;
    int16_t az_offset;

    int16_t gx_offset;
    int16_t gy_offset;
    int16_t gz_offset;
} mpu_data_t;


typedef struct
{
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
    float original_gyro[3]; //原始加速度值
} mpu6500_real_data_t;

typedef struct 
{
  uint8_t status;
  float mag[3];
} ist8310_real_data_t;

typedef struct
{
    void (*DJI_IMU_Init)(void);
    void (*mpu_get_data)(void);
    void (*IMU_Cali_Slove)(float gyro[3], float accel[3], float mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310);
    void (*IMU_data_read_over)(mpu6500_real_data_t *mpu6500_real_data, ist8310_real_data_t *ist8310_real_data);

} DJI_IMUFUN_t;

extern uint8_t first_temperate;
extern DJI_IMUFUN_t DJI_IMUFUN;

#endif /*_DJI_IMU_H*/
