#ifndef DEV_CIMU_H
#define DEV_CIMU_H

#include "BSP_CAN.h"
#include "main.h"
#include "typedef.h"

#pragma anon_unions

#define DJI_C_Angle 0x195
#define DJI_C_Gyro 0x165

#define Angle_turn_Radian 57.295779513082320876798154814105f

#define GETINFO_WAY 1

#define DJI_C_IMUFUNGroundInit            \
    {                                     \
        &DJI_C_Euler_getInfo,             \
            &DJI_C_Gyro_getInfo,          \
            &Updata_Hand_Euler_Gyro_Data, \
            &Check_DJI_C_IMU,             \
            &CIMU_Reset,                   \
    }

//处理过数据
typedef struct
{
    float yaw;
    float pitch;
    float last_yaw;
	  float last_pitch;
    int32_t turnCounts;
	  int32_t pitch_turnCounts;
    float total_yaw;
		float total_pitch;
    float Gyro_z;
    float Gyro_y;
    uint8_t OffLineFlag; //设备离线标志

} DJI_C_IMU_t;
extern DJI_C_IMU_t DJI_C_IMU;

//DJI_C 陀螺仪角度接收联合体
typedef union
{
    struct
    {
        float yaw;
        float pitch;
    };
    uint8_t BraodData[8];

    uint16_t InfoUpdateFrame; //帧率
} DJI_C_Euler_u;
extern DJI_C_Euler_u DJI_C_Euler_Receive;
//DJI_C 陀螺仪角速度接收联合体
typedef union
{
    struct
    {
        float Gyro_z;
        float Gyro_y;
    };
    uint8_t BraodData[8];
} DJI_C_Gyro_u;
extern DJI_C_Gyro_u DJI_C_Gyro_Receive;

typedef struct
{
    void (*DJI_C_Euler_getInfo)(Can_Export_Data_t CAN_Rx_Structure);
    void (*DJI_C_Gyro_getInfo)(Can_Export_Data_t CAN_Rx_Structure);
    void (*Updata_Hand_Euler_Gyro_Data)(void);
    void (*Check_DJI_C_IMU)(void);
    void (*CIMU_Reset)(DJI_C_IMU_t *DJI_C_IMU);
} DJI_C_IMUFUN_t;

extern uint32_t Gyro_FPS;
extern uint32_t Euler_FPS;
extern DJI_C_IMUFUN_t DJI_C_IMUFUN;

#endif
