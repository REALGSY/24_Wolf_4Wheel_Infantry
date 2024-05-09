/**
 * @file Control_Vision.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __Control_Vision_H
#define __Control_Vision_H

#include "CRC.h"
//#include "Cloud_control.h"
#include "PID.h"
//#include "RM_JudgeSystem.h"
//#include "Robot_Config.h"
#include "Robot_control.h"
#include "dma.h"
#include "gpio.h"
#include "typedef.h"
#include "usart.h"
#include <Math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h> //机器人的默认配置文件。

typedef struct
{
    float x;
    float y;
} XY_t;
#define Vision_Version 1
#define Vision_Oldversion 0 //旧视觉版本
#define Vision_Newversion 1 //新视觉版本
#define VISION_TX_SIZE    15

#if Vision_Version == Vision_Oldversion
//视觉的识别坐标系大小。
#define VisionPage_Width 1280
#define VisionPage_Height 800

typedef struct
{
    struct
    {
        char Start_Tag;
        int8_t mode; //数据是否可用(是否识别到装甲板)
        int8_t mode_select;
        int8_t whether_Fire; //是否可以射击(控制自动射击)
        int8_t _yaw;         //正负 0正1负
        int16_t x;           //yaw轴，为屏幕坐标点
        int8_t _pitch;       //正负 0正1负
        int16_t y;           //pitch 轴
        int16_t depth;       //深度
        int16_t crc;
        char End_Tag;
    } RawData; // 视觉的协议 接收一帧的数据结构体

    XY_t Target_Offset;  //根据收回的坐标数据，计算当前距离目标偏差值
    XY_t Gravity_Offset; //不是枪口对准目标就一定能打中（重力影响），用来控制枪管需要微调多少才能刚好击中。==》重力偏差值
    XY_t Final_Offset;   //最终合成给PID计算的偏差值。

    XY_t Target_LastOffset; //保存 视觉发回的上一次的目标偏差值
    XY_t ErrorChange_Rate;  //计算视觉传回来的误差变化率(量)

    XY_t SpeedGain;    //使用运动速度来增益。
    XY_t LpfAttFactor; //滤波系数

    uint8_t InfoUpdateFlag;      //信息读取更新标志
    uint16_t InfoUpdateFrame;    //帧率
    uint8_t OffLineFlag;         //设备离线标志
    uint32_t FPS;                //视觉的数据帧率
#define Vision_BuffSIZE (20 + 2) //视觉数据缓冲区长度
} VisionData_t;

#elif Vision_Version == Vision_Newversion

typedef struct
{
    struct
    {
			#pragma pack(1) 
        union
        {
            struct
            {
                uint8_t Start_Tag;
                uint8_t mode; //数据是否可用(是否识别到装甲板)
                uint8_t whether_Fire; //是否可以射击(控制自动射击)
								float yaw_angle;
								float pitch_angle;
                uint16_t depth;
								uint8_t crc;
								uint8_t End_Tag;
            };
            uint8_t VisionRawData[14];
        };
			#pragma pack() 
        float x;
        float y;
       // float depth;
        float yaw_;
    } RawData; // 视觉的协议 接收一帧的数据结构体

    XY_t Gravity_Offset; //不是枪口对准目标就一定能打中（重力影响），用来控制枪管需要微调多少才能刚好击中。==》重力偏差值
    XY_t Final_Offset;   //最终合成给PID计算的偏差值。
		XY_t Error_Reme;    ////记录合规的数值。
    XY_t ErrorChange_Rate; //计算视觉传回来的误差变化率(量)

    XY_t SpeedGain;    //使用运动速度来增益。
    XY_t LpfAttFactor; //滤波系数

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
    uint32_t FPS;             //视觉的数据帧率
} VisionData_t;

#define Vision_BuffSIZE (13 + 2) //视觉数据缓冲区长度
#endif

#define VisionExportDataGroundInit \
    {                              \
        {0},                       \
            {0},                   \
            {0},                   \
            0,                     \
            0,                     \
            &VisionSend_IMU,       \
    }

#define Control_Vision_FUNGroundInit   \
    {                                  \
        &Vision_Init,                  \
            &Vision_processing,        \
            &Vision_Handler,           \
            &Vision_USART_Receive_DMA, \
            &GetVisionDiscMode,        \
            &Check_Vision,             \
    }

typedef struct
{
    union
    {
        struct
        {
            float YawAngle_Error;   //陀螺仪YAW角度差
            float PitchAngle_Error; //陀螺仪Pitch角度差
        };
        uint8_t Angle_Error_Data[8];
    } VisionSend_t;

    int Gyro_z;           //陀螺仪加速度小数点后两位
    uint8_t Gyro_z_Hight; //陀螺仪加速度小数点后两位高八位
    uint8_t Gyro_z_low;   //陀螺仪加速度小数点后两位低八位

} VisionSend_IMU_t;

typedef struct
{
    XY_t FinalOffset;        //最终赋给云台pid 计算的偏差值。
    XY_t FinalOffset_Last;   //最终赋给云台pid 计算的偏差值。
    XY_t FinalSpeed;         //最终赋给云台pid 计算的偏差值算出的速度。
    bool AbleToFire;         //表示目标已经瞄准，可以直接射击
    float FinalOffset_depth; //最终视觉深度
    VisionSend_IMU_t *VisionSend_IMU;

} VisionExportData_t;

#pragma pack(1)
typedef struct
{
	uint8_t start_tag;		/*<! 帧头'S' */
	// uint16_t crc;			/*<! crc检验位 */
	uint8_t robot_id;		/*<! 机器人ID */
	uint8_t mode;			/*<! 视觉模式 */
	float yaw_angle;		/*<! IMU Yaw */
	float pit_angle;		/*<! IMU Pit */
	uint8_t bullet_velocity;/*<! 射速 */
	uint8_t end_tag;		/*<! 帧尾'E' */

}VisionSend_Pack_t;
#pragma pack()

typedef union 
{
	uint8_t data[VISION_TX_SIZE];
	VisionSend_Pack_t Pack;
}VisionSendMsg_u;


typedef struct
{
    void (*Vision_Init)(void);
    void (*Vision_processing)(void);
    void (*Vision_Handler)(UART_HandleTypeDef *huart);
    void (*Vision_USART_Receive_DMA)(UART_HandleTypeDef *huartx);
    uint8_t (*GetVisionDiscMode)(void);
    void (*Check_Vision)(void);
} Control_Vision_FUN_t;
extern VisionSendMsg_u Send_Msg; /*<! 打包数据至小电脑 */
extern uint8_t Vision_DataBuff[Vision_BuffSIZE];
extern VisionExportData_t VisionExportData;
extern VisionData_t VisionData;
extern Control_Vision_FUN_t Control_Vision_FUN;
extern VisionSend_IMU_t VisionSend_IMU;
extern float last_cloud;
extern float offcloud;
//extern WorldTime_RxTypedef VisionKF_TIME;
extern UART_HandleTypeDef *vision_uart; // 外部句柄
void Vision_classdef_SendToPC(VisionSendMsg_u *pack2vision);

#ifdef Enable_Vision_Test
extern uint8_t Vision_SendBuf[5][5];
void Vision_I_T_Set(uint8_t ID, uint8_t Type);
#endif

#endif
