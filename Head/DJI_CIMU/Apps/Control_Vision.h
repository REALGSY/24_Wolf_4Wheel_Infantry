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
#include <string.h> //�����˵�Ĭ�������ļ���

typedef struct
{
    float x;
    float y;
} XY_t;
#define Vision_Version 1
#define Vision_Oldversion 0 //���Ӿ��汾
#define Vision_Newversion 1 //���Ӿ��汾
#define VISION_TX_SIZE    16

#if Vision_Version == Vision_Oldversion
//�Ӿ���ʶ������ϵ��С��
#define VisionPage_Width 1280
#define VisionPage_Height 800

typedef struct
{
    struct
    {
        char Start_Tag;
        int8_t mode; //�����Ƿ����(�Ƿ�ʶ��װ�װ�)
        int8_t mode_select;
        int8_t whether_Fire; //�Ƿ�������(�����Զ����)
        int8_t _yaw;         //���� 0��1��
        int16_t x;           //yaw�ᣬΪ��Ļ�����
        int8_t _pitch;       //���� 0��1��
        int16_t y;           //pitch ��
        int16_t depth;       //���
        int16_t crc;
        char End_Tag;
    } RawData; // �Ӿ���Э�� ����һ֡�����ݽṹ��

    XY_t Target_Offset;  //�����ջص��������ݣ����㵱ǰ����Ŀ��ƫ��ֵ
    XY_t Gravity_Offset; //����ǹ�ڶ�׼Ŀ���һ���ܴ��У�����Ӱ�죩����������ǹ����Ҫ΢�����ٲ��ܸպû��С�==������ƫ��ֵ
    XY_t Final_Offset;   //���պϳɸ�PID�����ƫ��ֵ��

    XY_t Target_LastOffset; //���� �Ӿ����ص���һ�ε�Ŀ��ƫ��ֵ
    XY_t ErrorChange_Rate;  //�����Ӿ������������仯��(��)

    XY_t SpeedGain;    //ʹ���˶��ٶ������档
    XY_t LpfAttFactor; //�˲�ϵ��

    uint8_t InfoUpdateFlag;      //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame;    //֡��
    uint8_t OffLineFlag;         //�豸���߱�־
    uint32_t FPS;                //�Ӿ�������֡��
#define Vision_BuffSIZE (20 + 2) //�Ӿ����ݻ���������
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
								uint8_t header;		
								uint8_t enemy;		/*<! ������ID */
								bool auto_aim;
								//float roll_angle;		/*<! IMU Pit */
								float pit_angle;
								float yaw_angle;	
								uint8_t robot_id;
								uint16_t checksum;	   /*<! CRCУ���� */
            };
            uint8_t VisionRawData[14];
        };
			#pragma pack() 
        float x;
        float y;
       // float depth;
        float yaw_;
    } RawData; // �Ӿ���Э�� ����һ֡�����ݽṹ��

    XY_t Gravity_Offset; //����ǹ�ڶ�׼Ŀ���һ���ܴ��У�����Ӱ�죩����������ǹ����Ҫ΢�����ٲ��ܸպû��С�==������ƫ��ֵ
    XY_t Final_Offset;   //���պϳɸ�PID�����ƫ��ֵ��
		XY_t Error_Reme;    ////��¼�Ϲ����ֵ��
    XY_t ErrorChange_Rate; //�����Ӿ������������仯��(��)

    XY_t SpeedGain;    //ʹ���˶��ٶ������档
    XY_t LpfAttFactor; //�˲�ϵ��

    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
    uint32_t FPS;             //�Ӿ�������֡��
} VisionData_t;

#define Vision_BuffSIZE (2 + 12) //�Ӿ����ݻ���������
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
            float YawAngle_Error;   //������YAW�ǶȲ�
            float PitchAngle_Error; //������Pitch�ǶȲ�
        };
        uint8_t Angle_Error_Data[8];
    } VisionSend_t;

    int Gyro_z;           //�����Ǽ��ٶ�С�������λ
    uint8_t Gyro_z_Hight; //�����Ǽ��ٶ�С�������λ�߰�λ
    uint8_t Gyro_z_low;   //�����Ǽ��ٶ�С�������λ�Ͱ�λ

} VisionSend_IMU_t;

typedef struct
{
    XY_t FinalOffset;        //���ո�����̨pid �����ƫ��ֵ��
    XY_t FinalOffset_Last;   //���ո�����̨pid �����ƫ��ֵ��
    XY_t FinalSpeed;         //���ո�����̨pid �����ƫ��ֵ������ٶȡ�
    bool AbleToFire;         //��ʾĿ���Ѿ���׼������ֱ�����
    float FinalOffset_depth; //�����Ӿ����
    VisionSend_IMU_t *VisionSend_IMU;

} VisionExportData_t;

#pragma pack(1)
typedef struct
{
	uint8_t start_tag;		
	uint8_t robot_color;		/*<! ������ID */
	float roll_angle;		/*<! IMU Pit */
	float pit;
	float yaw_angle;	
  uint16_t checksum;

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
extern VisionSendMsg_u Send_Msg; /*<! ���������С���� */
extern uint8_t Vision_DataBuff[Vision_BuffSIZE];
extern VisionExportData_t VisionExportData;
extern VisionData_t VisionData;
extern Control_Vision_FUN_t Control_Vision_FUN;
extern VisionSend_IMU_t VisionSend_IMU;
extern float last_cloud;
extern float offcloud;
extern int To_mode;
//extern WorldTime_RxTypedef VisionKF_TIME;
extern UART_HandleTypeDef *vision_uart; // �ⲿ���
void Vision_classdef_SendToPC(VisionSendMsg_u *pack2vision);

#ifdef Enable_Vision_Test
extern uint8_t Vision_SendBuf[5][5];
void Vision_I_T_Set(uint8_t ID, uint8_t Type);
#endif

#endif