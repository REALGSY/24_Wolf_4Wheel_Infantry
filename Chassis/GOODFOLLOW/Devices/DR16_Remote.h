/**
 * @file DR16_Remote.h
 * @author Gsy
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
 
#ifndef __DR16_REMOTE__H__
#define __DR16_REMOTE__H__

#include "usart.h"
#include "typedef.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "PID.h"
#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart1 

typedef __packed struct
{
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t roll;
  uint8_t sw1;
  uint8_t sw2;
	uint8_t infoUpdateFrame;
	uint8_t OffLineFlag;
} rc_info_t;

#define rc_Init   \
{                 \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
}


#define DR16_ExportDataGroundInit \
    {                             \
            {0, 0, 0, 0, 0, 0},   \
            &ControlSwitch,       \
            0,                    \
            0,                    \
    }

typedef enum
{
    RemotePole_UP = 1,  //��
    RemotePole_MID = 3, //��
    RemotePole_DOWM = 2 //��
} RemotePole_e;

typedef struct
{
    RemotePole_e Left;
    RemotePole_e Right;

} ControlSwitch_t; //ң������s1��s2����



typedef struct
{
//    struct
//    {
//        float x;
//        float y;
//    } mouse;

//    struct
//    {

//        uint32_t Press_Flag;                //�����±�־
//        uint32_t Click_Press_Flag;          //���󵥻���־
//        uint32_t Long_Press_Flag;           //���󳤰���־
//        uint8_t PressTime[KEYMOUSE_AMOUNT]; //�����³���ʱ��
//    } KeyMouse;                             //���Ķ��������

    struct
    {
        float Forward_Back_Value; //Vx
        float Omega_Value;        //����ֵ��
        float Left_Right_Value;   //Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel; //����
    } Robot_TargetValue;  //ң�ؼ����������˶��ٶ�
    ControlSwitch_t *ControlSwitch;
    uint16_t infoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־ 
		uint8_t Alldead;
} DR16_Export_Data_t;         //�������ļ�ʹ�õ�������ݡ�
void DR16_ROLL_REC(Can_Export_Data_t RxMessage);
void Check_DR16(void);
extern DR16_Export_Data_t DR16_Export_Data;
extern rc_info_t rc;
void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
void RemoteControl_Output(void);
void DR16_CAN2_REC(Can_Export_Data_t RxMessage);
#endif


