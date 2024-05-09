#ifndef DEV_SUPCAP_H
#define DEV_SUPCAP_H


#include <stdint.h>
#include "BSP_CAN.h"
#include <stdbool.h>
#include "typedef.h"
#define SCCM_RECEIVE_ID 0x600
#define SCCM_SEND_ID 0x601

#define SupCap_ON 1
#define SupCap_OFF 0
#define Charging_ON 1
#define Charging_OFF 0
#define Power_Supply 1
#define Power_NotSupply 0

typedef union
{
	uint8_t data[8];
    struct
    {
        float chassis_power;  /* ���̹��ʣ���λ��W */
        uint8_t chassis_buff; /* ���̹��ʻ��� */
        uint8_t Is_enable;    /* ���ݿ��Խ������ */
        uint8_t cap_cell;     /* ����ʣ�����������ָ��� */
    }Pack;
		
} SCCM_RecvData_u;

typedef union
{
		uint8_t data[8];
    struct
    {
        float charge_power;    /* ��繦�ʣ���λ��W ,��Χ 0-80W */
        uint8_t charge_enable; /* ���ʹ�� */
        uint8_t is_cap_output; /* ʹ�õ��ݹ��� */
    }Pack;

} SCCM_SendData_u;

typedef struct
{
    uint8_t Data[8];
    uint8_t State;
    uint16_t ChassisPower_Limit;
    float Charging_Power;      // ���Ĺ���
    SCCM_RecvData_u RecvData;
    SCCM_SendData_u SendData;
} SupCap_classdef;

typedef struct  
{
		float infoUpdateFrame;
		float OffLineFlag;
}SupCap_t;

void SupCap_Control(void);
void SupCap_Process(void);
void SupCap_Send_Msg(CAN_HandleTypeDef *hcan);
void SupCap_Main_Switch( bool Ctrl, bool Charge, bool Supply);
void SupCap_ChargeControl(float Charging_Power);
void SupCap_ChargeSwitch(bool Switch);
void SupCap_SupplySwitch(bool Switch);
uint8_t SupCap_Get_Cell(void);
extern uint8_t SupCap_C_Flag ;
extern SupCap_classdef Supcap_Rec;
bool SupCap_Is_CanbeUse(void);
bool SupCap_Is_Output(void);
void Check_SupCap(void);
void SupCap_Update(Can_Export_Data_t RxMessage);
#endif
