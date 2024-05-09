/**
 * @file Robot_control.h
 * @author Gsy 
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __Robot_CONTROL_H
#define __Robot_CONTROL_H

void Robot_Disable(void);
void Cloud_control(void);
void Chassis_Follow_control(void);
void Shot_control (void);
void Robot_control (void);

typedef enum
{
    ChassisWorkMode_CloudFollow = 0,        //������̨ģʽ
    ChassisWorkMode_NewFollow = 1,     //�������ģʽ
    ChassisWorkMode_Reversefollow = 2, //���������̨ģʽ
    ChassisWorkMode_Spin = 3,          //���߱���ģʽ
	  ChassisWorkMode_CloudOnly = 4,     //ֻ��ͷģʽ
    ChassisWorkMode_Twister,           //Ť��ģʽ
    ChassisWorkMode_AutoTrace,         //�Զ�׷��ģʽ
    ChassisWorkMode_Supply,            //����ģʽ
    ChassisWorkMode_Lock,
	  ChassisWorkMode_Square = 9,
    ChassisWorkMode_Disable //ʧ��ģʽ
} ChassisWorkMode_e;

typedef enum
{
    CloudWorkMode_Normal = 0, //����ģʽ��360����̨��
    CloudWorkMode_LimitAngle, //�Ƕ�����ģʽ����360����̨��
    CloudWorkMode_SupplyWork, //����ģʽ
    CloudWorkMode_Disable     //ʧ��
} CloudWorkMode_e;

typedef enum
{
    FrictWorkMode_Disable,   //Ħ���ֹر�ģʽ
    FrictWorkMode_HighSpeed, //Ħ���ָ���ģʽ
    FrictWorkMode_LowSpeed,  //Ħ���ֵ���ģʽ
    FrictWorkMode_AutoSpeed  //Ħ�����Զ�ģʽ

} FrictWorkMode_e;

typedef struct 
{
	ChassisWorkMode_e ChassisWorkMode;
	CloudWorkMode_e Sport_CloudWorkMode; 
	FrictWorkMode_e Device_FrictMode; //Ħ���ֹ���ģʽ��
}Robot_t;

extern Robot_t Robot;
#endif
