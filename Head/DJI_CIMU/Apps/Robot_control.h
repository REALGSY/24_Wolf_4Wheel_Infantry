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
void Robot_Reset (void);
void Robot_Soft_Reset(void);
typedef enum
{
    ChassisWorkMode_CloudFollow = 0,        //跟随云台模式
    ChassisWorkMode_NewFollow = 1,     //跟随底盘模式
    ChassisWorkMode_Reversefollow = 2, //反向跟随云台模式
    ChassisWorkMode_Spin = 3,          //边走边旋模式
	  ChassisWorkMode_CloudOnly = 4,     //只动头模式
    ChassisWorkMode_Twister,           //扭腰模式
    ChassisWorkMode_AutoTrace,         //自动追踪模式
    ChassisWorkMode_Supply,            //补给模式
    ChassisWorkMode_Lock = 8,
		ChassisWorkMode_Square = 9,
    ChassisWorkMode_Disable //失能模式
} ChassisWorkMode_e;

typedef enum
{
    CloudWorkMode_Normal = 0, //正常模式（360°云台）
    CloudWorkMode_LimitAngle, //角度限制模式（非360°云台）
    CloudWorkMode_SupplyWork, //补给模式
    CloudWorkMode_Disable     //失能
} CloudWorkMode_e;

typedef enum
{
    FrictWorkMode_Disable,   //摩擦轮关闭模式
    FrictWorkMode_HighSpeed, //摩擦轮高速模式
    FrictWorkMode_LowSpeed,  //摩擦轮低速模式
    FrictWorkMode_AutoSpeed  //摩擦轮自动模式

} FrictWorkMode_e;

//机器人控制的来源，非信号来源。
typedef enum
{
    ControlSource_RC = 0, //遥控器模式
    ControlSource_PC = 1, //键鼠模式
    ControlSource_Stop    //关闭机器
} ControlSource_e;


typedef struct 
{
	ChassisWorkMode_e ChassisWorkMode;
	CloudWorkMode_e Sport_CloudWorkMode; 
	FrictWorkMode_e Device_FrictMode; //摩擦轮工作模式。
  ControlSource_e ControlSource; //机器人当前受什么设备控制
}Robot_t;
void Robot_ChangeControlSource(ControlSource_e controlSource);
extern Robot_t Robot;
#endif
