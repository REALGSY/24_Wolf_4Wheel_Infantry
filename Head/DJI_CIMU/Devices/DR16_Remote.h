/**
 * @file DR16_Remote.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __DR16_REMOTE
#define __DR16_REMOTE

#include "dma.h"
#include "usart.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h> //机器人的默认配置文件。

/*调用定义*/
#define DR16BufferNumber 22
#define DR16BufferTruthNumber 18
#define DR16BufferLastNumber 4

#define DR16_ROCKER_MAXVALUE 660 //遥控摇杆最大值

#define KEYMOUSE_AMOUNT 18 //键盘鼠标总和：18个键。

#define TIME_KeyMouse_Press 1 //超过该时间视为 按下。
//在两者之间视为 单击
#define TIME_KeyMouse_LongPress 20 //超过该时间视为 长按

#define DR16_GroundInit            \
    {                              \
        {0},                       \
            {0, 0, 0, 0, 0, 0, 0}, \
            {0, 0, 0, 0, 0},       \
            {0},                   \
    }

#define DR16_ExportDataGroundInit \
    {                             \
        {0, 0},                   \
            {0, 0, 0, {0}},       \
            {0, 0, 0, 0, 0, 0},   \
            &ControlSwitch,       \
            0,                    \
            0,                    \
    }

#define DR16_FunGroundInit           \
    {                                \
        &DR16_Handler,               \
            &DR16_USART_Receive_DMA, \
            &GetKeyMouseAction,      \
            &RemoteControl_Output,   \
            &DR16_DataCheck,         \
            &Check_DR16,             \
    }

bool Calibration_Shoot(void); //给射速擦除使用
/**********************************************/
//遥控拨杆位置选项,s1、s2均适用。
typedef enum
{
    RemotePole_UP = 1,  //上
    RemotePole_MID = 3, //中
    RemotePole_DOWM = 2 //下
} RemotePole_e;

typedef struct
{
    RemotePole_e Left;
    RemotePole_e Right;

} ControlSwitch_t; //遥控器的s1、s2拨杆
/**********************************************/

typedef struct
{
    struct
    {
        float x;
        float y;
    } mouse;

    struct
    {

        uint32_t Press_Flag;                //键鼠按下标志
        uint32_t Click_Press_Flag;          //键鼠单击标志
        uint32_t Long_Press_Flag;           //键鼠长按标志
        uint8_t PressTime[KEYMOUSE_AMOUNT]; //键鼠按下持续时间
    } KeyMouse;                             //鼠标的对外输出。

    struct
    {
        float Forward_Back_Value; //Vx
        float Omega_Value;        //自旋值。
        float Left_Right_Value;   //Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel; //拨轮
    } Robot_TargetValue;  //遥控计算比例后的运动速度
    ControlSwitch_t *ControlSwitch;
    uint16_t infoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} DR16_Export_Data_t;         //供其他文件使用的输出数据。

/*遥控器结构体*/
typedef struct
{
    uint8_t DR16Buffer[DR16BufferNumber];
    struct
    {
        int16_t ch0; //yaw
        int16_t ch1; //pitch
        int16_t ch2; //left_right
        int16_t ch3; //forward_back
        uint8_t s_left;
        uint8_t s_right;
        int16_t ch4_DW; //拨轮
    } rc;               //遥控器接收到的原始值。

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t keyLeft;
        uint8_t keyRight;

    } mouse;

    union
    { //union联合体用法
        uint16_t key_code;
        struct
        { //位域的使用
            bool press_W : 1;
            bool press_S : 1;
            bool press_A : 1;
            bool press_D : 1;

            bool press_Shift : 1;
            bool press_Ctrl : 1;
            bool press_Q : 1;
            bool press_E : 1;

            bool press_R : 1;
            bool press_F : 1;
            bool press_G : 1;
            bool press_Z : 1;

            bool press_X : 1;
            bool press_C : 1;
            bool press_V : 1;
            bool press_B : 1;
        } Key_Code;
    } keyBoard;

    uint16_t infoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} DR16_t;

typedef enum
{
    //与DR16_Export_data.KeyMouse 的flag位一一对应
    KEY_W = 0,
    KEY_S = 1,
    KEY_A,
    KEY_D,
    KEY_SHIFT,
    KEY_CTRL,
    KEY_Q,
    KEY_E,
    KEY_R,
    KEY_F,
    KEY_G,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,
    MOUSE_Left,
    MOUSE_Right
} KeyList_e;

typedef enum
{
    KeyAction_CLICK,
    KeyAction_PRESS,
    KeyAction_LONG_PRESS
} KeyAction_e; //鼠标键盘（键）事件类型。

typedef struct
{
    void (*DR16_Handler)(UART_HandleTypeDef *huart);
    void (*DR16_USART_Receive_DMA)(UART_HandleTypeDef *huart);
    bool (*GetKeyMouseAction)(KeyList_e KeyMouse, KeyAction_e Action);
    void (*RemoteControl_Output)(void);
    int (*DR16_DataCheck)(void);
    void (*Check_DR16)(void);
} DR16_Fun_t;

extern DR16_Export_Data_t DR16_Export_Data;
extern DR16_Fun_t DR16_Fun;
extern DR16_t DR16;
#endif /*__DR16_REMOTE*/
