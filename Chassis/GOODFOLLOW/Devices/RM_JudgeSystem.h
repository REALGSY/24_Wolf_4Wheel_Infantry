/**
 * @file RM_JudgeSystem.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _RM_JUDGESYSTEM_H
#define _RM_JUDGESYSTEM_H

#include "dma.h"
#include "usart.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "M3508_Motor.h"
#include "Robot_control.h"
#include "Control_Vision.h"

#pragma anon_unions

#define JudgeSystem_FUNGroundInit       \
    {                                   \
        &JudgeSystem_USART_Receive_DMA, \
            &JudgeSystem_Handler,       \
            &Check_Judge,               \
    }

typedef struct
{
    void (*JudgeSystem_USART_Receive_DMA)(UART_HandleTypeDef *huartx);
    void (*JudgeSystem_Handler)(UART_HandleTypeDef *huart);
    void (*Check_Judge)(void);
} JudgeSystem_FUN_t;
extern JudgeSystem_FUN_t JudgeSystem_FUN;

typedef struct
{
    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} RM_Judge_t;
extern RM_Judge_t RM_Judge;

float GetVision_CP(void);

//裁判系统预编译
#define JUDGE_20 20
#define JUDGE_21 21
#define JUDGE_VERSION JUDGE_21

#define JUDGE_DATA_ERROR 0
#define JUDGE_DATA_CORRECT 1
/*-------------------------------2020------------------------------------*/
#if JUDGE_VERSION == JUDGE_20
//__packed是进行一字节对齐。使用_packed一般会以降低运行性能为代价，由于大多数cpu处理数据在合适的字节边界数的情况下会更有效，packed的使用会破坏这种自然的边界数
// 2020裁判系统官方接口协议
typedef struct
{
    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} RM_Judge_t;

#define JUDGESYSTEM_PACKSIZE 303u    //裁判系统包大小
#define JUDGESYSTEM_FRAMEHEADER 0xA5 //帧头
//红蓝方
#define BLUE 0
#define RED 1

//信息传输
#define TRUE 1
#define FALSE 0

//发送对象
void Send_Infantry(void);
void Send_Hero(void);
void Send_Sentry(void);
void Send_Engineering(void);

#define JudgeInfoType_Game_state 0x0001
#define JudgeInfoType_Game_result 0x0002
#define JudgeInfoType_Game_robot_HP 0x0003
#define JudgeInfoType_Dart_status 0x0004
#define JudgeInfoType_ICRA_buff 0x0005
#define JudgeInfoType_Event_data 0x0101
#define JudgeInfoType_Supply_projectile_action 0x0102
#define JudgeInfoType_Referee_warning 0x0104
#define JudgeInfoType_Dart_remaining_time 0x0105
#define JudgeInfoType_Game_robot_state 0x0201
#define JudgeInfoType_Power_heat_data 0x0202
#define JudgeInfoType_Game_robot_pos 0x0203
#define JudgeInfoType_Buff_musk 0x0204
#define JudgeInfoType_Robot_energy 0x0205
#define JudgeInfoType_Robot_hurt 0x0206
#define JudgeInfoType_Shoot_data 0x0207
#define JudgeInfoType__Bullet_remaining 0x0208
#define JudgeInfoType___Rfid_status 0x0209
#define JudgeInfoType___Dart_client 0x020A

//整个数据帧的长度，FrameHeader(5-Byte)+CmdID(2-Byte)+Data(n-Byte)+FrameTail(2-Byte, CRC16, 整包校验)
#define JudgeInfoLength_Game_state 12
#define JudgeInfoLength_Game_result 10
#define JudgeInfoLength_Game_robot_HP 41
#define JudgeInfoLength_Dart_status 12
#define JudgeInfoLength_ICRA_buff 12
#define JudgeInfoLength_Event_data 13
#define JudgeInfoLength_Supply_projectile_action 13
#define JudgeInfoLength_Referee_warning 11
#define JudgeInfoLength_Dart_remaining_time 10
#define JudgeInfoLength_Game_robot_state 27
#define JudgeInfoLength_Power_heat_data 25
#define JudgeInfoLength_Game_robot_pos 25
#define JudgeInfoLength_Buff_musk 10
#define JudgeInfoLength_Robot_energy 12
#define JudgeInfoLength_Robot_hurt 10
#define JudgeInfoLength_Shoot_data 15
#define JudgeInfoLength_Bullet_remaining 11
#define JudgeInfoLength_Rfid_status 13
#define JudgeInfoLength_Dart_client 21

/************************************************裁判系统结构体*******************************************************/
typedef struct
{
    uint16_t infoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} Judge_Monitor_t;

//1.比赛状态数据，频率 1Hz 推送 CmdID 0x0001
typedef struct
{
    union
    {
        uint8_t dataBuff[3];
        __packed struct
        {
            uint8_t game_type : 4;      //比赛类型
            uint8_t game_progress : 4;  //当前比赛阶段
            uint16_t stage_remain_time; //当前阶段剩余时间  单位s
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_game_state_t;

//2.比赛结果数据：0x0002。发送频率：比赛结束后发送
typedef struct
{
    union
    {
        uint8_t dataBuff[1];
        __packed struct
        {
            uint8_t winner; //0 平局 1 红方胜利 2 蓝方胜利
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_game_result_t;

//3.机器人血量数据：0x0003。发送频率：1Hz
typedef struct
{
    union
    {
        uint8_t dataBuff[32];
        __packed struct
        {
            uint16_t red_1_robot_HP;  //红1英雄机器人血量，未上场以及罚下血量为0
            uint16_t red_2_robot_HP;  //红2工程机器人血量
            uint16_t red_3_robot_HP;  //红3步兵机器人血量
            uint16_t red_4_robot_HP;  //红4步兵机器人血量
            uint16_t red_5_robot_HP;  //红5步兵机器人血量
            uint16_t red_7_robot_HP;  //红7哨兵机器人血量
            uint16_t red_outpost_HP;  //红方前哨站血量
            uint16_t red_base_HP;     //红方基地血量
            uint16_t blue_1_robot_HP; //蓝1英雄机器人血量
            uint16_t blue_2_robot_HP; //蓝2工程机器人血量
            uint16_t blue_3_robot_HP; //蓝3步兵机器人血量
            uint16_t blue_4_robot_HP; //蓝4步兵机器人血量
            uint16_t blue_5_robot_HP; //蓝5步兵机器人血量
            uint16_t blue_7_robot_HP; //蓝7哨兵机器人血量
            uint16_t blue_outpost_HP; //蓝方前哨站血量
            uint16_t blue_base_HP;    //蓝方基地血量
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_game_robot_HP_t;

//4.飞镖发射状态：0x0004 飞镖发射时发送
typedef struct
{
    union
    {
        uint8_t dataBuff[3];
        __packed struct
        {
            uint8_t dart_belong;           //发射飞镖的队伍    1 红方 2 蓝方
            uint16_t stage_remaining_time; //发射时的剩余比赛时间 单位s
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_dart_status_t;

//5.人工智能挑战赛加成与惩戒区状态 0x005   1hz发送
typedef struct
{
    union
    {
        uint8_t dataBuff[3];
        __packed struct
        {
            uint8_t F1_zone_status : 1;
            uint8_t F1_zone_buff_debuff_status : 3;
            uint8_t F2_zone_status : 1;
            uint8_t F2_zone_buff_debuff_status : 3;
            uint8_t F3_zone_status : 1;
            uint8_t F3_zone_buff_debuff_status : 3;
            uint8_t F4_zone_status : 1;
            uint8_t F4_zone_buff_debuff_status : 3;
            uint8_t F5_zone_status : 1;
            uint8_t F5_zone_buff_debuff_status : 3;
            uint8_t F6_zone_status : 1;
            uint8_t F6_zone_buff_debuff_status : 3;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_ICRA_buff_debuff_zone_status_t;

//6.场地事件数据：0x0101。发送频率：1hz周期发送
typedef struct
{
    union
    {
        uint8_t dataBuff[4];
        __packed struct
        {
            uint32_t event_type; //详情见裁判系统协议P9-10
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_event_data_t;

//7.补给站动作标识：0x0102。发送频率：动作触发后发送
typedef struct
{
    union
    {
        uint8_t dataBuff[4];
        __packed struct
        {
            uint8_t supply_projectile_id;   //补给站口 ID： 1：1号补给口  2:2号补给口
            uint8_t supply_robot_id;        //补弹机器人 ID
                                            /*0为当前无机器人补弹，1为红方英雄机器人补弹，2为红方工程机器人补弹，
			    3 / 4 / 5为红方步兵机器人补弹，101为蓝方英雄机器人补弹，102为蓝方工程机器人补弹，
				             103 / 104 / 105为蓝方步兵机器人补弹*/
            uint8_t supply_projectile_step; //出弹口开闭状态 0为关闭 1为子弹准备中 2为子弹下落
            uint8_t supply_projectile_num;  //补弹数量   50颗 100颗 150颗 200颗
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_supply_projectile_action_t;

//8.裁判警告数据：0x0104。 发送频率：警告发生后发送
typedef struct
{
    union
    {
        uint8_t dataBuff[2];
        __packed struct
        {
            uint8_t level;         //警告等级
            uint8_t foul_robot_id; //犯规机器人ID  1级与5级警告时 ID为0
                                   //  2--4级警告时   ID为犯规机器人ID
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_referee_warning_t;

//9.飞镖发射口倒计时 0x0105  发送频率：1hz
typedef struct
{
    union
    {
        uint8_t dataBuff[1];
        __packed struct
        {
            uint8_t dart_remaining_time; //15s 倒计时
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_dart_remaining_time_t;

//10.机器人状态数据：0x0201。发送频率：10Hz 发送范围：单一机器人
typedef struct
{
    union
    {
        uint8_t dataBuff[18];
        __packed struct
        {
            uint8_t robot_id;                       //机器人 ID
            uint8_t robot_level;                    //机器人等级  1  2   3
            uint16_t remain_HP;                     //机器人剩余血量
            uint16_t max_HP;                        //机器人上限血量
            uint16_t shooter_heat0_cooling_rate;    //机器人 17mm 枪口每秒冷却值
            uint16_t shooter_heat0_cooling_limit;   //机器人 17mm 枪口热量上限
            uint16_t shooter_heat1_cooling_rate;    //机器人 42mm 枪口每秒冷却值
            uint16_t shooter_heat1_cooling_limit;   //机器人 42mm 枪口热量上限
            uint8_t shooter_heat0_speed_limit;      //机器人 17mm 枪口上限速度 单位 m/s
            uint8_t shooter_heat1_speed_limit;      //机器人 42mm 枪口上限速度 单位 m/s
            uint8_t max_chassis_power;              //机器人最大底盘  单位W
            uint8_t mains_power_gimbal_output : 1;  //gimbal口输出： 1为有24V输出， 0为无24v输出；
            uint8_t mains_power_chassis_output : 1; //chassis口输出： 1为有24V输出，0为无24v输出；
            uint8_t mains_power_shooter_output : 1; //shooter口输出： 1为有24V输出，0为无24v输出；
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_game_robot_state_t;

//11.实时功率热量数据：0x0202。发送频率：50Hz  发送范围：单一机器人
typedef struct
{
    union
    {
        uint8_t dataBuff[16];
        __packed struct
        {
            uint16_t chassis_volt;         //底盘输出电压 单位 毫伏
            uint16_t chassis_current;      //底盘输出电流 单位 毫安
            float chassis_power;           //底盘输出功率 单位 W 瓦
            int16_t chassis_power_buffer;  //底盘功率缓冲 单位 J 焦耳   备注：飞坡根据规则增加至 250J
            uint16_t shooter_heat0;        //17mm 枪口热量
            uint16_t shooter_heat1;        //42mm 枪口热量
            uint16_t mobile_shooter_heat2; //机动 17mm枪口热量
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_power_heat_data_t;

//12. 机器人位置：0x0203。发送频率：10Hz  发送范围：单一机器人
typedef struct
{
    union
    {
        uint8_t dataBuff[16];
        __packed struct
        {
            float x;   //位置 x 坐标，单位 m
            float y;   //位置 y 坐标，单位 m
            float z;   //位置 z 坐标，单位 m
            float yaw; //位置枪口，单位度
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_game_robot_pos_t;

//13.机器人增益：0x0204。发送频率：1hz  发送范围：单一机器人
typedef struct
{
    union
    {
        uint8_t dataBuff[1];
        __packed struct
        {
            uint8_t power_rune_buff; /*bit 0：机器人血量补血状态
			                              bit 1：枪口热量冷却加速
				                          bit 2：机器人防御加成
				                          bit 3：机器人攻击加成
				                                      其他 bit 保留*/
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_buff_musk_t;

//14. 空中机器人能量状态：0x0205。发送频率：10Hz   只有空中机器人主控发送
typedef struct
{
    union
    {
        uint8_t dataBuff[3];
        __packed struct
        {
            uint8_t energy_point; //积累的能量点
            uint8_t attack_time;  //可攻击时间 单位 s。30s 递减至 0
        };
    } data;
    uint8_t infoUpdateFlag;
} aerial_robot_energy_t;

//15. 伤害状态：0x0206。发送频率：伤害发生后发送   发送范围：单一机器人
typedef struct
{
    union
    {
        uint8_t dataBuff[1];
        __packed struct
        {
            uint8_t armor_id : 4;
            uint8_t hurt_type : 4;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_robot_hurt_t;

//16. 实时射击信息：0x0207。发送频率：射击后发送 发送范围：单一机器人
typedef struct
{
    union
    {
        uint8_t dataBuff[6];
        __packed struct
        {
            uint8_t bullet_type; //子弹类型 1:17mm 2:42mm
            uint8_t bullet_freq; //子弹射频  单位hz
            float bullet_speed;  //子弹射速  单位 m/s
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_shoot_data_t;

//17.子弹剩余发射数 0x0208  发送频率：1hz 空中机器人，哨兵机器人以及 ICRA 机器人主控发送，发送范围：单一机器人。
typedef struct
{
    union
    {
        uint8_t dataBuff[2];
        __packed struct
        {
            uint16_t bullet_remaining_num;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_bullet_remaining_t;

//18.机器人RFID状态 发送频率:1hz
typedef struct
{
    union
    {
        uint8_t dataBuff[4];
        __packed struct
        {
            uint32_t rfid_status; //详见裁判系统协议P17
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_rfid_status_t;

//19.飞镖机器人客户端指令数据   0x020A 发送频率:10hz
typedef struct
{
    union
    {
        uint8_t dataBuff[12];
        __packed struct
        {
            uint8_t dart_launch_opening_status; //当前飞镖发射口状态 0 关闭 1正在开启或者关闭 2 已经开启
            uint8_t dart_attack_target;         //飞镖的打击目标 默认为前哨站 1：前哨站 2：基地
            uint16_t target_change_time;        //切换打击目标时的比赛剩余时间，单位秒，从未切换默认为0。
            uint8_t first_dart_speed;           //检测到的第一枚飞镖速度，单位 0.1m/s/LSB, 未检测是为0。
            uint8_t second_dart_speed;          //检测到的第二枚飞镖速度，单位 0.1m/s/LSB，未检测是为0。
            uint8_t third_dart_speed;           //检测到的第三枚飞镖速度，单位 0.1m /s/LSB，未检测是为0。
            uint8_t fourth_dart_speed;          //检测到的第四枚飞镖速度，单位 0.1m/s/LSB，未检测是为0。
            uint16_t last_dart_launch_time;     //最近一次的发射飞镖的比赛剩余时间，单位秒，初始值为0。
            uint16_t operate_launch_cmd_time;   //最近一次操作手确定发射指令时的比赛剩余时间，单位秒,初始值为0。
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_dart_client_cmd_t;

//参赛队自定义数据，用于显示在操作界面，限频 10Hz CmdID 0x0100
typedef struct
{
    union
    {
        uint8_t dataBuff[13];
        struct
        {
            float data1;
            float data2;
            float data3;
            uint8_t mask;
        };
    } data;
    uint8_t infoUpdateFlag;

} Judge_SendPC_t;

/* 自定义帧头 */
typedef __packed struct
{
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;

} xFrameHeader;

/* 交互数据接收信息：0x0301  */
typedef __packed struct
{
    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/* 
	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180
	发送频率：上限 10Hz


	1.	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz 
	字节偏移量 	大小 	说明 				备注 
	0 			2 		数据的内容 ID 		0xD180 
	2 			2 		送者的 ID 			需要校验发送者机器人的 ID 正确性 
	4 			2 		客户端的 ID 		只能为发送者机器人对应的客户端 
	6 			4 		自定义浮点数据 1 	 
	10 			4 		自定义浮点数据 2 	 
	14 			4 		自定义浮点数据 3 	 
	18 			1 		自定义 8 位数据 4 	 

*/
typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t masks;
} client_custom_data_t;

typedef __packed struct
{
    uint8_t data[10]; //数据段,n需要小于113
} robot_interactive_data_t;

//机器人交互信息
typedef __packed struct
{
    xFrameHeader txFrameHeader;                            //帧头
    uint16_t CmdID;                                        //命令码
    ext_student_interactive_header_data_t dataFrameHeader; //数据段头结构
    robot_interactive_data_t interactData;                 //数据段
    uint16_t FrameTail;                                    //帧尾
} ext_CommunatianData_t;

//帧头  命令码   数据段头结构  数据段   帧尾
//上传客户端
typedef __packed struct
{
    xFrameHeader txFrameHeader;                            //帧头
    uint16_t CmdID;                                        //命令码
    ext_student_interactive_header_data_t dataFrameHeader; //数据段头结构
    client_custom_data_t clientData;                       //数据段
    uint16_t FrameTail;                                    //帧尾
} ext_SendClientData_t;

typedef struct
{
    union
    {
        uint8_t dataBuff[13];
        struct
        {
            float data1;
        };
    } data;
    uint8_t infoUpdateFlag;

} Judge_SendRobot_t;

extern ext_game_state_t ext_game_state;
extern ext_game_result_t ext_game_result;
extern ext_game_robot_HP_t ext_game_robot_HP;
extern ext_dart_status_t ext_dart_status;
extern ext_ICRA_buff_debuff_zone_status_t ext_ICRA_buff_debuff_zone_status;
extern ext_supply_projectile_action_t ext_supply_projectile_action;
extern ext_referee_warning_t ext_referee_warning;
extern ext_dart_remaining_time_t ext_dart_remaining_time;
extern ext_game_robot_state_t ext_game_robot_state;
extern ext_power_heat_data_t ext_power_heat_data;
extern ext_game_robot_pos_t ext_game_robot_pos;
extern ext_buff_musk_t ext_buff_musk;
extern aerial_robot_energy_t aerial_robot_energy;
extern ext_robot_hurt_t ext_robot_hurt;
extern ext_shoot_data_t ext_shoot_data;
extern ext_bullet_remaining_t ext_bullet_remaining;
extern ext_rfid_status_t ext_rfid_status;
extern ext_dart_client_cmd_t ext_dart_client_cmd;
extern Judge_Monitor_t Judge_Monitor;
extern Judge_SendPC_t Judge_SendData;
extern Judge_SendRobot_t Judge_SendRobot;
extern uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE];
extern RM_Judge_t RM_Judge;

void JudgeSystem_Init(void);
void Judge_getInfo(uint16_t dataLength);
void Judge_sendPC(void);
void Judge_sendRobot(uint16_t cmd);
_Bool is_red_or_blue(void);
void determine_ID(void);

/*-------------------------------2021------------------------------------*/
#elif JUDGE_VERSION == JUDGE_21
//2021 裁判系统官方接口协议

//对应通信协议格式   frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16,整包校验)
#define       LEN_HEADER        5/*frame_header*/
#define       LEN_CMDID         2/*cmd_id*/
#define       LEN_TAIL          2/*frame_tail*/

//起始字节，协议固定为0xA5
#define      JUDGE_FRAME_HEADER            0xA5
#define      JUDGESYSTEM_PACKSIZE 		    389u		//裁判系统包大小(354+35)

//红蓝方
#define BLUE 0
#define RED  1

//信息传输
#define TRUE 1
#define FALSE 0


/***********************************命令码ID*************************************/
/*

ID: 0x0001          Byte: 11      比赛状态数据                   发送频率 1Hz
ID: 0x0002          Byte: 1       比赛结果数据                   比赛结束后发送
ID: 0x0003          Byte: 32      比赛机器人血量数据             发送频率 1Hz
ID: 0x0004          Byte: 3       飞镖发射状态                   飞镖发射后发送
ID: 0x0005          Byte: 11      人工智能挑战赛加成与惩罚       发送频率 1Hz
ID: 0x0101          Byte: 4       场地事件数据                   事件改变后发送
ID: 0x0102          Byte: 4       场地补给站动作标识数据         动作改变后发送
//ID: 0X0103          Byte: 2       场地补给站预约子弹数据         参赛队发送，上限10Hz（RM对抗赛未开放） 
ID: 0X0104          Byte: 2       裁判警告数据                   警告发生后发送
ID: 0x0105          Byte: 1       飞镖发射口倒计时               发送频率 1Hz
ID: 0X0201          Byte: 27      机器人状态数据                 发送频率 10Hz
ID: 0X0202          Byte: 16      实时功率热量数据               发送频率 50Hz
ID: 0X0203          Byte: 16      机器人位置数据                 发送频率 10Hz
ID: 0X0204          Byte: 1       机器人增益数据                 增益状态改变后发送
ID: 0X0205          Byte: 2       空中机器人能量状态数据         发送频率 10Hz，只有空中机器人主控发送
ID: 0X0206          Byte: 1       伤害状态数据                   伤害发生后发送
ID: 0X0207          Byte: 7       实时射击数据                   子弹发射后发送
ID: 0X0208          Byte: 6       子弹剩余发送数                 无人机与哨兵发送  发送频率 1Hz
ID: 0X0209          Byte: 4       机器人RFID状态                 发送频率 1Hz
ID: 0x020A          Byte: 12      飞镖机器人客户端指令书         发送频率 10Hz
ID: 0X0301          Byte: n       机器人间交互数据               发送方触发发送    上限频率 10Hz
ID: 0x0302          Byte: n       自定义控制器交互数据接口       客户端触发发送    上限频率 30Hz
ID: 0x0303          Byte: 15      客户端小地图交互数据           触发发送
ID: 0x0304          Byte: 12      键盘、鼠标信息                 通过图传串口发送

*/

#define       Judge_Game_StatusData              0x0001 
#define       Judge_Game_ResultData              0x0002 
#define       Judge_Robot_HP                     0x0003 
#define       Judge_Dart_Launch                  0x0004
#define       Judge_AI_ChallengeBuff             0x0005
#define       Judge_Event_Data                   0x0101
#define       Judge_Supply_Station               0x0102
//#define       Judge_Request_Recharge             0x0103(对抗赛未开放)
#define       Judge_Referee_Warning              0x0104
#define       Judge_Dart_Countdown               0x0105
#define       Judge_Robot_State                  0x0201
#define       Judge_Power_Heat                   0x0202
#define       Judge_Robot_Position               0x0203
#define       Judge_Robot_Buff                   0x0204
#define       Judge_Aerial_Energy                0x0205
#define       Judge_Injury_State                 0x0206
#define       Judge_RealTime_Shoot               0x0207
#define       Judge_Remaining_Rounds             0x0208
#define       Judge_Robot_RFID                   0x0209
#define       Judge_Dart_Client                  0x020A
#define       Judge_Robot_Communicate            0x0301
#define       Judge_User_Defined                 0x0302
#define       Judge_Map_Interaction              0x0303
#define       Judge_KeyMouse_Message             0x0304
#define       Judge_Client_Map                   0x0305


/***************************DATA_Length*************************/
/*Calculation:frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16)*/

#define       JudgeLength_Game_StatusData        20
#define       JudgeLength_Game_ResultData        10
#define       JudgeLength_Robot_HP               41
#define       JudgeLength_Dart_Launch            12
#define       JudgeLength_AI_ChallengeBuff       20
#define       JudgeLength_Event_Data             13
#define       JudgeLength_Supply_Station         13
//#define       JudgeLength_Request_Recharge       11(对抗赛未开放)
#define       JudgeLength_Referee_Warning        11
#define       JudgeLength_Dart_Countdown         10
#define       JudgeLength_Robot_State            22
#define       JudgeLength_Power_Heat             25
#define       JudgeLength_Robot_Position         25
#define       JudgeLength_Robot_Buff             10
#define       JudgeLength_Aerial_Energy          10
#define       JudgeLength_Injury_State           10
#define       JudgeLength_RealTime_Shoot         16
#define       JudgeLength_Remaining_Rounds       15
#define       JudgeLength_Robot_RFID             13
#define       JudgeLength_Dart_Client            21
#define       JudgeLength_Robot_Commute          35
#define       JudgeLength_Robot_Map              26

/**************************************自定义UI机器人指示信息文字*****************************************/

#define       LED_YELLOWMeaning                 "Fric:Manual"
#define       LED_GREENMeaning                  "Shot:Spin:Maga"
#define       LED_ORANGEMeaning                 "ChassisMode:Normal"
#define       LED_PURPLEMeaning                 "Square:USE_SUP:SUP"
#define       LED_PINKMeaning                   "   NO!!       "

#define       LED_CYANMeaning                   "Resever"
#define       LED_BLACKMeaning                  "SuperCap"
#define       LED_WHITEMeaning                  "Windmill"
#define       LED_Init                          "         "

extern int YellowFlag;
extern int GreenFlag;
extern int OrangeFlag;
extern int PurpleFlag;
extern int PinkFlag;
/**************************************裁判系统结构体*****************************************/
/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
}xFrameHeader;

/* ID: 0x0001    Byte: 11     比赛状态数据 */
typedef struct
{
  union {
		uint8_t dataBuff[11];
		__packed struct {
			uint8_t game_type : 4;             //比赛类型
			uint8_t game_progress : 4;         //当前比赛阶段
			uint16_t stage_remain_time;        //当前阶段剩余时间  单位s
		};
	}data;
	uint8_t infoUpdateFlag;
}ext_game_status_t;

/* ID: 0x0002    Byte: 1       比赛结果数据 */
typedef struct
{
	union{
	  uint8_t dataBuff[1];
		__packed struct
		{
			uint8_t winner;//0平局 1红胜 2蓝胜
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_result_t;

/* ID: 0x0003     Byte: 32     比赛机器人血量数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[32];
		__packed struct
		{
	     uint16_t red_1_robot_HP;//红 1 英雄机器人血量，未上场以及罚下血量为 0
       uint16_t red_2_robot_HP;//红 2 工程机器人血量
       uint16_t red_3_robot_HP;//红 3 步兵机器人血量
       uint16_t red_4_robot_HP;//红 4 步兵机器人血量
       uint16_t red_5_robot_HP;//红 5 步兵机器人血量
       uint16_t red_7_robot_HP;//红 7 步兵机器人血量
       uint16_t red_outpost_HP;//红方前哨战血量
       uint16_t red_base_HP;//红方基地血量
       uint16_t blue_1_robot_HP;
       uint16_t blue_2_robot_HP; 
       uint16_t blue_3_robot_HP; 
       uint16_t blue_4_robot_HP; 
       uint16_t blue_5_robot_HP;
       uint16_t blue_7_robot_HP;
       uint16_t blue_outpost_HP;
       uint16_t blue_base_HP;			
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_robot_HP_t;

/* ID: 0x0004      Byte: 3    飞镖发射状态 */
typedef struct
{
	union
	{
		uint8_t dataBuff[3];
		__packed struct
		{
			uint8_t dart_belong;//发射飞镖的队伍：1：红方飞镖2：蓝方飞镖
			uint16_t stage_remaining_time;//发射时的剩余比赛时间，单位 s
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_dart_status_t;

/* ID: 0x0005      Byte: 11      人工智能挑战赛加成与惩罚 */
typedef struct
{
	union
	{
		uint8_t dataBuff[11];
		__packed struct
		{
			uint8_t F1_zone_status:1;
			uint8_t F1_zone_buff_debuff_status:3;
			uint8_t F2_zone_status:1;
			uint8_t F2_zone_buff_debuff_status:3;
			uint8_t F3_zone_status:1;
			uint8_t F3_zone_buff_debuff_status:3; 
			uint8_t F4_zone_status:1;
			uint8_t F4_zone_buff_debuff_status:3; 
			uint8_t F5_zone_status:1;
			uint8_t F5_zone_buff_debuff_status:3;
			uint8_t F6_zone_status:1;
			uint8_t F6_zone_buff_debuff_status:3;
			uint16_t red1_bullet_left;
			uint16_t red2_bullet_left;
			uint16_t blue1_bullet_left;
			uint16_t blue2_bullet_left;
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_ICRA_buff_debuff_zone_status_t;

/* ID: 0x0101     Byte: 4       场地事件数据 */
typedef struct
{
	union
	{
		uint8_t dataBuff[4];
		__packed struct
		{
			uint32_t event_type;
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_event_data_t;

/* ID: 0x0102     Byte: 4       场地补给站动作标识数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[4];
		__packed struct
		{
			uint8_t supply_projectile_id;  //补给站口 ID
			uint8_t supply_robot_id;       //补弹机器人 ID
			uint8_t supply_projectile_step;//出弹口开闭状态 
			uint8_t supply_projectile_num; //补弹数量
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_supply_projectile_action_t;

/* ID: 0X0104          Byte: 2       裁判警告数据 */
typedef struct
{
	union
	{
    uint8_t dataBuff[2];
	  __packed struct
	 {
		 uint8_t level;//警告等级：
		 uint8_t foul_robot_id; //犯规机器人 ID：判负时，机器人 ID 为 0 黄牌、红牌时，机器人 ID 为犯规机器人 ID
	 };
	}data;
	uint8_t InfoUpdataFlag;
}ext_referee_warning_t;

/* ID: 0x0105          Byte: 1       飞镖发射口倒计时  */
typedef struct
{
	union
	{
		uint8_t dataBuff[1];
		__packed struct
		{
			uint8_t dart_remaining_time;//15s 倒计时
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_dart_remaining_time_t;

/* ID: 0X0201          Byte: 27      机器人状态数据 */
typedef struct
{
	union
	{
		uint8_t dataBuff[27];
		__packed struct
		{
		  uint8_t robot_id;
      uint8_t robot_level;
      uint16_t remain_HP;//机器人剩余血量
      uint16_t max_HP;//机器人上限血量
      uint16_t shooter_id1_17mm_cooling_rate; //机器人 1 号 17mm 枪口每秒冷却值
      uint16_t shooter_id1_17mm_cooling_limit;//机器人 1 号 17mm 枪口热量上限
      uint16_t shooter_id1_17mm_speed_limit;  //机器人 1 号 17mm 枪口上限速度 单位 m/s
      uint16_t shooter_id2_17mm_cooling_rate;
      uint16_t shooter_id2_17mm_cooling_limit;
      uint16_t shooter_id2_17mm_speed_limit;
      uint16_t shooter_id1_42mm_cooling_rate;
      uint16_t shooter_id1_42mm_cooling_limit;
      uint16_t shooter_id1_42mm_speed_limit;
      uint16_t chassis_power_limit;           //机器人底盘功率限制上限
			/*主控电源输出情况：
       0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
       1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
       2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；*/
      uint8_t mains_power_gimbal_output : 1;
      uint8_t mains_power_chassis_output : 1;
      uint8_t mains_power_shooter_output : 1;	
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_robot_status_t;

/* ID: 0X0202          Byte: 16      实时功率热量数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[16];
		__packed struct
		{
			uint16_t chassis_volt; //底盘输出电压 单位 毫伏
      uint16_t chassis_current; //底盘输出电流 单位 毫安
      float chassis_power;//底盘输出功率 单位 W 瓦
      uint16_t chassis_power_buffer;//底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
      uint16_t shooter_id1_17mm_cooling_heat;//枪口热量
      uint16_t shooter_id2_17mm_cooling_heat;
      uint16_t shooter_id1_42mm_cooling_heat;	
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_power_heat_data_t;

/* ID: 0X0203          Byte: 16      机器人位置数据  */
typedef struct
{
  union
	{
		uint8_t dataBuff[16];
		__packed struct
		{
		 float x;//位置 x 坐标
     float y;//位置 y 坐标
     float z;//位置 z 坐标
     float yaw;//位置枪口
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_robot_pos_t;

/* ID: 0X0204          Byte: 1       机器人增益数据 */
typedef struct
{
	union
	{
		uint8_t dataBuff[1];
		__packed struct
		{
			uint8_t power_rune_buff;/*bit 0：机器人血量补血状态
                                bit 1：枪口热量冷却加速
                                bit 2：机器人防御加成
                                bit 3：机器人攻击加成*/
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_buff_t;
/* ID: 0X0205          Byte: 2       空中机器人能量状态数据 */
typedef struct
{
	union
	{
		uint8_t dataBuff[2];
		__packed struct
		{
			uint8_t attack_time;//可攻击时间 单位 s。30s 递减至 0
		};
	}data;
	uint8_t InfoUpdataFlag;
}aerial_robot_energy_t;

/* ID: 0X0206          Byte: 1       伤害状态数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[1];
		__packed struct
		{
		  uint8_t armor_id : 4;/*bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的
                             五个装甲片，其他血量变化类型，该变量数值为 0。*/
      uint8_t hurt_type : 4;/*bit 4-7：血量变化类型,0x0 装甲伤害扣血；
                              0x1 模块掉线扣血；
                              0x2 超射速扣血；
                              0x3 超枪口热量扣血；
                              0x4 超底盘功率扣血；
                              0x5 装甲撞击扣血*/
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_robot_hurt_t;

/* ID: 0X0207          Byte: 7       实时射击数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[7];
		__packed struct
		{
		  uint8_t bullet_type;//子弹类型: 1：17mm 弹丸 2：42mm 弹丸
      uint8_t shooter_id;//发射机构 ID
      uint8_t bullet_freq;//子弹射频 单位 Hz
      float bullet_speed;//子弹射速 单位 m/s
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_shoot_data_t;

/* ID: 0X0208          Byte: 6       子弹剩余发送数 */
typedef struct
{
  union
	{
		uint8_t dataBuff[6];
		__packed struct
		{
		  uint16_t bullet_remaining_num_17mm;//17mm 子弹剩余发射数目
      uint16_t bullet_remaining_num_42mm;//42mm 子弹剩余发射数目
			uint16_t coin_remaining_num;//剩余金币数量
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_bullet_remaining_t;

/* ID: 0X0209          Byte: 4       机器人RFID状态 */
/*bit 0：基地增益点 RFID 状态；
bit 1：高地增益点 RFID 状态；
bit 2：能量机关激活点 RFID 状态；
bit 3：飞坡增益点 RFID 状态；
bit 4：前哨岗增益点 RFID 状态；
bit 5：资源岛增益点 RFID 状态；
bit 6：补血点增益点 RFID 状态；
bit 7：工程机器人补血卡 RFID 状态；
RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不能
获取对应的增益效果。*/
typedef struct
{
	union
	{
		uint8_t dataBuff[4];
		__packed struct
		{
			uint32_t rfid_status;
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_rfid_status_t;

/* ID: 0x020A          Byte: 12      飞镖机器人客户端指令书 */
typedef struct
{
	union
	{
		uint8_t dataBuff[12];
		__packed struct
		{
		  uint8_t dart_launch_opening_status;//当前飞镖发射口的状态
      uint8_t dart_attack_target;//飞镖的打击目标，默认为前哨站
      uint16_t target_change_time;//切换打击目标时的比赛剩余时间，单位秒，从未切换默认为 0。
      uint8_t first_dart_speed; //检测到的第一枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0
      uint8_t second_dart_speed;//检测到的第二枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0
      uint8_t third_dart_speed; //检测到的第三枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0
      uint8_t fourth_dart_speed;//检测到的第四枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0
      uint16_t last_dart_launch_time;//最近一次的发射飞镖的比赛剩余时间，单位秒，初始值为 0。
      uint16_t operate_launch_cmd_time;	//最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0。
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_dart_client_cmd_t;


/* 

交互数据包括一个统一的数据段头结构。
数据段包含了内容 ID，发送者以及接收者的 ID 和内容数据段，
整个交互数据的包总共长最大为 128 个字节，
减去 frame_header,cmd_id 和 frame_tail 共 9 个字节以及数据段头结构的 6 个字节，
故而发送的内容数据段最大为 113。 
每个机器人交互数据与自定义控制器数据上下行合计带宽不超过 5000 Byte。上下行发送频率分别不超过
30Hz。

机器人 ID：
1，英雄(红)；
2，工程(红)；
3/4/5，步兵(红)；
6，空中(红)；
7，哨兵(红)；
9，雷达站（红）；

101，英雄(蓝)；
102，工程(蓝)；
103/104/105，步兵(蓝)；
106，空中(蓝)；
107，哨兵(蓝)；
109，雷达站（蓝）。

客户端 ID：
0x0101 为英雄操作手客户端(红)；
0x0102，工程操作手客户端((红)；
0x0103/0x0104/0x0105，步兵操作手客户端(红)；
0x0106，空中操作手客户端((红)；

0x0165，英雄操作手客户端(蓝)；
0x0166，工程操作手客户端(蓝)；
0x0167/0x0168/0x0169，步兵操作手客户端步兵(蓝)；
0x016A，空中操作手客户端(蓝)

*/

/* 
  由于存在多个内容 ID，但整个 cmd_id 上行频率最大为 10Hz，
  学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz  

	字节偏移量 	大小 	说明 			           备注 
	   0 			   2 		数据的内容 ID 	 0x0200~0x02FF 
										                 可以在以上 ID 段选取，具体 ID 含义由参赛队自定义 
	
	   2 			   2 		发送者的 ID 	   需要校验发送者的 ID 正确性， 
	
	   4 			   2 		接收者的 ID 	   需要校验接收者的 ID 正确性，
										                 例如不能发送到敌对机器人的ID 
	
	   6 			   n 		数据段 			     n 需要小于 113 


*/

/* ID: 0X0301          Byte: n       机器人间交互数据 */
typedef __packed struct
{
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

/* data */
typedef __packed struct
{
	uint8_t data[20];//数据段n小于113
}robot_interactive_data_t;

/* 
	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0x0100   0x0101  0x0102  0x0103  0x0110  0x0104
	发送频率：上限 10Hz


*/

/* 客户端删除图形 机器人间通信：0x0301 */
typedef __packed struct
{
  uint8_t operate_tpye; 
  uint8_t layer; 
}ext_client_custom_graphic_delete_t;

/* 图形数据 */
typedef __packed struct
{ 
  uint8_t graphic_name[3]; 
  uint32_t operate_tpye:3; 
  uint32_t graphic_tpye:3; 
  uint32_t layer:4; 
  uint32_t color:4; 
  uint32_t start_angle:9;
  uint32_t end_angle:9;
  uint32_t width:10; 
  uint32_t start_x:11; 
  uint32_t start_y:11; 
  uint32_t radius:10; 
  uint32_t end_x:11; 
  uint32_t end_y:11; 
}graphic_data_struct_t;

typedef __packed struct
{ 
  uint8_t graphic_name[3]; 
  uint32_t operate_tpye:3; 
  uint32_t graphic_tpye:3; 
  uint32_t layer:4; 
  uint32_t color:4; 
  uint32_t start_angle:9;
  uint32_t end_angle:9;
  uint32_t width:10; 
  uint32_t start_x:11; 
  uint32_t start_y:11; 
	int32_t data;
}ClientData_struct_t;

/* 客户端绘制一个图形 机器人间通信：0x0301 */
typedef __packed struct
{
  graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;

/* 客户端绘制二个图形 机器人间通信：0x0301 */
typedef __packed struct
{
  graphic_data_struct_t grapic_data_struct[2];
}ext_client_custom_graphic_double_t;

/* 客户端绘制五个图形 机器人间通信：0x0301 */
typedef __packed struct
{
  graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;

/* 客户端绘制七个图形 机器人间通信：0x0301 */
typedef __packed struct
{
  graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t;

/* 客户端绘制字符 机器人间通信：0x0301 */
typedef __packed struct
{
  graphic_data_struct_t grapic_data_struct;
  char data[30];
}ext_client_custom_character_t;


/*雷达站目前的想法：可能是需要让客户端小地图下发信息再进行客户端接收信息，不然可能无法实现*/
/*小地图交互*/
/* 客户端下发信息：0x0303 (15)*/
typedef struct
{
	union
	{
		uint8_t dataBuff[15];
		__packed struct
		{
			float target_position_x;
			float target_position_y;
			float target_position_z;
			uint8_t commd_keyboard;
			uint16_t target_robot_ID;
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_robot_command_t;

//typedef __packed struct
//{
//	xFrameHeader   							txFrameHeader;//帧头
//	uint16_t								CmdID;//命令码
//	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
//	ext_robot_command_t  	 			interactData;//数据段
//	uint16_t		 						FrameTail;//帧尾
//}ext_MapCommunate_t;

/*客户端接收信息:0x0305*/
//typedef struct
//{
//	union
//	{
//		uint8_t dataBuff[10];
//		__packed struct
//		{
//			uint16_t target_robot_ID;
//			float target_position_x;
//			float target_position_y;
//		};
//	}data;
//	uint8_t InfoUpdataFlag;
//}ext_client_map_command_t;
typedef __packed struct
{
	uint16_t target_robot_ID;
	float target_position_x;
	float target_position_y;
	float reserved;
}ext_client_map_command_t;

typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t								CmdID;//命令码
	//ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_map_command_t  	 			interactData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_MapCommunate_t;

/* 图传遥控客户端下发信息:0x0304*/
typedef __packed struct
{
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  int8_t left_button_down;
  int8_t right_button_down;
  uint16_t keyboard_value;
  uint16_t reserved;
}ext_robot_keycommand_t;


/*机器人交互信息：0x0301*/
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t								CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	robot_interactive_data_t  	 			interactData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_CommunatianData_t;

//帧头  命令码   数据段头结构  数据段   帧尾

/*客户端结构体*/
//上传客户端
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	graphic_data_struct_t cilentData[7];//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_SendClientData_t;

typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	graphic_data_struct_t cilentData[5];//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_ShowCrossHair_t;

//删除客户端
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_graphic_delete_t cilentData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_DeleteClientData_t;

//射速等级上传结构体
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_character_t cilentData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_ShootLevelData_t;

//电容模块上传
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_character_t cilentData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_Cap_Energyvalue_t;

typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_character_t cilentData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_Cap_Energy_t;

//抬升高度上传结构体
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_character_t cilentData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_UpliftHeightData_t;

//夹子角度结构体
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_character_t cilentData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_ClipAngeleData_t;

//车距框上传
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	graphic_data_struct_t cilentData[5];//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_CarDistance_t;

//哨兵状态指示结构体
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_character_t cilentData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_SentryStatusData_t;

//飞镖状态指示结构体
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_character_t cilentData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_DartStatusData_t;

//机器人指示信息
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_character_t cilentData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_LedMeaning_t;

typedef struct {
	union {
		uint8_t dataBuff[13];
		struct {
			float data;
		};
	}data;
	uint8_t InfoUpdateFlag;
}Judge_SendRobot_t;


/*仅测试发送  不调用*/
void Judge_sendRobot(uint16_t cmd);




/*发送给哨兵(仅限一台步兵调用)*/
void Judge_SendSentry(uint16_t);
/*发送给飞镖发射架(仅限空中机器人调用)*/
void Judge_SendDart(uint16_t);
/*发送对象（先判断红蓝方再进行发送）*/
void Send_Infantry(void);
void Send_Hero(void);
void Send_Sentry(void);
void Send_Engineering(void);
void Send_Dart(void);
void Send_Radar(void);
void determine_ID(void);
_Bool is_red_or_blue(void);


//上传给PC端
/*删除所有客户端*/
void UI_DeleteAll(void);
/*瞄准镜*/
void Show_CrossHair(void);
/*射速等级*/
void ShootLv_show(void);
void ShootLv_Updata(void);
/*超级电容模块*/
void CapEnergy_Show(void);
void Cap_EnergyUpdata(void);
/*车距框图*/
void Car_DistanceFrame(void);
/*工程抬升高度*/
void UpliftHeight_Show(void);
void UpliftHeight_Updata(void);
/*工程抬升角度*/
void ClipAngle_Show(void);
void Clip_AngleUpdata(void);
/*哨兵状态提示*/
void SentryStatus_Show(void);
void SentryStatus_Updata(void);
/*飞镖状态指示*/
void DartStatus_Show(void);
void DartStatus_Updata(void);

/**********机器人状态指示灯************/
/**黄色**/
void RobotStatus_LEDYellow(void);
void RobotStatus_YellowDelete(void);
void LEDYellow_Mean(void);
void LEDYellow_MeanDelete(void);
/*黄色总控制*/
void Yellow_ShowOn(void);
void LEDYellow_MeanUpdate(void);
void Yellow_DeleteAll(void);
/**绿色**/
void RobotStatus_LEDGreen(void);
void RobotStatus_GreenDelete(void);
void LEDGreen_Mean(void);
void LEDGreen_MeanDelete(void);
/*绿色总控制*/
void Green_ShowOn(void);
void LEDGreen_MeanUpdate(void);
void Green_DeleteAll(void);
/**橙色**/
void RobotStatus_LEDOrange(void);
void RobotStatus_OrangeDelete(void);
void LEDOrange_Mean(void);
void LEDOrange_MeanDelete(void);
/*橙色总控制*/
void Orange_ShowOn(void);
void LEDOrange_MeanUpdate(void);
void Orange_DeleteAll(void);
/**紫色**/
void RobotStatus_LEDPurple(void);
void RobotStatus_PurpleDelete(void);
void LEDPurple_Mean(void);
void LEDPurple_MeanDelete(void);
/*紫色总控制*/
void Purple_ShowOn(void);
void LEDPurple_MeanUpdate(void);
void Purple_DeleteAll(void);
/**粉色**/
void RobotStatus_LEDPink(void);
void RobotStatus_PinkDelete(void);
void LEDPink_Mean(void);
void LEDPink_MeanDelete(void);
/*粉色总控制*/
void Pink_ShowOn(void);
void LEDPink_MeanUpdate(void);
void Pink_DeleteAll(void);
/**青色**/
void RobotStatus_LEDCyan(void);
void RobotStatus_CyanDelete(void);
void LEDCyan_Mean(void);
void LEDCyan_MeanDelete(void);
/*青色总控制*/
void Cyan_ShowOn(void);
void Cyan_DeleteAll(void);
/**黑色**/
void RobotStatus_LEDBlack(void);
void RobotStatus_BlackDelete(void);
void LEDBlack_Mean(void);
void LEDBlack_MeanDelete(void);
/*黑色总控制*/
void Black_ShowOn(void);
void Black_DeleteAll(void);
/**白色**/
void RobotStatus_LEDWhite(void);
void RobotStatus_WhiteDelete(void);
void LEDWhite_Mean(void);
void LEDWhite_MeanDelete(void);
/*白色总控制*/
void White_ShowOn(void);
void White_DeleteAll(void);
/***************************************/

//小地图交互信息  客户端接收信息 0x0305
void Client_MapSend(void);


void Judge_GetMessage(uint16_t Data_Length);

extern RM_Judge_t JudgeSystem;
extern Judge_SendRobot_t Judge_SendRobot;

extern ext_game_status_t      ext_game_status;
extern ext_game_result_t     ext_game_result;
extern ext_game_robot_HP_t   ext_game_robot_HP;
extern ext_dart_status_t     ext_dart_status;
extern ext_ICRA_buff_debuff_zone_status_t    ext_ICRA_buff_debuff_zone_status;
extern ext_event_data_t ext_even_data;
extern ext_supply_projectile_action_t  ext_supply_projectile_action;
extern ext_referee_warning_t         ext_referee_warning;
extern ext_dart_remaining_time_t     ext_dart_remaining_time;
extern ext_game_robot_status_t  ext_game_robot_state;
extern ext_power_heat_data_t  ext_power_heat_data;
extern ext_game_robot_pos_t  ext_game_robot_pos;
extern ext_buff_t Buff;
extern aerial_robot_energy_t  aerial_robot_energy;
extern ext_robot_hurt_t  ext_robot_hurt;
extern ext_shoot_data_t  ext_shoot_data;
extern ext_bullet_remaining_t  ext_bullet_remaining;
extern ext_rfid_status_t        ext_rfid_status;
extern ext_dart_client_cmd_t   ext_dart_client_cmd;
extern ext_CommunatianData_t     CommuData;		//队友通信信息
extern ext_SendClientData_t      ShowData;			//客户端信息

extern uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE];


#endif //版本号
/*-----------------------------------------------------------------------*/

#endif //头文件
