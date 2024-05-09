/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file   : referee.cpp
  * @author : kainan 837736328@qq.com
  * @brief  : Code for communicating with Referee system of Robomaster 2020.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ******************************************************************************
  */
#ifndef __DEV_REFEREE_H
#define __DEV_REFEREE_H

/* Includes ------------------------------------------------------------------*/
#include  <stdint.h>
#include  <string.h>
#include <cmsis_compiler.h>
#include "BSP_UART.h"
#ifdef __cplusplus
/* Private define ------------------------------------------------------------*/

#define DRAWING_PACK	15
#define	ROBOT_COM_PACK	8
/* Private variables ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* ����ϵͳ״̬��0�����ߣ�1������ */
enum RF_status_e
{
	RF_OFFLINE = 0U,
	RF_ONLINE
};

/* ö�ٱ��������ڳ���ͨ��������� */
enum{
	HERO = 0U,
	ENGINEER,
	INFANTRY_3,
	INFANTRY_4,
	INFANTRY_5,
	AERIAL,
	SENTRY,
	RADAR = 8U
};

enum{
	Robot_Red = 0U,
	Robot_Blue
};

typedef enum
{
	RMUC = 1,
	RMUT = 2,
	ICRA = 3,
	RMUL_3V3 = 4,
	RMUL_1V1 = 5,
}Game_type_e;

/* ����������ID����ǰ�����˿ͻ���ID */
typedef	__PACKED_STRUCT{ 
	uint8_t hero;
	uint8_t engineer;
	uint8_t infantry_3;
	uint8_t infantry_4;
	uint8_t infantry_5;
	uint8_t aerial;
	uint8_t sentry;
	uint8_t drat;														//���ڷ����
	uint8_t radar;														//�״�վ
	uint8_t local;														//��������ID

	uint8_t robot_where;												//���� or �췽������
	uint16_t client;													//��Ӧ�Ŀͻ���ID
}RC_ID;

/* ����֡֡ͷ�ṹ�� */
typedef __PACKED_STRUCT{ 
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;
	uint16_t CmdID;
}FrameHeader;

/* ��������֡��ͳһ���ݶζ�ͷ */
typedef __PACKED_STRUCT { 					
	uint16_t data_cmd_id;
	uint16_t send_ID;
	uint16_t receiver_ID;
} DataHeader;

/* ----------------------------------�����������Ͷ�Ӧ��ID��--------------------------------------- */
typedef enum {
	/* ����ϵͳ����������֡ID���ͣ�ע��������CMD_ID */
	GameState_ID                    = 0x0001,		//����״̬���� 1Hz
	GameResult_ID                   = 0x0002,		//����������� ��������
	GameRobotHP_ID                  = 0x0003,		//������Ѫ������ 1Hz
	DartStatus_ID 					= 0x0004,		//���ڷ���״̬����
	ICRA_DebuffStatus_ID 			= 0x0005,		//�˹�������ս���ӳ���ͷ�״̬ 1Hz
	EventData_ID                    = 0x0101,		//�����¼����� 1Hz
	SupplyProjectileAction_ID       = 0x0102,		//����վ������ʶ ��������

	RefereeWarning_ID               = 0x0104,		//���о������� �������
	DartRemainingTime_ID            = 0x0105,		//���ڷ���ڵ���ʱ 1Hz
	GameRobotState_ID               = 0x0201,		//������״̬ 10Hz
	PowerHeatData_ID                = 0x0202,		//������������ 50Hz
	GameRobotPos_ID                 = 0x0203,		//������λ������ 10Hz
	BuffMusk_ID                     = 0x0204,		//���������� ״̬�ı����
	AerialRobotEnergy_ID            = 0x0205,		//���л��������� 10Hz
	RobotHurt_ID                    = 0x0206,		//�˺����� �˺���������
	ShootData_ID                    = 0x0207,   	//ʵʱ������� �������
	BulletRemaining_ID              = 0x0208,		//�ӵ�ʣ�෢���� �ڱ������� 1Hz
	RFID_Status_ID           		= 0x0209,		//RFID״̬ 1Hz
	ExtDartClientCmd_ID             = 0x020A,		//���ڻ����˿ͻ��˷���ָ�� 10Hz
	StudentInteractiveHeaderData_ID	= 0x0301, 		//�����˽������� 30Hz����
	CustomControllerData_ID			= 0x0302,	 	//�Զ���������������ݽӿ� 30Hz
	MiniMapInteractiveData_ID		= 0x0303,		//С��ͼ�������� ��������
	VT_Data_ID						= 0x0304,		//���̡�������ݣ�ͨ��ͼ������
	ClientMapCommand_ID				= 0x0305,		//С��ͼ������Ϣ

	/* �����˽������ݶε�ID���ͣ�ע�����������ݶ�����ID�� */	
	RobotComData_ID					= 0x0233,		//���佻������ID�ɸ��������Զ���
	CustomData_ID					= 0xD180,		//�Զ�������ID
	Drawing_Clean_ID				= 0x0100,	
	Drawing_1_ID					= 0x0101,
	Drawing_2_ID					= 0x0102,
	Drawing_5_ID					= 0x0103,
	Drawing_7_ID					= 0x0104,
	Drawing_Char_ID					= 0x0110,

} RefereeSystemID_t;

/* ----------------------------------��������֡�����ݶο��--------------------------------------- */
/**
  @brief  ����״̬���ݣ�0x0001��1Hz
*/
typedef __PACKED_STRUCT{
	uint8_t game_type : 4;	    //��������
	uint8_t game_progress : 4;  //��ǰ�����׶�
	uint16_t stage_remain_time; //��ǰ�׶�ʣ��ʱ��
	
	uint64_t SyncTimeStamp;	    //�����˽��յ���ָ��ľ�ȷ Unix ʱ��
}ext_game_status_t;

/**
   @brief ����������ݣ�0x0002���ڱ�����������
*/
typedef struct {
  uint8_t winner;
} ext_game_result_t;

/**
   @brief ������Ѫ�����ݣ�0x0003��1Hz
*/
typedef struct {
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP; 
	uint16_t red_3_robot_HP; 
	uint16_t red_4_robot_HP; 
	uint16_t red_5_robot_HP; 
	uint16_t red_7_robot_HP; 
	uint16_t red_outpost_HP;
	uint16_t red_base_HP; 
	
	uint16_t blue_1_robot_HP; 
	uint16_t blue_2_robot_HP; 
	uint16_t blue_3_robot_HP; 
	uint16_t blue_4_robot_HP; 
	uint16_t blue_5_robot_HP; 
	uint16_t blue_7_robot_HP; 
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/**
   @brief ���ڷ���״̬��0x0004�����ڷ�����ͣ�05.06ɾȥ��
*/
typedef __PACKED_STRUCT
{
 uint8_t dart_belong; 													//���䷽
 uint16_t stage_remaining_time; 										//����ʱ����ʣ��ʱ�� 
}ext_dart_status_t;

/**
   @brief �˹�������ս���ӳ���ͷ���״̬��0x0005��1Hz
*/
typedef __PACKED_STRUCT
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
	
	uint16_t red1_bullet_felt;
	uint16_t red2_bullet_felt;
	uint16_t blue1_bullet_felt;
	uint16_t blue2_bullet_felt;

	uint8_t lurk_mode;
 	uint8_t res;
}ext_ICRA_buff_debuff_zone_status_t;

/**
   @brief �����¼����ݣ�0x0101���¼��ı䷢��
*/
typedef struct {
  uint32_t event_type;
} ext_event_data_t;

/**
   @brief ����վ������ʶ��0x0102	�����ı����
*/
typedef struct {
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;  
} ext_supply_projectile_action_t;

/**
   @brief ���о�����Ϣ��0x0104	���淢������
*/
typedef struct {
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;

/**
   @brief ���ڷ���ڵ���ʱ��0x0105��1Hz
*/
typedef struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/**
   @brief ��ǰ����������״̬��0x0201��10Hz
*/
typedef __PACKED_STRUCT{ 
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_id1_17mm_cooling_rate;/* 17mm */
	uint16_t shooter_id1_17mm_cooling_limit;
	uint16_t shooter_id1_17mm_speed_limit;
	
	uint16_t shooter_id2_17mm_cooling_rate;
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;
	
	uint16_t shooter_id1_42mm_cooling_rate;
	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;
	
	uint16_t max_chassis_power;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
}ext_game_robot_status_t;   

/**
   @brief ʵʱ�����������ݣ�0x0202��50Hz
*/
typedef __PACKED_STRUCT{
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power; 
	uint16_t chassis_power_buffer; 
	uint16_t shooter_id1_17mm_heat; 
	uint16_t shooter_id2_17mm_heat; 
	uint16_t shooter_id1_42mm_heat;
}ext_power_heat_data_t;

/**
   @brief ������λ�ã�0x0203,10Hz
*/
typedef __PACKED_STRUCT{ 
  float x;
  float y;
  float z;
  float yaw;
} ext_game_robot_pos_t;

/**
   @brief ���������棺0x0204��״̬�ı����
*/
typedef struct {
  uint8_t power_rune_buff;
} ext_buff_t;

/**
   @brief ���л���������״̬��0x0205��10Hz
*/
typedef struct {
  uint8_t attack_time;
} aerial_robot_energy_t;

/**
   @brief �˺�״̬��0x0206���ܵ��˺�����
*/
typedef struct {
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/**
   @brief ʵʱ�����Ϣ��0x0207���������
*/
typedef __PACKED_STRUCT{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
}ext_shoot_data_t;

/**
   @brief �ӵ�ʣ�෢������0x0208��10Hz
*/
typedef struct {
  	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

/**
   @brief ������RFID״̬��0x0209��1Hz
*/
typedef struct
{
  uint32_t rfid_status;  /* ÿ��λ����ͬ�ص��RFID״̬ */
} ext_rfid_status_t;	/*RFID״̬����ȫ�����Ӧ������򴦷�״̬������з���ռ��ĸߵ�����㣬���ܻ�ȡ��Ӧ������Ч��*/

/**
	@brief ���ڻ����˿ͻ���ָ�����ݣ�0x020A��10Hz ֻ�Է��ڷ���
*/
typedef __PACKED_STRUCT{
  	uint8_t dart_launch_opening_status;
	uint8_t dart_attack_target;
	uint16_t target_change_time;
	uint16_t operate_launch_cmd_time;
}ext_dart_client_cmd_t;


/* ----------------------------------����ϵͳ�ͻ��˽�������--------------------------------------- */
/**
   @brief �������ݶ�ͨ�ö�ͷ�ṹ�嶨�壺�Զ������ݵ�����IDΪ��0x0301��10Hz
*/
typedef struct {
  uint16_t data_cmd_id;
  uint16_t sender_ID;     
  uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/**
   @brief ѧ�������˼�ͨ�ţ�����IDΪ0x0301������ID����0x0200~0x02FF������ѡ��10Hz
   @brief ���ﶨ�������������ݶθ�ʽ����Ȼ��������Щ���࣬�����㴦�����������˵ķ����������ݱ�
*/
typedef __PACKED_STRUCT{
	uint16_t data_cmd_id;												//���ݶζ���
	uint16_t sender_ID;     
	uint16_t receiver_ID;	
	uint8_t data[112];           										//!<������ҪС��113���ֽڣ��ٷ��ֲ�Լ����
}robot_interactive_data_t;

typedef struct {
	uint8_t data[ROBOT_COM_PACK];           							//���յ�������
} my_interactive_data_t;

/**
   @brief �Զ���������������ݣ�0x0302��30Hz��
   @brief ע�⣡�ý������ݰ����ݶ�û�ж��ף�ֱ�Ӿ������ݲ���
*/
typedef struct {
	uint8_t data[30];           										//!<������ҪС��30���ֽڣ��ٷ��ֲ�Լ����
} custom_controller_interactive_data_t;

/**
   @brief С��ͼ�·����ݣ�0x0303 ����ʱ���ͣ�
*/
typedef __PACKED_STRUCT
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
}ext_mini_map_command_t;

/**
   @brief ������Ϣ��ͨ��ͼ�����͵������ˣ�ͼ�����Ͷ˵�3pin�ӿڣ���0x0304��
*/
typedef __PACKED_STRUCT
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
}ext_VT_command_t;

/**
   @brief �״�վ�·�����֡���ݶ��壬���������������Ե�һ�˳ƿ�����0x305
*/
typedef __PACKED_STRUCT
{
	uint16_t target_robot_ID;											//�з������˵�����
	float target_position_x;
	float target_position_y;
	float toward_angle;
}ext_client_map_command_t;

/* ----------------------------------����õ��ĺ궨�塢ö����--------------------------------------- */
#define START_ID	0XA5												//����֡����ʼID���ٷ�Լ��
#define PROTOCAL_FRAME_MAX_SIZE	60 										//��������ڽ��ʱֻ��Ϊ���µ����ݰ������жϣ���������Ϊ >= ��󳤶�
#define HEADER_LEN 	4													//����֡֡ͷ���ȣ�Ϊ5����4��Ϊ�˷���ֱ�����������
#define CRC_ALL_LEN	3													//CRCУ���볤��
#define CRC_8_LEN	1									
#define CMD_LEN	2														//����ָ֡��ID

/* ����֡���״̬ */
typedef enum {
    STEP_HEADER_SOF=0,													//֡ͷSOF��ӦΪ0xA5
    STEP_LENGTH_LOW,													//���ݶγ��ȵ�8λ
    STEP_LENGTH_HIGH,													//���ݶγ��ȸ�8λ
    STEP_FRAME_SEQ,														//�����
    STEP_HEADER_CRC8,													//֡ͷCRC8У����
    STEP_DATA_CRC16														//֡ĩCRC16У����
} unPackStep_e;

/* Exported ------------------------------------------------------------------*/

/* ----------------------------------�ͻ��˻�ͼ���--------------------------------------- */
/* ����ͼ�β��������ݶ� */
typedef __PACKED_STRUCT
{
	uint8_t graphic_name[3]; 											//ͼ�����ƣ���Ϊ�ͻ����е�����
	uint32_t operate_tpye:3; 											//ͼ�β������ա����ӡ��޸ġ�ɾ����
	uint32_t graphic_tpye:3; 											//ͼ�����ͣ�����ͼ�������֣�
	uint32_t layer:4; 													//ͼ����
	uint32_t color:4; 													//��ɫ
	uint32_t start_angle:9;												//��ʼ�Ƕ�
	uint32_t end_angle:9;												//��ֹ�Ƕ�
	uint32_t width:10; 													//�߿�
	uint32_t start_x:11; 												//�������
	uint32_t start_y:11; 
	uint32_t radius:10; 												//�����С��뾶
	uint32_t end_x:11; 													//�յ�����
	uint32_t end_y:11;
}graphic_data_struct_t;


/* ɾ��ָ��ͼ����������ݶ� */
typedef __PACKED_STRUCT
{
	uint8_t operate_tpye; 
	uint8_t layer; 
} ext_client_custom_graphic_delete_t;

/* ͼ�����ò���ָ����չٷ��ֲ�P23 */
typedef enum {
	NULL_OPERATION = 0U,
    ADD_PICTURE = 1U,
    MODIFY_PICTURE = 2U,
    CLEAR_ONE_PICTURE = 3U,
} drawOperate_e;

/* ͼ���������ָ�� */
typedef enum {
    CLEAR_ONE_LAYER = 1U,
    CLEAR_ALL = 2U,
} clearOperate_e;

/* ͼ�λ������� */
typedef enum {
	LINE = 0U,
	RECTANGLE = 1U,
	CIRCLE = 2U,
	OVAL = 3U,
	ARC = 4U,
	_FLOAT = 5U,
	_INT = 6U,
	_CHAR = 7U,
} graphic_tpye_e;

/* ͼ��ɫ������ */
typedef enum
{
	RED = 0U,
	BLUE = 0U,
	YELLOW,
	GREEN,
	ORANGE,
	PURPLE,
	PINK,
	DARKGREEN,
	BLACK,
	WHITE
}colorType_e;

/* ��������֡��ͨ�����ͣ������˼�ͨ�� or �Զ���UI or С��ͼͨ�� */
typedef enum
{
	CV_OtherRobot = 0U,
	UI_Client,
	MiniMap_Client
}receive_Type_e;

typedef uint32_t (*SystemTick_Fun)(void);

/* ----------------------------------����ϵͳ����ͨ����--------------------------------------- */
class referee_Classdef
{
	public:
		/* ������ID */
		RC_ID robot_client_ID;	
		
		/* ����ϵͳ����״̬��Todo */
		RF_status_e status;

		/* ������������ */
		ext_game_status_t GameState;													//����״̬����
		ext_game_result_t GameResult;													//�����������
		ext_game_robot_HP_t GameRobotHP;												//������Ѫ������
		ext_dart_status_t DartStatus;													//����״̬����
		ext_ICRA_buff_debuff_zone_status_t ICRA_Buff;									//ICRA����buff״̬
		ext_event_data_t EventData;														//�����¼�����
		ext_supply_projectile_action_t SupplyAction;									//����վ��������
		ext_referee_warning_t RefereeWarning;											//���о�����Ϣ����
		ext_dart_remaining_time_t DartRemainTime;										//���ڷ��䵹��ʱ����
		ext_game_robot_status_t GameRobotState;											//������״̬����ǰѪ�������١����̹��ʵ�
		ext_power_heat_data_t PowerHeatData;											//�����˹��ʡ���������
		ext_game_robot_pos_t RobotPos;													//������λ������
		ext_buff_t RobotBuff;															//��������������
		aerial_robot_energy_t AerialEnergy;												//���л���������״̬����
		ext_robot_hurt_t RobotHurt;														//�������˺�״̬����
		ext_shoot_data_t ShootData;														//ʵʱ�����Ϣ����
		ext_bullet_remaining_t BulletRemaining;											//ʣ�൯��������
		ext_rfid_status_t RFID_Status;													//RFID״̬����
		ext_dart_client_cmd_t DartClientCmd;											//���ڻ����˿ͻ���ָ������
	
		/* �������� */
		my_interactive_data_t robot_rec_data[9];										//�洢�������������˷��͵���Ϣ��������״�վ											
		custom_controller_interactive_data_t custom_control_data;						//�Զ�����������ݶβ���
		ext_mini_map_command_t mini_map_data;											//С��ͼ�·���Ϣ
		ext_VT_command_t VT_command_data;												//������Ϣ��ͨ��ͼ�����Ͷ˽���
		ext_client_map_command_t radar_map_data;										//�״�վ��Ϣ

		referee_Classdef(){}															//���캯������

		
		void Init(UART_HandleTypeDef *_huart, uint32_t (*getTick_fun)(void));			//��ʼ��ѧ������		
		void unPackDataFromRF(uint8_t *data_buf, uint32_t length);						//����֡��֡���
		void CV_ToOtherRobot(uint8_t target_id, uint8_t* data1, uint8_t length);		//������֮�䳵��ͨ��
		void Radar_dataTransmit(uint8_t target_id, float position_x, float position_y, float toward_angle);	//�״�վ��Ϣ����
		
		/* �������ͼ�� */
		void clean_one_picture(uint8_t vlayer,uint8_t name[]);							//�������ͼ��
		void clean_two_picture(uint8_t vlayer,uint8_t name1[], uint8_t name2[]);		//�������ͼ��
		void clean_layer(uint8_t _layer);												//�������ͼ��
		void clean_all();																//�������
			
		/* ���ͼ���û��ӿ�ʵ�֡��Ѳ��ԡ� */
		/* �ַ������� */
		void Draw_Char(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t* char_name, uint8_t* data, uint16_t str_len, uint16_t str_size, colorType_e _color, drawOperate_e _operate_type);
		/* ͨ��UI��� */
		uint8_t UI_ruler(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t scale_step, uint16_t scale_long, uint16_t scale_short, colorType_e _color, drawOperate_e _operate_type);
		/* ͨ��UI׼�� */
		void UI_Collimator(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t line_length, colorType_e _color, drawOperate_e _operate_type);
		
		/* ��ͼ��0���Ӿ����鿪����־ */
		void Draw_Vision(uint8_t vision_flag, uint8_t state, uint8_t depth, uint16_t enable_cnt, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
		/* ��ͼ��0�������߻��� */
		void Draw_Robot_Limit(uint16_t height, uint16_t distance, uint16_t center_x, uint16_t line_width, colorType_e _color, drawOperate_e _operate_type);
		/* ��ͼ��3����̬���ֱ�־���������������ͼ���ڸ�ͼ�� */
		void Draw_Magazine(uint8_t mag_flag, uint16_t enable_cnt, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
		/* ��ͼ��5��Ħ���ֱ�־ */
		void Draw_Fric(uint8_t fric_flag, uint8_t state, uint16_t enable_cnt, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
		/* ��ͼ��2����̬С���ݱ�־ */
		void Draw_Spin(uint8_t spin_flag, uint16_t enable_cnt, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
		/* ��ͼ��2����̬����Ť����־ */
		void Draw_SpecialSwing(uint8_t specialswing_flag, uint16_t enable_cnt, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
		/* ��ͼ��1���������ݿ������� */		
		void Draw_Boost(uint8_t boost_flag,  uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
		/* ��ͼ��1�����ݵ�ѹ�ٷֱȻ��ƣ�����ϳ������ݿ�����ӡ��ʾ�ַ��� */
		void Draw_Cap_Energy(uint8_t cap_cell, uint8_t enable, uint8_t state, uint8_t enable_cnt, uint16_t center_x, uint16_t center_y);
		/* ��ͼ��4��ʣ�൯����ʾ */
		void Draw_Bullet(uint8_t enable_cnt, uint16_t center_x, uint16_t center_y);
		/* ��ͼ��6����Ѫ����ʾ */
		void Draw_LowHP(uint16_t center_x, uint16_t center_y);
		/* ��ͼ��7����̬װ�װ� */
		void Draw_Armor(uint8_t enable_cnt, uint8_t vision_state, uint16_t center_x, uint16_t center_y, uint16_t length, uint16_t width, uint16_t line_size, colorType_e _color);
		// void Draw_Armor(uint8_t enable_cnt, uint8_t x_angle, uint16_t y_angle, uint16_t length, uint16_t width, uint16_t line_size, colorType_e _color);
		
		/* ---------------------Ӣ�ۻ�������׼�ǣ��Զ���ͼ��----------------------- */
		void Hero_UI_ruler(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t* line_distance, uint16_t* line_length, colorType_e* _color, drawOperate_e _operate_type);
		
		/* ---------------------���л�����UI���ƣ��Զ���ͼ��----------------------- */
		//���л�����pitch����
		void Aerial_PitchRuler_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint16_t long_scale_length, uint16_t short_scale_length, colorType_e ruler_color, colorType_e current_color, colorType_e target_color);
		void Aerial_Pitch_Update(float pitch_angle, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e tag_color);			//���л�����pitch�ḡ��
		void Aerial_Pitch_Target(float pitch_target, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e target_color);		//���л�����pitch��Ŀ�긡��
		
		/* ---------------------���̻�����UI���ƣ��Զ���ͼ��----------------------- */
		//���̻�����̧���߶ȱ��
		void Engineer_HighthRuler_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint16_t long_scale_length, uint16_t short_scale_length, uint8_t ruler_tag, colorType_e ruler_color, colorType_e current_color);
		//���̻����˵�ǰ̧���߶ȸ���
		void Engineer_Height_Update(float height, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint8_t ruler_tag, colorType_e tag_color);
		//���̻����˵�ǰ̧���߶ȸ��꣨������ߵĸ���ϲ�Ϊһ�����ݰ����ͣ��������ݰ������ӳٴ����Ķ�̬Ч����ͬ����
		void Engineer_HeightMul_Update(float *height, uint8_t _layer, uint16_t *center_x, uint16_t *center_y, uint16_t *total_length, colorType_e tag_color);
		//���̻�����Ŀ��̧���߶ȸ���
		void Engineer_Target_Height(float target_height, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint8_t ruler_tag, colorType_e tag_color);
		
		void Mine_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t mine_tag, colorType_e _color, drawOperate_e _operate_type);		//��ʯͼ��
		void Engineer_MineRe_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size);																//���̻����˵�ǰ�洢��ʯ��UI���
		void Engineer_MineRe_Update(uint8_t mine_num, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size);											//���¹��̻����˴洢��ʯ��
		void Engineer_MineMode_Update(uint8_t mode, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size, colorType_e _color);							//���̻������Զ�/�ֶ���״̬
		void Engineer_RescueMode_Update(uint8_t mode, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size, colorType_e _color);						//���̻����˾�Ԯ/ˢ��״̬
		void Engineer_AutoMode_Update(uint8_t status, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size);											//���̻������Զ���״̬
			
		/* ----------------------�ڱ�������UI���ƣ��Զ���ͼ��----------------------- */
		void Sentry_PosRuler_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e ruler_color, colorType_e current_color);//�ڱ��������ƶ��������
		void Sentry_Pos_Update(float pos_percent, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e tag_color);				//�ڱ������˵�ǰλ�ø���
		void Sentry_Patrol_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t size, uint8_t *keys, colorType_e patrol_color);						//�ڱ�������Ѳ���������
		void Sentry_Patrol_Update(uint8_t tag, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t size, colorType_e _color);							//�ڱ�����������״ָ̬ʾ
		void Sentry_Bullet_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);											//�ڱ������˷���״̬���
		void Sentry_Bullet_Update(uint8_t tag, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);								//�ڱ������˷���״̬
		
		/* ----------------------�״�վ���Լ���ʾ��ռ��ͼ��9------------------------
		���Լ������仯ʱ����ռ�ô����Ի��ƶ�ӦUI*/
		void Armor(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t armor_tag, colorType_e _color, drawOperate_e _operate_type);			//����ͼ��
		void Sword(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t sword_tag, colorType_e _color, drawOperate_e _operate_type);			//��ͼ��
		void Outpost_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);					//ǰ��վͼ��
		void Sentry_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);						//�ڱ�ͼ��
		void Missle_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);						//����ͼ��
		void High_Land(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);						//�ߵ�ͼ��
		void Windmill_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);					//���ͼ��
		void FlyingSlope_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);				//����ͼ��
		void Radar_Strategy_Frame(uint16_t *frame_pos_x, uint16_t *frame_pos_y);																					//�״�վ����UI���
		void Radar_CStrategy_Update(uint8_t protect, uint8_t attack, uint8_t comment_startegy, uint16_t *pos_x, uint16_t *pos_y);									//�״�վȫ�ֲ���UI����
		void Radar_SStrategy_Update(uint16_t special_startegy, uint16_t *pos_x, uint16_t *pos_y);																	//�״�վר�ò���UI����

		private:
		/* �ⲿ��� */
		UART_HandleTypeDef *refereeUart;
		SystemTick_Fun Get_SystemTick;   												/*<! Pointer of function to get system tick */
		
		uint8_t repeat_cnt = 0;															//UI�����ظ�����
		
		/* �������ݰ������������128�ֽ� */
		uint8_t transmit_pack[128];
		/* UI�������ݰ� */
		uint8_t data_pack[DRAWING_PACK*7] = {0};

		/* �ͻ���ɾ��ͼ�����ݰ� */
		ext_client_custom_graphic_delete_t cleaning;
										
		/* �����ҷ�������ID */
		void Calc_Robot_ID(uint8_t local_id);

		/* ��������õ��ı��������� */
		uint8_t DataCheck(uint8_t **p);																				//�滭ʱ����ƴ�ӵĿռ�
		unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);		//��ȡ���ݰ���ͷ��CRC8У���
		uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);							//��ȡ���ݰ�������CRC16У���
		void RefereeHandle(uint8_t *data_buf); 																		//���¶�ӦID������
		void RobotInteractiveHandle(robot_interactive_data_t* RobotInteractiveData_t);								//�����˼�ͨ��

		/* ����ʱ�õ��� */
		void pack_send_robotData(uint16_t _data_cmd_id, uint16_t _receiver_ID, uint8_t* _data, uint16_t _data_len);	//0x301�������ݣ�����UI���ƺͳ���ͨ��	
		void send_toReferee(uint16_t _com_id, uint16_t length, receive_Type_e _receive_type);

		/** 
		����ͼ�εĻ��ƣ��ղ������ߡ����Ρ�Բ����Բ��Բ�������������������ַ�����ָ�����Ƶ�ͼ��
		*/
		graphic_data_struct_t* null_drawing(uint8_t _layer,uint8_t name[]);
		graphic_data_struct_t* line_drawing(uint8_t _layer,drawOperate_e _operate_type,uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy, uint16_t line_width, colorType_e vcolor,uint8_t name[]);
		graphic_data_struct_t* rectangle_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t length,uint16_t width, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
		graphic_data_struct_t* circle_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t r, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
		graphic_data_struct_t* oval_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t minor_semi_axis,uint16_t major_semi_axis, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
		graphic_data_struct_t* arc_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t minor_semi_axis,uint16_t major_semi_axis,int16_t start_angle,int16_t end_angle,uint16_t line_width,colorType_e vcolor, uint8_t name[]);
		graphic_data_struct_t* float_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint16_t width,colorType_e vcolor, float data, uint8_t name[]);
		graphic_data_struct_t* int_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint16_t width, colorType_e vcolor, int32_t data,uint8_t name[]);
		graphic_data_struct_t* character_drawing(uint8_t _layer,drawOperate_e _operate_type,uint16_t startx,uint16_t starty,uint16_t size, uint8_t width,uint8_t* data, uint16_t str_len, colorType_e vcolor, uint8_t name[]);		
};

#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
