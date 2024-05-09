///**
//  ******************************************************************************
//  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
//  * @file   : referee.cpp
//  * @author : Lingzi_Xie 1357657340@qq.com
//  * @brief  : Code for communicating with Referee system of Robomaster 2021.
//  * @date   : 2021-03-21
//  * @par Change Log��
//  *  Date           Author   Version    Notes
//  *  2019-03-01     charlie   2.0.0     Creator
//  *  2019-12-28     kainan	  3.0.0     ���ӻ滭��
//  *  2020-05-26 	kainan    4.0.0		��Ӧ20�����
//  *  2021-03-21		Lingzi    5.0.0		�������ݰ���ʽ��Ӧ21���У���������� 	
//  *  2021-05-19		Lingzi    5.4.1		�Ż���ʵ���е�һЩ��������ʹ���
//  *  2021-05-20		Lingzi    5.4.2		����21�곬���Կ�������
//  *  2021-05-22		Lingzi    5.4.5		��ӹ��̡��ڱ������˻�������UI
//  *  2021-06-06  	Lingzi 	  5.4.
//  ==============================================================================
//                          How to use this driver  
//  ==============================================================================
//	Init()��ʼ��ģ��
//	
//	����ϵͳ���ݽ������ⲿ��ȡ
//	1.ʹ��unPackDataFromRF()�������ϵͳ��������
//	2.�����Ҫ�õ�����ϵͳ�ṩ�ĸ������ݣ�������Щʲô������鿴�ֲᣩ����ȡ��Ӧ�ṹ�弴��
//	
//	�����˳���ͨ��
//	1. ���Ͷ˵���CV_ToOtherRobot()��������
//	2. ���ն���ѯ������ID��Ӧ��robot_rec_data[]����������繤�̷��͹���������Ϊrobot_rec_data[ENGINEER]

//	�����ֽ���UI
//	1.Set_DrawingLayer()����ͼ�㣬0-9
//	2.���ͼ�Σ��ṩ�˸������ֵ�һϵ��UI������������Ҫ���ö�Ӧ��������
//	3.UI�ͳ���ͨ���ڵײ���õ���vTaskDelay���Ʒ������ʣ��ʲ���Ҫ��UI�����������ٵ�����ʱ
//	4.ע�⣺�����ݸ�����ϵͳ�����ע��ȴ��ϵ��ȶ�֮��ŷ��ͣ�����ᶪ��
//	
//	ע�⣺
//	����DMA���ջ��������С�������õ���256����С������ɽ���������ݱ����ǡ�������Ҫ�ʹ��ڽ��ղ�ͬ�����������ʿ����ڲ��д�����vTaskDelay
//	Ҫ�ȴ�һ��ʱ��(�ȴ��ڡ�����ϵͳ�ȶ�)���ٷ���clean�����ݡ�UI�� 
//	�ر�ע��Ҫ�����µĲ���ϵͳ�ͻ��ˣ��ɰ��е�����
//	
//  	Ŀǰ�ο����ǲ���ϵͳ����Э�鸽¼V1.0-2021-04-30
//  	�������⣬��ο���RM2021����ϵͳ�û��ӿ�Э�鸽¼V2.1��

//  ******************************************************************************
//  * @attention:
//  * 
//  * if you had modified this file, please make sure your code does not have many 
//  * bugs, update the version NO., write dowm your name and the date, the most
//  * important is make sure the users will have clear and definite understanding 
//  * through your new brief.
//  ******************************************************************************
//  */
// #include "DEV_Referee.h"
///* Includes ------------------------------------------------------------------*/
//#include "FreeRTOS.h"
//#include "task.h"//��Ҫ�õ�taskDelay
//#include <stdio.h>

///* Private define ------------------------------------------------------------*/
//#ifndef _DEVICES_MONITOR_H_
//#define On_line   0
//#define Off_line  1
//#endif
///* Private function declarations --------------------------------------------*/
///* Private variables ---------------------------------------------------------*/

///* 8λCRCУ���룬��������֡֡ͷУ�� */
//static const unsigned char CRC8_TAB[256] = {
//    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3,
//    0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01,
//    0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe,
//    0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 0x5c,
//    0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46,
//    0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb,
//    0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5,
//    0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
//    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99,
//    0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2,
//    0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93,
//    0xcd, 0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31,
//    0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d,
//    0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d,
//    0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76,
//    0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
//    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4,
//    0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75,
//    0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9,
//    0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
//};

///* 16λCRCУ���룬��������֡��֡У�� */
//static const uint16_t wCRC_Table[256] = {
//    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e,
//    0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64,
//    0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e,
//    0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
//    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e,
//    0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44,
//    0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e,
//    0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
//    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e,
//    0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324,
//    0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e,
//    0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
//    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e,
//    0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704,
//    0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e,
//    0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
//    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1,
//    0x0f78
//};
///* Private type --------------------------------------------------------------*/

///* Private function declarations ---------------------------------------------*/

///* Private function prototypes -----------------------------------------------*/

///**
// * @brief 
// * @note Template of data len limitation, use in SendDrawData() and character_drawing()
// * @param 
// * @retval 
// */
//template<typename Type>
//Type _referee_Constrain(Type input,Type min,Type max){
//  if (input <= min)
//    return min;
//  else if(input >= max)
//    return max;
//  else return input;
//}

///**
//  * @brief   Set referee's communicate channel at one time 
//  * @param   *_huart, handle of HAL_uart
//  * @param   *getTick_fun, handle of get microtick fun
//  * @retval  
//  */
//void referee_Classdef::Init(UART_HandleTypeDef *_huart, uint32_t (*getTick_fun)(void))
//{
//	refereeUart = _huart;
//	
//	if(getTick_fun != NULL)
//    Get_SystemTick = getTick_fun;

//}


///**
//  * @brief   CRC8 data check.
//  * @param   *pchMessage:Data to be processed
//              dwLength:Length of check data
//              ucCRC8:Data after processing
//  * @retval  	Gets the CRC8 	
//  */
//unsigned char referee_Classdef::Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
//{
//    unsigned char ucIndex;
//    while (dwLength--) {
//        ucIndex = ucCRC8^(*pchMessage++);										
//        ucCRC8 = CRC8_TAB[ucIndex];
//    }
//    return(ucCRC8);
//}
///**
//  * @brief   CRC16 data check.
//  * @param   *pchMessage:Data to be processed
//             dwLength:Length of check data
//             ucCRC8:Data after processing
//  * @retval  Gets the CRC16 checksum
//  */
//uint16_t referee_Classdef::Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
//{
//    uint8_t chData;
//    if (pchMessage == NULL) {
//        return 0xFFFF;
//    }
//    while(dwLength--) {
//        chData = *pchMessage++;
//        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
//    }
//    return wCRC;
//}


///* ----------------------------------�Խ�������֡���н��--------------------------------------- */

///**
//  * @brief   ѧ����������֡���
//  * @note    ��ѧ����������֡���н�������������ݰ������ͽ���Ϣ�洢����Ӧ�Ľṹ����
//  * @param   *data_buff: ָ�򴮿ڽ��յ�������֡
//			 length�����յ�������֡����
//  * @retval  None
//  */
//void referee_Classdef::unPackDataFromRF(uint8_t *data_buf, uint32_t length)
//{
//	static uint8_t RFdataBuff[256];														//��data_bufָ�������֡���Ƶ���������

//	static int32_t index,buff_read_index;												//�������洢��������������������								
//	static short CRC16_Function,CRC16_Referee;											//��¼��֡��CRC16����������֡ĩ��CRC16��ֵ
//	static uint8_t byte;
//	static int32_t read_len;															//��¼����֡�����ֽ���
//	static uint16_t data_len;															//��¼����֡�����ݶ��ֽ���
//	static uint8_t unpack_step;															//��¼��ǰ����֡�Ľ��״̬
//	static uint8_t protocol_packet[PROTOCAL_FRAME_MAX_SIZE];							//�������洢��
//	
//    /*��ʼ����ȡ״̬*/
//    buff_read_index = 0;
//    memcpy(RFdataBuff,data_buf,length);
//	
//    /*������֡֡ͷ��ʼ��ȡ */
//    read_len=length;
//	
//    while (read_len--) 
//	{
//		byte = RFdataBuff[buff_read_index++];											//���뻺������ǰ����
//		switch(unpack_step) {
//		case STEP_HEADER_SOF: {
//			if(byte == START_ID) {
//				unpack_step = STEP_LENGTH_LOW;											//������֡֡ͷ��SOFд�����洢��
//				protocol_packet[index++] = byte;										
//			} else {
//				index = 0;
//			}
//		}
//		break;

//        case STEP_LENGTH_LOW: {
//            data_len = byte;
//            protocol_packet[index++] = byte;											//������֡֡ͷ��len�Ͱ�λд�����洢��										
//            unpack_step = STEP_LENGTH_HIGH;
//        }
//        break;

//        case STEP_LENGTH_HIGH: {
//            data_len |= (byte << 8);
//            protocol_packet[index++] = byte;
//            if(data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_ALL_LEN)) {	
//                unpack_step = STEP_FRAME_SEQ;											//������֡֡ͷ��len�߰�λд�����洢��
//            } else {
//                unpack_step = STEP_HEADER_SOF;											//������֡��������������ո�֡
//                index = 0;
//            }
//        }
//        break;

//        case STEP_FRAME_SEQ: {	
//            protocol_packet[index++] = byte;											//������֡֡ͷ�İ�����SEQд�����洢��						
//            unpack_step = STEP_HEADER_CRC8;
//        }
//        break;

//        case STEP_HEADER_CRC8: {														//֡ͷCRC8���У��
//            protocol_packet[index++] = byte;											//��CRC8У����д����������

//            if (index == HEADER_LEN+1) {
//                if ( Get_CRC8_Check_Sum(protocol_packet, HEADER_LEN,0xff)== protocol_packet[HEADER_LEN]) {
//                    unpack_step = STEP_DATA_CRC16;										//֡ͷУ��ɹ����������֡У��
//                } else {
//                    unpack_step = STEP_HEADER_SOF;
//                    index = 0;
//                }
//            }
//        }
//        break;

//        case STEP_DATA_CRC16: {
//            if (index < (HEADER_LEN + CMD_LEN + data_len + CRC_ALL_LEN)) {				//δ��������֡�ֽ�
//                protocol_packet[index++] = byte;
//            }
//            if (index >= (HEADER_LEN + CMD_LEN + data_len + CRC_ALL_LEN)) {				//��������֡����������ʼCRC16У��

//                CRC16_Function=Get_CRC16_Check_Sum(protocol_packet, HEADER_LEN + CMD_LEN + data_len +CRC_8_LEN,0xffff);
//                CRC16_Referee=* (__packed short *)(&protocol_packet[index-2]);			//ȡ������֡֡ĩ��CRC16У���
//                if ( CRC16_Function==CRC16_Referee) {									//�Ա�CRCУ����
//                    RefereeHandle(protocol_packet);										//У����ɣ�������������cmd_idת�浽��Ķ�Ӧ��Ա�ṹ����
//                }
//                unpack_step = STEP_HEADER_SOF;
//                index = 0;
//            }
//        }
//        break;

//        default: {																		//���������ֱ�����ý��״̬������
//            unpack_step = STEP_HEADER_SOF;
//            index = 0;
//        }
//        break;
//        }
//    }
//}

///**
//  * @brief  Receive and handle referee system data
//  * @param  *data_buf:data which is unpacked successfully
//  * @retval void
//  */
//void referee_Classdef::RefereeHandle(uint8_t *data_buf)
//{
//    switch(((FrameHeader *)data_buf)->CmdID) {											//ȡ����������cmd_id

//			case GameState_ID: 
//				GameState = *(ext_game_status_t*)(&data_buf[7]);						//ȡ���������ݶεĶ���ʼ�ֽڵ�ַ��ת��Ϊ��Ӧ�ṹ�����͵�ָ�룬���Ը�ָ��ȡֵ����������
//			break;
//			
//			case GameResult_ID: 
//					GameResult = *(ext_game_result_t*)(&data_buf[7]);
//			break;
//			
//			case GameRobotHP_ID: 
//					GameRobotHP = *(ext_game_robot_HP_t*)(&data_buf[7]);
//			break;	
//			
//			case DartStatus_ID: 
//					DartStatus = *(ext_dart_status_t*)(&data_buf[7]);
//			break;

//			case ICRA_DebuffStatus_ID:
//					ICRA_Buff = *(ext_ICRA_buff_debuff_zone_status_t*)(&data_buf[7]);
//			break;			
//			
//			case EventData_ID: 
//					EventData = *(ext_event_data_t*)(&data_buf[7]);
//			break;			
//			
//			case SupplyProjectileAction_ID: 
//					SupplyAction = *(ext_supply_projectile_action_t*)(&data_buf[7]);
//			break;			
//			
//			case RefereeWarning_ID: 
//					RefereeWarning = *(ext_referee_warning_t*)(&data_buf[7]);
//			break;			
//			
//			case DartRemainingTime_ID: 
//					DartRemainTime = *(ext_dart_remaining_time_t*)(&data_buf[7]);
//			break;			
//			
//			case GameRobotState_ID: 
//					GameRobotState = *(ext_game_robot_status_t*)(&data_buf[7]);
//					Calc_Robot_ID(GameRobotState.robot_id);
//			break;			
//						
//			case PowerHeatData_ID: 
//					PowerHeatData = *(ext_power_heat_data_t*)(&data_buf[7]);
//			break;			
//			
//			case GameRobotPos_ID: 
//					RobotPos = *(ext_game_robot_pos_t*)(&data_buf[7]);
//			break;			
//			
//			case BuffMusk_ID: 
//					RobotBuff = *(ext_buff_t*)(&data_buf[7]);
//			break;			
//			
//			case AerialRobotEnergy_ID: 
//					AerialEnergy = *(aerial_robot_energy_t*)(&data_buf[7]);
//			break;			
//			
//			case RobotHurt_ID: 
//					RobotHurt = *(ext_robot_hurt_t*)(&data_buf[7]);
//			break;			
//			
//			case ShootData_ID: 
//					ShootData = *(ext_shoot_data_t*)(&data_buf[7]);
//			break;			
//						
//			case BulletRemaining_ID: 
//					BulletRemaining = *(ext_bullet_remaining_t*)(&data_buf[7]);
//			break;						
//			
//			case RFID_Status_ID: 
//					RFID_Status = *(ext_rfid_status_t*)(&data_buf[7]);
//			break;			
//						
//			case ExtDartClientCmd_ID:				
//					DartClientCmd = *(ext_dart_client_cmd_t*)(&data_buf[7]);
//			break;
//			
//			case StudentInteractiveHeaderData_ID:																//��Ϊ���ݽ���������֡���򽻸�������Ա�������д���							
//					RobotInteractiveHandle((robot_interactive_data_t*)(&data_buf[7]));							//robot_interactive_data_t���������ݶΣ�
//			break;

//			case CustomControllerData_ID:
//					custom_control_data = *(custom_controller_interactive_data_t*)(&data_buf[7]);
//			break;

//			case MiniMapInteractiveData_ID:
//					mini_map_data = *(ext_mini_map_command_t*)(&data_buf[7]);
//			break;
//			default:
//					break;
//    }
//}

///**
//  * @brief  ������ͨ�����ݰ� 
//  * @param  RobotInteractiveData_t:����ͨ�����ݰ������ݶ��׵�ַ
//  * @retval None
//  */
//void referee_Classdef::RobotInteractiveHandle(robot_interactive_data_t* RobotInteractiveData_t)
//{
//	if(GameRobotState.robot_id == RobotInteractiveData_t->receiver_ID && GameRobotState.robot_id != 0) {		//���ȷʵ�Ƿ����������˵Ľ�������֡������ܸ�֡
//		if(RobotInteractiveData_t->data_cmd_id == RobotComData_ID) {											//���ݷ����ߵ�ID�Ź������ݶβ��洢����Ӧ����
//			if(RobotInteractiveData_t->sender_ID > 100)															//�������ݽ���
//				memcpy(&robot_rec_data[RobotInteractiveData_t->sender_ID - 101], RobotInteractiveData_t->data, ROBOT_COM_PACK);
//			else																								//�췽���ݽ���
//				memcpy(&robot_rec_data[RobotInteractiveData_t->sender_ID - 1], RobotInteractiveData_t->data, ROBOT_COM_PACK);
//		}
//	}
//}

///**
//  * @brief  Calculate robot ID 
//  * @param  local_id: ����ϵͳ���͵ı�������ID
//  * @retval None
//  */
//void referee_Classdef::Calc_Robot_ID(uint8_t local_id)
//{
//	uint8_t *id_ptr = (uint8_t*)&robot_client_ID;
//	uint8_t i = 1;

//	if(local_id !=0 )																	
//	{
//		if(local_id < 10)																//���㵱ǰ�����˵�ID�ţ��췽��
//		{
//			for(i = 1;i < 10;i++)
//				(*id_ptr++) = i;

//			robot_client_ID.robot_where = Robot_Red;
//			robot_client_ID.local = local_id;
//			robot_client_ID.client = 0x100 + local_id;									
//		}
//		else																			//���㵱ǰ�����˵�ID�ţ�������
//		{
//			for(i = 1;i < 10;i++)
//				(*id_ptr++) = i + 100;

//			robot_client_ID.robot_where = Robot_Blue;	
//			robot_client_ID.local = local_id;	
//			robot_client_ID.client = 0x0100 + local_id;							
//		}
//	}
//}

///* ----------------------------------�ײ�����֡���ͼ����ʿ���--------------------------------------- */

///**
// * @brief ���������֮��Ľ������ݣ��Լ�UI�����·����ײ㷢��
// * @param _data_cmd_id: ���ݶ�ID
// * 		  _receiver_ID: ���շ�ID�������ǻ����˶�Ӧ�ͻ��ˡ����߼�������������
// * 		  _data: ���ݶζ���ָ��
// * 		  _data_len: ���ݶγ���
// * @retval None
// */
//void referee_Classdef::pack_send_robotData(uint16_t _data_cmd_id, uint16_t _receiver_ID, uint8_t* _data, uint16_t _data_len)
//{
//	DataHeader data_header;																//�������ݶζ��ײ�����										
//	data_header.data_cmd_id = _data_cmd_id;
//	data_header.send_ID = robot_client_ID.local;										//���÷�����ID
//	data_header.receiver_ID = _receiver_ID;
//	
//	uint8_t header_len = sizeof(data_header);											
//	memcpy((void*)(transmit_pack + 7), &data_header, header_len);						//������֡�����ݶν��з�װ����װ���ף�
//	memcpy((void*)(transmit_pack + 7 + header_len), _data, _data_len);					//������֡�����ݶν��з�װ����װ���ݣ�
//	
//	if(data_header.receiver_ID == robot_client_ID.client)								//��UI���ƣ������÷��͸�����Ĳ���ϵͳ�ͻ���
//		send_toReferee(StudentInteractiveHeaderData_ID, header_len+_data_len, UI_Client);
//	else																				//���������͸�����������
//		send_toReferee(StudentInteractiveHeaderData_ID, header_len+_data_len, CV_OtherRobot);
//}

///**
// * @brief �ײ㷢�ͺ��������������ݰ����Լ��Է�������������
// * @param _cmd_id��
// * @param _data��
// * @param data_len��
// * @param _receive_type���жϳ���ͨ�� or �״�վ or UI�����������費��Ҫ����Σ��ͻ������ݾ���������������Ҫ����Σ����Լ�����Ƶ��
// */
//void referee_Classdef::send_toReferee(uint16_t _cmd_id, uint16_t _data_len, receive_Type_e _receive_type)
//{	
//	static uint8_t seq = 0;
//	static uint32_t next_send_time = 0;																						//���ڿ��Ʒ�������
//	FrameHeader send_frame_header;																							//��������֡֡ͷ����	

//	send_frame_header.SOF = START_ID;
//	send_frame_header.DataLength = _data_len;
//	send_frame_header.Seq = seq++;
//	send_frame_header.CRC8 = Get_CRC8_Check_Sum((uint8_t*)&send_frame_header,4,0xff);
//	send_frame_header.CmdID = _cmd_id;
//	
//	uint8_t header_len = sizeof(send_frame_header);
//	
//	memcpy((void*)transmit_pack, &send_frame_header, header_len);															//��֡ͷװ�뻺����																		//�����ݶ�ת�뻺����	
//	
//	*(__packed short *)(&transmit_pack[header_len + _data_len]) = Get_CRC16_Check_Sum(transmit_pack,header_len + _data_len,0xffff);	//��ȡ��֡��CRC16У���룬��ֱ�����뻺����
//	
//	uint8_t send_cnt = 3;																									//�����������δ���ʱ��
//	uint16_t total_len = header_len + _data_len + 2;																		//header_len + _data_len + CRC16_len
//	
//	while(send_cnt != 0)
//	{
//		uint32_t now_time = Get_SystemTick() / 1000;																		//��ȡ��ǰʱ�����ת��Ϊms
//		if(now_time > next_send_time)															
//		{
//			while(HAL_UART_Transmit_DMA(refereeUart,transmit_pack,total_len) != HAL_OK);									//��ʱ�ѵ�������
////			while(HAL_UART_Transmit(refereeUart, transmit_pack, total_len, 0xff) != HAL_OK);
//			next_send_time = now_time + float(total_len) / 5000 * 1000;														//������һ���������ʱ�䣬2021�����ٷ�Լ����������Ϊ5000bps
//			
//			switch (_receive_type)
//			{
//			case CV_OtherRobot:								//����ͨ�ţ���һ��
//				send_cnt = 0;
//				vTaskDelay(35);								//ÿ����һ������������ʱһ��ʱ��
//				break;
//			case UI_Client:									//UI���ƣ�������
//				// send_cnt--;
//				send_cnt = 0;	//--- ��Ϊ��һ��
//				vTaskDelay(15);
//				break;
//			case MiniMap_Client:							//С��ͼ��������һ��
//				send_cnt = 0;
//				vTaskDelay(100);
//			default:
//				break;
//			}
//		}	
//	}
//	
//}

///* ----------------------------------����ͨ�ţ��״�վͨ���Լ�UI����--------------------------------------- */

///**
//  * @brief  ���������˼��ͨ��
//  * @note   ע��ͨ������ӦС��113�ֽ�
//  * @retval void
//  */
//void referee_Classdef::CV_ToOtherRobot(uint8_t target_id, uint8_t* _data, uint8_t length)
//{
//	pack_send_robotData(RobotComData_ID, target_id, (uint8_t*)_data, length);
//}

///**
// * @brief �״�վ������Ϣ���ͣ�0x305
// * @brief �״�վ���ͼ������굽����ϵͳ���������в����ֿͻ��˵�С��ͼ������ʾ
// * @param target_id: �з�������ID
// * 		  position_x, position_y: �з�����������
// * 		  toward_angle: �з������˷����
// */
//void referee_Classdef::Radar_dataTransmit(uint8_t target_id, float position_x, float position_y, float toward_angle)
//{
//	/* ���õз�Ŀ������˵�ID */
//	if(robot_client_ID.robot_where)								//��Ϊ����
//		radar_map_data.target_robot_ID = target_id - 100;
//	else														//��Ϊ�췽
//		radar_map_data.target_robot_ID = target_id + 100;

//	radar_map_data.target_position_x = position_x;
//	radar_map_data.target_position_y = position_y;
//	radar_map_data.toward_angle = toward_angle;

//	uint8_t radar_data_len = sizeof(radar_map_data);

//	memcpy((void*)(transmit_pack + 9), &radar_map_data, radar_data_len);				//������֡�����ݶν��з�װ����װ���ݣ�
//	send_toReferee(ClientMapCommand_ID, radar_data_len, MiniMap_Client);
//}

///**
// * @brief �ղ������ݰ�
// * @param 
// * @retval 
// */
//graphic_data_struct_t* referee_Classdef::null_drawing(uint8_t _layer, uint8_t name[])
//{
//	static graphic_data_struct_t drawing;
//	memcpy(drawing.graphic_name,name,3);	

//	drawing.operate_tpye = NULL_OPERATION;
//	
//	return &drawing;
//}

///**
// * @brief ֱ�߻������ݰ�
// * @param line_width �߿�
// * @retval 
// */
//graphic_data_struct_t* referee_Classdef::line_drawing(uint8_t _layer,drawOperate_e _operate_type,uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy, uint16_t line_width, colorType_e vcolor,uint8_t name[])
//{
//	static graphic_data_struct_t drawing;
//	
//	memcpy(drawing.graphic_name,name,3);																			//ͼ�����ƣ�3λ
//	drawing.layer = _layer;
//	drawing.operate_tpye = _operate_type;
//	drawing.graphic_tpye = LINE;
//	drawing.width = line_width;
//	drawing.color = vcolor;
//	drawing.start_x=startx;
//	drawing.start_y=starty;
//	drawing.end_x=endx;
//	drawing.end_y=endy;
//	
//	return &drawing;
//}

///**
// * @brief ���λ���
// * @note 
// * @param line_width �߿�
// * @retval 
// */
//graphic_data_struct_t* referee_Classdef::rectangle_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t length,uint16_t width, uint16_t line_width, colorType_e vcolor, uint8_t name[])
//{
//	static graphic_data_struct_t drawing;
//	
//	memcpy(drawing.graphic_name, name, 3);
//	drawing.layer = _layer;	
//	drawing.operate_tpye = _operate_type;
//	drawing.graphic_tpye = RECTANGLE;
//	drawing.width = line_width;
//	drawing.color = vcolor;
//	drawing.start_x = startx;
//	drawing.start_y = starty;
//	drawing.end_x = startx+length;
//	drawing.end_y = starty+width;
//	
//	return &drawing;
//}

///**
// * @brief ԲȦ����
// * @note 
// * @param 
// * @retval 
// */
//graphic_data_struct_t* referee_Classdef::circle_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t r, uint16_t line_width, colorType_e vcolor, uint8_t name[])
//{
//	static graphic_data_struct_t drawing;
//	
//	memcpy(drawing.graphic_name, name, 3);	
//	drawing.layer = _layer;	
//	drawing.operate_tpye = _operate_type;
//	drawing.graphic_tpye = CIRCLE;
//	drawing.width = line_width;
//	drawing.color=vcolor;
//	drawing.start_x=centrex;
//	drawing.start_y=centrey;
//	drawing.radius = r;
//	
//	return &drawing;
//}

///**
// * @brief ��Բ����
// * @note 
// * @param minor_semi_axis x�᳤
// * @param major_semi_axis y�᳤
// * @retval 
// */
//graphic_data_struct_t* referee_Classdef::oval_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t minor_semi_axis,uint16_t major_semi_axis, uint16_t line_width, colorType_e vcolor, uint8_t name[])
//{
//	static graphic_data_struct_t drawing;
//	
//	memcpy(drawing.graphic_name, name, 3);	
//	drawing.layer = _layer;
//	drawing.operate_tpye = _operate_type;	
//	drawing.graphic_tpye=OVAL ;
//	drawing.width=line_width;
//	drawing.color=vcolor;
//	drawing.start_x=centrex;
//	drawing.start_y=centrey;
//	drawing.end_x=major_semi_axis;
//	drawing.end_y=minor_semi_axis;	
//	
//	return &drawing;
//}

///**
// * @brief ��Բ������
// * @note 
// * @param 
// * @retval 
// */
//graphic_data_struct_t* referee_Classdef::arc_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t minor_semi_axis,uint16_t major_semi_axis,int16_t start_angle,int16_t end_angle, uint16_t line_width, colorType_e vcolor, uint8_t name[])
//{
//	static graphic_data_struct_t drawing;
//	
//	memcpy(drawing.graphic_name, name, 3);	
//	drawing.layer = _layer;
//	drawing.operate_tpye = _operate_type;	
//	drawing.graphic_tpye=ARC ;
//	drawing.width=line_width;
//	drawing.color=vcolor;
//	drawing.start_x=centrex;
//	drawing.start_y=centrey;
//	drawing.end_x=minor_semi_axis;
//	drawing.end_y=major_semi_axis;
//	drawing.start_angle=start_angle;
//	drawing.end_angle=end_angle;
//	
//	return &drawing;
//}

///**
// * @brief ���������ơ��°�ͻ�����ʱ�����ã����ȹٷ����¡�
// * @note 
// * @param 
// * @retval 
// */
//graphic_data_struct_t* referee_Classdef::float_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t startx,uint16_t starty, uint16_t size, uint16_t width, colorType_e vcolor, float data, uint8_t name[])
//{
//	static graphic_data_struct_t drawing;
//	static uint8_t* drawing_ptr = (uint8_t* )&drawing;
//	
//	memcpy(drawing.graphic_name, name, 3);	
//	drawing.layer = _layer;
//	drawing.operate_tpye = _operate_type;	
//	drawing.graphic_tpye=5U ;
//	drawing.start_angle = size;																						//���������������С����Ҫ����
//	drawing.end_angle = 2;																							//������������ЧС��λ������Ҫ����
//	drawing.width=width;
//	drawing.color=vcolor;
//	drawing.start_x=startx;
//	drawing.start_y=starty;
//	
//	memcpy((void*)(drawing_ptr + 11), (uint8_t*)&data, 4);															//��32λ��������ֵ��drawing�ṹ����
//	return &drawing;
//}

///**
// * @brief �������ơ��°�ͻ�����ʱ�����ã����ȹٷ����¡�
// * @note 
// * @param 
// * @retval 
// */
//graphic_data_struct_t* referee_Classdef::int_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint16_t width, colorType_e vcolor, int32_t data,uint8_t name[])
//{
//	static graphic_data_struct_t drawing;
//	static uint8_t* drawing_ptr = (uint8_t* )&drawing;
//	
//	memcpy(drawing.graphic_name, name, 3);	
//	drawing.layer = _layer;
//	drawing.operate_tpye = _operate_type;	
//	drawing.graphic_tpye=_INT ;
//	drawing.start_angle = size;																						//���������������С����Ҫ����
//	drawing.width = width;
//	drawing.color=vcolor;
//	drawing.start_x=startx;
//	drawing.start_y=starty;
//	
//	memcpy((void*)(drawing_ptr + 11), (uint8_t*)&data, 4);															//��32λ������ֵ��drawing�ṹ����
//	return &drawing;
//}

///**
// * @brief �ַ�������
// * @note 
// * @param 
// * @retval 
// */
//graphic_data_struct_t* referee_Classdef::character_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint8_t width,uint8_t* data, uint16_t str_len, colorType_e vcolor, uint8_t name[])
//{
//	static graphic_data_struct_t drawing;
//	static uint8_t char_length;
//	
//	char_length = _referee_Constrain((uint8_t)str_len, (uint8_t)0, (uint8_t)30);											//���ַ�������Լ����30��֮��
//	
//	memcpy(drawing.graphic_name, name, 3);
//	drawing.layer = _layer;	
//	drawing.operate_tpye = _operate_type;	
//	drawing.graphic_tpye=_CHAR ;
//	drawing.width=width;
//	drawing.color=vcolor;
//	drawing.start_x=startx;
//	drawing.start_y=starty;
//	drawing.radius=0;
//	drawing.start_angle = size;																						//�����ַ���С���Ƽ������С���߿�ı���Ϊ10:1
//	drawing.end_angle=char_length;																					//�����ַ�������
//	
//	return &drawing;
//}

///**
// * @brief ���ĳ��ͼ���µ�һ��ͼƬ
// * @note Referee.clean_one_picture(2, test);
// * @param 
// * @retval 
// */
//void referee_Classdef::clean_one_picture(uint8_t vlayer,uint8_t name[])												//ɾ��ָ��ͼ���µ�ָ��ͼ��
//{
//	static graphic_data_struct_t drawing;
//	memcpy(drawing.graphic_name, name, 3);		
//	drawing.layer = vlayer ;
//	drawing.operate_tpye=CLEAR_ONE_PICTURE;
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)&drawing, sizeof(drawing));
//}

///**
// * @brief ���ĳ��ͼ���µ�����ͼƬ
// * @note Referee.clean_two_picture(2, test1, test2);
// * @param 
// * @retval 
// */
//void referee_Classdef::clean_two_picture(uint8_t vlayer,uint8_t name1[], uint8_t name2[])												//ɾ��ָ��ͼ���µ�ָ��ͼ��
//{
//	static graphic_data_struct_t drawing[2];
//	memcpy(drawing[0].graphic_name, name1, 3);		
//	drawing[0].layer = vlayer ;
//	drawing[0].operate_tpye=CLEAR_ONE_PICTURE;

//	memcpy(drawing[1].graphic_name, name2, 3);		
//	drawing[1].layer = vlayer ;
//	drawing[1].operate_tpye=CLEAR_ONE_PICTURE;

//	pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)drawing, sizeof(drawing));
//}

///**
// * @brief ���ĳһ��ͼ��
// * @note Referee.clean_layer(2);
// * @param 
// * @retval 
// */
//void referee_Classdef::clean_layer(uint8_t _layer)																	//ɾ��ָ��ͼ��
//{	
//	cleaning.layer = _layer;
//	cleaning.operate_tpye = CLEAR_ONE_LAYER;
//	
//	pack_send_robotData(Drawing_Clean_ID, robot_client_ID.client, (uint8_t*)&cleaning, sizeof(cleaning));
//}
///**
// * @brief �������UI����ͼ��
// * @note Referee.clean_all();
// * @param 
// * @retval 
// */
//void referee_Classdef::clean_all()																					//��������Զ���ͼ��
//{		
//	cleaning.operate_tpye = CLEAR_ALL;	
//	pack_send_robotData(Drawing_Clean_ID, robot_client_ID.client, (uint8_t*)&cleaning, sizeof(cleaning));
//}

///* ----------------------------------����ϵͳ�ͻ���UI�����û��ӿ�--------------------------------------- */

///**
// * @brief ���Զ���ͼ�㡿�����ַ���
// * @brief �ַ������ó���30���ַ�
// * @note ���Ժ��ʵ��������ͨ������referee.Draw_Char(8, 1300, 700, cap_name1, cap_str, sizeof(cap_str), WHITE, ADD_PICTURE);
// */
//void referee_Classdef::Draw_Char(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t* char_name, uint8_t* data, uint16_t str_len, uint16_t str_size, colorType_e _color, drawOperate_e _operate_type)
//{
//	for(uint8_t i = DRAWING_PACK;i < DRAWING_PACK + 30;i++)
//		data_pack[i] = 0;
//	
//	memcpy(data_pack, (uint8_t*)character_drawing(_layer, _operate_type, start_x,start_y,str_size, str_size / 10, data, str_len, _color, char_name), DRAWING_PACK);
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t* )data, str_len);
//	pack_send_robotData(Drawing_Char_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK + 30);
//}

///**
// * @brief ���Զ���ͼ�㡿UI��߻��ƣ�һ���Ի���һ�����
// * @note  ׼��Բ�뾶Ϊ24 
// * @param _sys_time, sacle_num�������̶���(<9),ruler_tag�ڼ������, startpoint(������Ͻ����), step(���),scale_long(���̶��ߵĳ���),scale_short
// * @note ���Ժ��ʵ��������ͨ������referee.UI_ruler(4,961,538,30,70,40,BLUE,ADD_PICTURE);
// */
//uint8_t referee_Classdef::UI_ruler(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t scale_step, uint16_t scale_long, uint16_t scale_short, colorType_e _color, drawOperate_e _operate_type)
//{	
//	static uint8_t ruler_name[] = "ru0";				
//	static uint8_t if_short = 0;
//	
//	ruler_name[2] = '0';
//	memcpy(data_pack, (uint8_t*)circle_drawing(_layer,_operate_type,start_x,start_y,24, 3, _color, ruler_name), DRAWING_PACK);
//	ruler_name[2] = '1';
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer,_operate_type,start_x,start_y,start_x,start_y - 200, 3, _color, ruler_name), DRAWING_PACK);	
//	ruler_name[2] = '2';
//	memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer,_operate_type,start_x - 100,start_y - 10,start_x + 100,start_y - 10, 3, _color, ruler_name), DRAWING_PACK);	

//	for(uint8_t i = 0;i < 4;i++)
//	{
//		if(if_short == 0)
//		{
//			if_short = 1;
//			ruler_name[2] = '3'+i;
//			memcpy(&data_pack[DRAWING_PACK*(i + 3)], (uint8_t*)line_drawing(_layer,_operate_type,start_x - scale_short/2,start_y - 24 - scale_step*(i + 1),start_x + scale_short/2,start_y - 24 - scale_step*(i + 1), 3, _color, ruler_name), DRAWING_PACK);	
//		}
//		else
//		{
//			if_short = 0;
//			ruler_name[2] = '3'+i;
//			memcpy(&data_pack[DRAWING_PACK*(i + 3)], (uint8_t*)line_drawing(_layer,_operate_type,start_x - scale_long/2,start_y - 24 - scale_step*(i + 1),start_x + scale_long/2,start_y - 24 - scale_step*(i + 1), 3, _color, ruler_name), DRAWING_PACK);	
//		}	
//	}
//	
//	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);

//	return 0;
//}

///**
// * @brief ���Զ���ͼ�㡿UI׼�ǻ��ƣ�һ���Ի���һ��׼�ǣ���׼���е�Ϊ���⼤�����ĵ�
// * @note ���Ժ��ʵ��������ͨ������referee.UI_Collimator(5, 961, 538, 25, YELLOW, ADD_PICTURE);
// * @param 
// */
//void referee_Classdef::UI_Collimator(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t line_length, colorType_e _color, drawOperate_e _operate_type)
//{
//	static uint8_t point_name[] = "poi";
//	static uint8_t line_name[] = "cl0";

//	//���ĵ�
//	line_name[2] = '0';
//	memcpy(data_pack, (uint8_t*)circle_drawing(_layer,_operate_type,start_x,start_y,1, 3, _color, point_name), DRAWING_PACK);
//	
//	//׼���ķ������
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer,_operate_type,start_x - 5,start_y, start_x - 5 - line_length, start_y, 3,_color, line_name), DRAWING_PACK);
//	line_name[2] = '1';
//	memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer,_operate_type,start_x + 5,start_y, start_x + 5 + line_length, start_y, 3,_color, line_name), DRAWING_PACK);
//	line_name[2] = '2';	
//	memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)line_drawing(_layer,_operate_type,start_x,start_y - 5, start_x, start_y - 5 - line_length, 3,_color, line_name), DRAWING_PACK);
//	line_name[2] = '3';
//	memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)line_drawing(_layer,_operate_type,start_x,start_y + 5, start_x, start_y + 5 + line_length, 3,_color, line_name), DRAWING_PACK);

//	//�ղ�����
//	line_name[2] = '4';	
//	memcpy(&data_pack[DRAWING_PACK*5], (uint8_t*)null_drawing(_layer, line_name), DRAWING_PACK);
//	line_name[2] = '5';
//	memcpy(&data_pack[DRAWING_PACK*6], (uint8_t*)null_drawing(_layer, line_name), DRAWING_PACK);
//	
//	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
//}

///**
// * @brief ���Զ���ͼ�㡿Ӣ�۵�UI��߻���
// * @note ���Ժ��ʵ������Ӣ�ۣ�
// */
//void referee_Classdef::Hero_UI_ruler(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t* line_distance, uint16_t* line_length, colorType_e* _color, drawOperate_e _operate_type)
//{
//	uint16_t total_distance = 0;
//	static uint8_t line_name[] = "he0";

//	//���Ƴ�ʼ׼�Ǻ���
//	line_name[2] = '6';
//	memcpy(&data_pack[DRAWING_PACK * 6], (uint8_t*)line_drawing(_layer,ADD_PICTURE,start_x-line_length[0]/2, start_y, start_x+line_length[0]/2, start_y, 2,_color[0], line_name), DRAWING_PACK);
//	total_distance += line_distance[0];

//	//��װ����׼��С���ߣ���2��6��
//	for(uint8_t i = 1;i < 6;i++)
//	{
//		line_name[2] = '0' + i;
//		memcpy(&data_pack[DRAWING_PACK * i], (uint8_t*)line_drawing(_layer,ADD_PICTURE,start_x-line_length[i]/2, start_y-total_distance, start_x+line_length[i]/2, start_y-total_distance, 2,_color[i], line_name), DRAWING_PACK);
//		
//		total_distance += line_distance[i];				//������ֱ���ܳ���
//	}
//	
//	//��װ����ֱ��
//	line_name[2] = '0';
//	memcpy(data_pack, (uint8_t*)line_drawing(_layer,ADD_PICTURE,start_x, start_y, start_x, start_y-total_distance, 1,_color[6], line_name), DRAWING_PACK);
//	
//	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
//}

///**
// * @brief ��ͼ��1�����ݵ�ѹ�ٷֱȻ���
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������ݿ���ʱ�����ӡ��ʾ�ַ�����ͬʱ�޸ĳ����ߺ�׼����ɫ���ù��ܴ����ơ�
// * @note ���Ժ��ʵ����������Referee.Draw_Cap_Energy(cell, flag, state, enablecnt, 1025, 620);
// */
//void referee_Classdef::Draw_Cap_Energy(uint8_t cap_cell, uint8_t enable, uint8_t state, uint8_t enable_cnt, uint16_t center_x, uint16_t center_y)
//{
//	static uint8_t volt_proportion;
//	static colorType_e _color;
//	static drawOperate_e _operate_type;

//	static uint8_t num_char[] = "0123456789";
//	
//	//���ݱ�����ʾ�ַ���
//	static uint8_t cap_str[] = "cap:";
//	static uint8_t cap_state_str[] = "ON  ";
//	static uint8_t cap_name1[] = "cp1";
//	
//	//�������ٷ���
//	static uint8_t cap_num_str[] = " 90%";
//	static uint8_t cap_name2[] = "cp2";
//	static uint8_t cap_name3[] = "cp3";
//		
//	volt_proportion = cap_cell;
//	
//	//��ѹ������ֵ���֣���ʾ��ͬ����ɫ
//	if(volt_proportion > 70)
//		_color = GREEN;
//	else if(volt_proportion > 30)
//		_color = YELLOW;
//	else
//		_color = PINK;
//	
//	//�������£����»���ͼ��
//	if(enable_cnt)
//	{	
//		//���Ƶ��������ַ�������Ҫ��λ���
//		_operate_type = ADD_PICTURE;
//		Draw_Char(1, center_x, center_y, cap_name1, cap_str, sizeof(cap_str), 20, WHITE, _operate_type);	
//	}
//	else
//	{
//		_operate_type = MODIFY_PICTURE;
//	}


//	
//	//����ѹֵת��Ϊ�ַ�����ʽ
//	if(volt_proportion < 100)
//	{
//		cap_num_str[0] = ' ';
//		cap_num_str[1] = num_char[volt_proportion / 10];
//		cap_num_str[2] = num_char[volt_proportion % 10];
//	}
//	else
//	{
//		cap_num_str[0] = '1';
//		cap_num_str[1] = '0';
//		cap_num_str[2] = '0';
//	}


//	//���������ٷ���
//	Draw_Char(1, center_x, center_y - 40, cap_name2, cap_num_str, sizeof(cap_num_str), 20, _color, _operate_type);

//	if(state == Off_line)
//	{
//		strcpy((char*)cap_state_str, "LOST");
//		_color = ORANGE;
//	}
//	else
//	{
//		enable==true?strcpy((char*)cap_state_str, "ON  "):strcpy((char*)cap_state_str, "OFF ");
//		enable==true?_color = GREEN:_color = WHITE;
//	}
//	//���Ƶ���״̬
//	Draw_Char(1, center_x+80, center_y, cap_name3, cap_state_str, sizeof(cap_state_str), 20, _color, _operate_type);
//}
//	
///**
// * @brief ��ͼ��1���������ݿ������ƣ���⵽�����ػ��½��غ󣬶�η���ȷ�����Ƴɹ� 
// * @param 
// * @note ���Ժ��ʵ��������ͨ������referee.Draw_Boost(boost_flag, 1500, 840, 10, PINK);
// */
//void referee_Classdef::Draw_Boost(uint8_t boost_flag,  uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color)
//{
//	static uint8_t last_boost_flag = 0;
//	static uint8_t enable_cnt[2] = {0,0};
//	static uint8_t boost_name[] = "bos";
//	
//	if(boost_flag && (!last_boost_flag))			//�ź������أ���λ��Ʊ�־
//		enable_cnt[0] = 6;
//	else if((!boost_flag) && last_boost_flag)		//�ź��½��أ����ɾ����־
//		enable_cnt[1] = 6;
//	else;
//	
//	if(enable_cnt[0])
//	{
//		enable_cnt[0]--;
//		memcpy(data_pack, (uint8_t*)arc_drawing(1, ADD_PICTURE, center_x,center_y,20,20,45,315,line_width,_color, boost_name), DRAWING_PACK);
//		pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);
//	}
//	else if(enable_cnt[1])
//	{
//		enable_cnt[1]--;
//		clean_one_picture(1, boost_name);
//	}

//	last_boost_flag = boost_flag;
//	return;
//}

///**
// * @brief ��ͼ��2�����ƾ�̬С���ݱ�־
// * @param spin_flag��С����ʹ�ܱ�־λ
// * @note ���Ժ��ʵ����������Referee.Draw_Spin(flag, enablecnt, 810, 750, 10, GREEN);
// */
//void referee_Classdef::Draw_Spin(uint8_t spin_flag, uint16_t enable_cnt, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color)
//{
//	static uint8_t first_init = true;
//	static uint8_t last_spin_flag = !spin_flag;
//	static uint8_t spin_name0[] = "sp0";
//	static uint8_t spin_name1[] = "sp1";
//	static uint8_t spin_name2[] = "sp2";
//	static uint8_t spin_name3[] = "sp3";
//	static uint8_t modify_cnt[2] = {0,0};

//	static uint8_t top_str[] = "Top";
//	static uint8_t top_name1[] = "to1";
//	static drawOperate_e _operate_type;



//	//--- ��ʼ��ͼ�㣬�໭����
//	if(enable_cnt)
//	{
//		_operate_type = ADD_PICTURE;
//		Draw_Char(0, center_x-25, center_y+60, top_name1, top_str, sizeof(top_str), 20, GREEN, _operate_type);

//		if(first_init == false) //--- ˢ��UI��ʱ����Ҫ�ٻ���һ��
//		{
//			last_spin_flag = !spin_flag;
//		}
//	}
//	else
//	{
//		first_init = false;
//		_operate_type = MODIFY_PICTURE;
//	}
//	
//	if(spin_flag && (!last_spin_flag))				//�ź������أ���λ��Ʊ�־
//	{
//		modify_cnt[0] = 6;
//		modify_cnt[1] = 0;
//	}
//	else if(last_spin_flag && (!spin_flag))			//�ź��½��أ����ɾ����־
//	{
//		modify_cnt[1] = 6;
//		modify_cnt[0] = 0;
//	}
//	else;
//	
//	if(modify_cnt[0])
//	{
//		modify_cnt[0]--;
//		clean_two_picture(2, spin_name2, spin_name3);

//		memcpy(data_pack, (uint8_t*)arc_drawing(2, ADD_PICTURE, center_x,center_y,20,20,180,270,line_width,_color, spin_name0), DRAWING_PACK);
//		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(2, ADD_PICTURE, center_x,center_y,20,20,0,90,line_width,_color, spin_name1), DRAWING_PACK);
//	
//		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
//	}
//	else if(modify_cnt[1])	//С���ݽ���ʱ�����ͼ��
//	{
//		modify_cnt[1]--;
//		clean_two_picture(2, spin_name0, spin_name1);

//		memcpy(data_pack, (uint8_t*)arc_drawing(2, ADD_PICTURE, center_x,center_y,20,20,270,360,line_width,DARKGREEN, spin_name2), DRAWING_PACK);
//		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(2, ADD_PICTURE, center_x,center_y,20,20,90,180,line_width,DARKGREEN, spin_name3), DRAWING_PACK);

//		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);

//	}

//	last_spin_flag = spin_flag;
//	return;
//}

///**
// * @brief ��ͼ��2�����ƾ�̬����Ť����־
// * @param specialswing_flag������Ť��ʹ�ܱ�־λ
// * @note ���Ժ��ʵ����������Referee.Draw_Spin(flag, enablecnt, 810, 750, 10, GREEN);
// */
//void referee_Classdef::Draw_SpecialSwing(uint8_t specialswing_flag, uint16_t enable_cnt, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color)
//{
//	static uint8_t first_init = true;
//	static uint8_t last_specialswing_flag = !specialswing_flag;
//	static uint8_t specialswing_name0[] = "ss0";
//	static uint8_t specialswing_name1[] = "ss1";
//	static uint8_t specialswing_name2[] = "ss2";
//	static uint8_t specialswing_name3[] = "ss3";
//	static uint8_t modify_cnt[2] = {0,0};

//	static uint8_t sps_str[] = "Sps";
//	static uint8_t sps_name1[] = "sp1";
//	static drawOperate_e _operate_type;



//	//--- ��ʼ��ͼ�㣬�໭����
//	if(enable_cnt)
//	{
//		_operate_type = ADD_PICTURE;
//		Draw_Char(0, center_x-25, center_y+60, sps_name1, sps_str, sizeof(sps_str), 20, GREEN, _operate_type);

//		if(first_init == false) //--- ˢ��UI��ʱ����Ҫ�ٻ���һ��
//		{
//			last_specialswing_flag = !specialswing_flag;
//		}
//	}
//	else
//	{
//		first_init = false;
//		_operate_type = MODIFY_PICTURE;
//	}
//	
//	if(specialswing_flag && (!last_specialswing_flag))				//�ź������أ���λ��Ʊ�־
//	{
//		modify_cnt[0] = 6;
//		modify_cnt[1] = 0;
//	}
//	else if(last_specialswing_flag && (!specialswing_flag))			//�ź��½��أ����ɾ����־
//	{
//		modify_cnt[1] = 6;
//		modify_cnt[0] = 0;
//	}
//	else;
//	
//	if(modify_cnt[0])
//	{
//		modify_cnt[0]--;
//		clean_two_picture(2, specialswing_name2, specialswing_name3);

//		memcpy(data_pack, (uint8_t*)arc_drawing(2, ADD_PICTURE, center_x,center_y,20,20,180,270,line_width,_color, specialswing_name0), DRAWING_PACK);
//		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(2, ADD_PICTURE, center_x,center_y,20,20,0,90,line_width,_color, specialswing_name1), DRAWING_PACK);
//	
//		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
//	}
//	else if(modify_cnt[1])	//С���ݽ���ʱ�����ͼ��
//	{
//		modify_cnt[1]--;
//		clean_two_picture(2, specialswing_name0, specialswing_name1);

//		memcpy(data_pack, (uint8_t*)arc_drawing(2, ADD_PICTURE, center_x,center_y,20,20,270,360,line_width,DARKGREEN, specialswing_name2), DRAWING_PACK);
//		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(2, ADD_PICTURE, center_x,center_y,20,20,90,180,line_width,DARKGREEN, specialswing_name3), DRAWING_PACK);

//		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);

//	}

//	last_specialswing_flag = specialswing_flag;
//	return;
//}

///**
// * @brief ��ͼ��3�����ֿ������ƣ���⵽�����ػ��½��غ󣬶�η���ȷ�����Ƴɹ�
// * @note ��ͼ������Ļ�������name�ظ���
// * @note ���Ժ��ʵ����������Referee.Draw_Magazine(flag, enablecnt, 1010, 750, 10, GREEN);
// */
//void referee_Classdef::Draw_Magazine(uint8_t mag_flag, uint16_t enable_cnt, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color)
//{
//	static uint8_t first_init = true;
//	static uint8_t last_mag_flag = !mag_flag;
//	static uint8_t modify_cnt[2] = {0,0};
//	static uint8_t mag_name0[] = "ma0";
//	static uint8_t mag_name1[] = "ma1";
//	static uint8_t mag_name2[] = "ma2";
//	static uint8_t mag_name3[] = "ma3";
//	static uint8_t mag_name4[] = "ma4";

//	static uint8_t mag_str[] = "R";
//	static drawOperate_e _operate_type;

//	//--- ��ʼ��ͼ�㣬�໭����
//	if(enable_cnt)
//	{
//		_operate_type = ADD_PICTURE;
//		//--- ��̬ͼ�㶼Ϊ0
//		Draw_Char(0, center_x-5, center_y+60, mag_name2, mag_str, sizeof(mag_str), 20, GREEN, _operate_type);

//		if(first_init == false) //--- ˢ��UI��ʱ����Ҫ�ٻ���һ��
//		{
//			last_mag_flag = !mag_flag;
//		}
//	}
//	else
//	{
//		first_init = false;
//		_operate_type = MODIFY_PICTURE;
//	}
//	
//	if(mag_flag == true && (!last_mag_flag))				//�ź������أ���λ��Ʊ�־
//	{
//		modify_cnt[0] = 6;
//		modify_cnt[1] = 0;
//	}
//	else if(last_mag_flag && mag_flag == false)			//�ź��½��أ����ɾ����־
//	{
//		modify_cnt[1] = 6;
//		modify_cnt[0] = 0;
//	}
//	else;
//	
//	if(modify_cnt[0])	//--- ON
//	{
//		modify_cnt[0]--;

//		clean_two_picture(3, mag_name3, mag_name4);

//		memcpy(data_pack, (uint8_t*)arc_drawing(3, ADD_PICTURE, center_x,center_y,20,20,180,270,line_width,_color, mag_name0), DRAWING_PACK);
//		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(3, ADD_PICTURE, center_x,center_y,20,20,0,90,line_width,_color, mag_name1), DRAWING_PACK);
//	
//		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
//	}
//	else if(modify_cnt[1])	//--- OFF
//	{
//		modify_cnt[1]--;
//		clean_two_picture(3, mag_name0, mag_name1);

//		memcpy(data_pack, (uint8_t*)arc_drawing(3, ADD_PICTURE, center_x,center_y,20,20,270,360,line_width, DARKGREEN, mag_name3), DRAWING_PACK);
//		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(3, ADD_PICTURE, center_x,center_y,20,20,90,180,line_width, DARKGREEN, mag_name4), DRAWING_PACK);

//		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
//	}

//	last_mag_flag = mag_flag;
//	return;
//}

///**
// * @brief ��ͼ��5��Ħ���ֿ������ƣ���⵽�����ػ��½��غ󣬶�η���ȷ�����Ƴɹ�
// * @note ��ͼ������Ļ�������name�ظ���
// * @note ���Ժ��ʵ����������Referee.Draw_Fric(flag, state, enablecnt, 1110, 750, 10, GREEN);
// */
//void referee_Classdef::Draw_Fric(uint8_t fric_flag, uint8_t state, uint16_t enable_cnt, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color)
//{
//	static uint8_t first_init = true;
//	static uint8_t last_fric_flag = !fric_flag, last_fric_state = 0;
//	static uint8_t fric_name0[] = "fr0";
//	static uint8_t fric_name1[] = "fr1";
//	static uint8_t modify_cnt[4] = {0,0,0,0};

//	static uint8_t fric_str[] = "FRIC";
//	static uint8_t fric_name2[] = "fr2";

//	static uint8_t fric_name3[] = "fr3";
//	static uint8_t fric_name4[] = "fr4";
//	static uint8_t fric_name5[] = "fr5";

//	static drawOperate_e _operate_type;

//	//--- ��ʼ��ͼ�㣬�໭����
//	if(enable_cnt)
//	{
//		_operate_type = ADD_PICTURE;
//		//--- ��̬�Ĺ̶��ַ���Ϊͼ��0
//		Draw_Char(0, center_x-30, center_y+60, fric_name0, fric_str, sizeof(fric_str), 20, GREEN, _operate_type);

//		if(first_init == false) //--- ˢ��UI��ʱ����Ҫ�ٻ���һ��
//		{
//			last_fric_flag = !fric_flag;
//			last_fric_state = !state;
//		}
//	}
//	else
//	{
//		first_init = false;
//		_operate_type = MODIFY_PICTURE;
//	}
//	/* �豸״̬ */
//	if(state == Off_line && (!last_fric_state))//����			//�ź������أ���λ��Ʊ�־
//	{
//		modify_cnt[2] = 6;
//		modify_cnt[3] = 0;
//	}
//	else if(state == On_line && (last_fric_state))//����		//�ź��½��أ����ɾ����־
//	{
//		modify_cnt[3] = 6;
//		modify_cnt[2] = 0;
//	}
//	else;
//	/* �豸���� */
//	if(fric_flag == true && (!last_fric_flag))      //�ź������أ���λ��Ʊ�־
//	{
//		modify_cnt[0] = 6;
//		modify_cnt[1] = 0;
//	}
//	else if(fric_flag == false && (last_fric_flag))  //�ź��½��أ����ɾ����־
//	{
//		modify_cnt[1] = 6;
//		modify_cnt[0] = 0;
//	}
//	else
//	{}
//	
//	if(modify_cnt[0] && state != Off_line)
//	{
//		modify_cnt[0]--;
//		clean_two_picture(5, fric_name3, fric_name4);

//		memcpy(data_pack, (uint8_t*)arc_drawing(5, ADD_PICTURE, center_x,center_y,20,20,180,270,line_width,_color, fric_name1), DRAWING_PACK);
//		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(5, ADD_PICTURE, center_x,center_y,20,20,0,90,line_width,_color, fric_name2), DRAWING_PACK);

//		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);

//	}
//	else if(modify_cnt[1] && state != Off_line)	
//	{
//		modify_cnt[1]--;
//		clean_two_picture(5, fric_name1, fric_name2);

//		memcpy(data_pack, (uint8_t*)arc_drawing(5, ADD_PICTURE, center_x,center_y,20,20,270,360,line_width,DARKGREEN, fric_name3), DRAWING_PACK);
//		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(5, ADD_PICTURE, center_x,center_y,20,20,90,180,line_width,DARKGREEN, fric_name4), DRAWING_PACK);

//		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
//	}

//	//--- ״̬��ʾͼ�� "/"
//	if(modify_cnt[2])
//	{
//		modify_cnt[2]--;

//		clean_two_picture(5, fric_name1, fric_name2);
//		clean_two_picture(5, fric_name3, fric_name4);

//		//--- �쳣״̬��־
//		memcpy(&data_pack, (uint8_t*)line_drawing(5, ADD_PICTURE, center_x-18, center_y-18, center_x+17, center_y+17, line_width, ORANGE, fric_name5), DRAWING_PACK);
//		pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)(data_pack), DRAWING_PACK);
//	}
//	else if(modify_cnt[3])
//	{
//		modify_cnt[3]--;
//		clean_one_picture(5, fric_name5); //--- ɾ��
//	}

//	//-------------------- �����ķָ��� --------------------//

//	// if(state != Off_line)
//	// {
//	// 	if(modify_cnt[0]) //--- ��
//	// 	{
//	// 		modify_cnt[0]--;

//	// 		memcpy(data_pack, (uint8_t*)arc_drawing(5, _operate_type, center_x,center_y,20,20,180,270,line_width,_color, fric_name1), DRAWING_PACK);
//	// 		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(5, _operate_type, center_x,center_y,20,20,0,90,line_width,_color, fric_name2), DRAWING_PACK);

//	// 		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);

//	// 	}
//	// 	else if(modify_cnt[1]) //--- ��
//	// 	{
//	// 		modify_cnt[1]--;

//	// 		memcpy(data_pack, (uint8_t*)arc_drawing(5, _operate_type, center_x,center_y,20,20,270,360,line_width, DARKGREEN, fric_name1), DRAWING_PACK);
//	// 		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(5, _operate_type, center_x,center_y,20,20,90,180,line_width, DARKGREEN, fric_name2), DRAWING_PACK);

//	// 		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
//	// 	}
//	// }
//	
//	// //--- ״̬��ʾͼ�� "/"
//	// if(modify_cnt[2])
//	// {
//	// 	modify_cnt[2]--;

//	// 	clean_two_picture(5, fric_name1, fric_name2);

//	// 	//--- �쳣״̬��־
//	// 	memcpy(&data_pack, (uint8_t*)line_drawing(5, ADD_PICTURE, center_x-18, center_y-18, center_x+17, center_y+17, line_width, ORANGE, fric_name5), DRAWING_PACK);
//	// 	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)(data_pack), DRAWING_PACK);
//	// }
//	// else if(modify_cnt[3])
//	// {
//	// 	modify_cnt[3]--;
//	// 	clean_one_picture(5, fric_name5); //--- ɾ�� "/" ��ʶ
//	// }

//	//----------------------------------------------------//
//	last_fric_state = state;
//	last_fric_flag = fric_flag;
//	return;
//}

///**
// * @brief ��ͼ��0�����Ƴ����ߣ��Զ������λ��
// * @note ��ͼ������Ļ�������name�ظ���
// * @note ���Ժ��ʵ��������ͨ������referee.Draw_Robot_Limit(180, 80, 961, 3, YELLOW, ADD_PICTURE);���߿�Ϊ3����ɫ��
// */
//void referee_Classdef::Draw_Robot_Limit(uint16_t height, uint16_t distance, uint16_t center_x, uint16_t line_width, colorType_e _color, drawOperate_e _operate_type)
//{
//	static uint8_t limit_name[] = "li0";

//	//�Ҳ೵���߻���
//	limit_name[2] = '0';
//	memcpy(data_pack, (uint8_t*)line_drawing(0, _operate_type, center_x + distance, height , center_x + distance + 200, height, line_width, _color, limit_name), DRAWING_PACK);
//	limit_name[2] = '1';
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(0, _operate_type, center_x + distance + 200, height, center_x + distance + 360, height - 100, line_width, _color, limit_name), DRAWING_PACK);
//	
//	//��೵���߻���
//	limit_name[2] = '2';
//	memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(0, _operate_type, center_x - distance, height, center_x - distance - 200, height, line_width, _color, limit_name), DRAWING_PACK);
//	limit_name[2] = '3';
//	memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)line_drawing(0, _operate_type, center_x - distance - 200, height, center_x - distance - 360, height - 100, line_width, _color, limit_name), DRAWING_PACK);
//	
//	//�հ�
//	limit_name[2] = '4';
//	memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)null_drawing(0, limit_name), DRAWING_PACK);
//		
//	pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);
//}

///**
// * @brief ��ͼ��0���������鿪��ʱ���ַ���ʾ���Զ������λ��
// * @note ��ͼ������Ļ�������name�ظ���
// * @note ���Ժ��ʵ����������Referee.Draw_Vision(flag, state, enablecnt, 910, 750, 10, GREEN);
// */
//void referee_Classdef::Draw_Vision(uint8_t vision_flag, uint8_t state,  uint8_t depth, uint16_t enable_cnt, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color)
//{
//	static uint8_t first_init = true;
//	static uint8_t last_vision_flag = !vision_flag, last_vision_state = 0;
//	static uint8_t modify_cnt[4] = {0};
//	static uint8_t auto_name0[] = "au0";
//	static uint8_t auto_name1[] = "au1";
//	// static uint8_t auto_name2[] = "au2";
//	// static uint8_t auto_name3[] = "au3";
//	static uint8_t auto_name4[] = "au4";

//	static uint8_t vision_str0[] = "V";
//	static uint8_t vision_name0[] = "v1";
//	static uint8_t vision_str1[] = "0";
//	static uint8_t vision_name1[] = "v2";
//	static uint8_t vision_mode_str[] = "0123456";

//	static uint8_t dep_name0[] = "de0";
//	static uint8_t dep_name1[] = "de1";
//	static uint8_t dep_str[] = "Depth:";
//	static uint8_t dep_num_str[] = "0.0";
//	static uint8_t num_char[] = "0123456789";

//	static drawOperate_e _operate_type;

//	//--- ��ʼ��ͼ�㣬�໭����
//	if(enable_cnt)
//	{
//		_operate_type = ADD_PICTURE;
//		// ��̬ͼ�㶼Ϊ0
//		Draw_Char(0, center_x-20, center_y+60, vision_name0, vision_str0, sizeof(vision_str0), 20, GREEN, _operate_type);

//		Draw_Char(0, 780, 620, dep_name0, dep_str, sizeof(dep_str), 20, PURPLE, _operate_type);

//		if(first_init == false) //--- ˢ��UI��ʱ����Ҫ�ٻ���һ��
//		{
//			last_vision_flag = !vision_flag;
//			last_vision_state = !state;
//		}
//		
//	}
//	else
//	{
//		first_init = false;
//		_operate_type = MODIFY_PICTURE;
//	}

//	// if(vision_flag == true && (!last_vision_flag))	  // �ź������أ���λ���ͼ��
//	// {
//	// 	modify_cnt[0] = 6;
//	// 	modify_cnt[1] = 0;
//	// }
//	// else if(last_vision_flag && vision_flag == false) // �ź��½��أ����ɾ��ͼ��
//	// {
//	// 	modify_cnt[1] = 6;
//	// 	modify_cnt[0] = 0;
//	// }
//	// else{};

//	if(state == Off_line && (!last_vision_state))		// �ź������أ���λ���ͼ��
//	{
//		modify_cnt[2] = 6;
//		modify_cnt[3] = 0;
//	}
//	else if(last_vision_state && state == On_line) // �ź��½��أ����ɾ��ͼ��
//	{
//		modify_cnt[3] = 6;
//		modify_cnt[2] = 0;

//		modify_cnt[0] = 6;
//	}
//	else{};

//	if(modify_cnt[0] && state != Off_line)
//	{
//		modify_cnt[0]--;

//		// clean_two_picture(0, auto_name2, auto_name3);
//		memcpy(data_pack, (uint8_t*)arc_drawing(0, ADD_PICTURE, center_x,center_y,20,20,180,270,line_width,_color, auto_name0), DRAWING_PACK);
//		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(0, ADD_PICTURE, center_x,center_y,20,20,0,90,line_width,_color, auto_name1), DRAWING_PACK);

//		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
//	}
//	// else if(modify_cnt[1] && state != Off_line)
//	// {
//	// 	modify_cnt[1]--;
//	// 	clean_two_picture(0, auto_name0, auto_name1);
//	// 	memcpy(data_pack, (uint8_t*)arc_drawing(0, ADD_PICTURE, center_x,center_y,20,20,270,360,line_width, DARKGREEN, auto_name2), DRAWING_PACK);
//	// 	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(0, ADD_PICTURE, center_x,center_y,20,20,90,180,line_width, DARKGREEN, auto_name3), DRAWING_PACK);
//	// 	pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
//	// }
//	//--- ״̬��ʾͼ�� "/"
//	if(modify_cnt[2])
//	{
//		modify_cnt[2]--;

//		clean_two_picture(0, auto_name0, auto_name1);
//		// clean_two_picture(0, auto_name2, auto_name3);

//		//--- �쳣״̬��־
//		memcpy(&data_pack, (uint8_t*)line_drawing(0, ADD_PICTURE, center_x-18, center_y-18, center_x+17, center_y+17, line_width, ORANGE, auto_name4), DRAWING_PACK);
//		pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)(data_pack), DRAWING_PACK);
//	}
//	else if(modify_cnt[3])
//	{
//		modify_cnt[3]--;
//		clean_one_picture(0, auto_name4); //--- ɾ��
//	}

//	//�����Ӿ�ģʽ
//	vision_str1[0] = vision_mode_str[vision_flag];
//	Draw_Char(0, center_x, center_y+60, vision_name1, vision_str1, sizeof(vision_str1), 20, _color, _operate_type);	

//	//�������
//	depth = depth>99?99:depth;
//	if(depth>=10)
//	{
//		dep_num_str[0] = num_char[depth/10];
//		dep_num_str[2] = num_char[depth%10];
//	}
//	else
//	{
//		dep_num_str[0] = '0';
//		dep_num_str[2] = num_char[depth];
//	}
//	
//	Draw_Char(4, 820, 580, dep_name1, dep_num_str, sizeof(dep_num_str), 20, PURPLE, _operate_type);



//	last_vision_flag = vision_flag;
//	last_vision_state = state;
//	return;
//}

///**
// * @brief ��ͼ��4��No Bullet
// * @note ��ͼ������Ļ�������name�ظ���
// * @note ���Ժ��ʵ����������Referee.Draw_Bullet(enablecnt,780,620);
// */
//void referee_Classdef::Draw_Bullet(uint8_t enable_cnt, uint16_t center_x, uint16_t center_y)
//{
//	// static uint8_t last_flag = 0;
//	// static uint8_t modify_cnt[2] = {0,0};
//	static uint8_t bullet_name0[] = "bu0";
//	static uint8_t bullet_name1[] = "bu1";
//	// static uint8_t bullet_str[] = "Bullet:";
//	static uint8_t bullet_str[] = "Depth:";
//	static uint8_t bullet_num_str[] = "0000";
//	static uint8_t num_char[] = "0123456789";

//	static drawOperate_e _operate_type;

//	// ��ʾ��ͬ����ɫ
//	// if(BulletRemaining.bullet_remaining_num_17mm > 200)
//	// 	_color = GREEN;
//	// else if(BulletRemaining.bullet_remaining_num_17mm > 100)
//	// 	_color = YELLOW;
//	// else
//	// 	_color = PINK;

//	

//	//�������£����»���ͼ��
//	if(enable_cnt)
//	{	
//		//�����ӵ������ַ�������Ҫ��λ���
//		_operate_type = ADD_PICTURE;
//		Draw_Char(0, center_x, center_y, bullet_name0, bullet_str, sizeof(bullet_str), 20, WHITE, _operate_type);
//	}
//	else
//		_operate_type = MODIFY_PICTURE;

//	
//	bullet_num_str[0] = num_char[BulletRemaining.bullet_remaining_num_17mm / 1000];
//	bullet_num_str[1] = num_char[BulletRemaining.bullet_remaining_num_17mm%1000/100];
//	bullet_num_str[2] = num_char[BulletRemaining.bullet_remaining_num_17mm%100/10];
//	bullet_num_str[3] = num_char[BulletRemaining.bullet_remaining_num_17mm % 10];


//	//���������ӵ���
//	Draw_Char(4, center_x+45, center_y - 40, bullet_name1, bullet_num_str, sizeof(bullet_num_str), 20, PURPLE, _operate_type);
//}


///**
// * @brief ��ͼ��6����Ѫ����ʾ
// * @note ��ͼ������Ļ�������name�ظ���
// * @note ���Ժ��ʵ��������ͨ������Draw_LowHP(850, 700)
// */
//void referee_Classdef::Draw_LowHP(uint16_t center_x, uint16_t center_y)
//{
//	static uint8_t hp_name0[] = "hp0";
//	static uint8_t low_hp_str[] = "GG!RUN!";
//	static uint8_t modify_cnt = 0;

//	static drawOperate_e _operate_type;

//	modify_cnt++;

//	if(GameRobotState.remain_HP <= GameRobotState.max_HP*0.3)
//	{
//		_operate_type = ADD_PICTURE;
//	}
//	else
//	{
//		_operate_type = CLEAR_ONE_PICTURE;
//	}
//	if((modify_cnt%=2)==0)
//	{
//		Draw_Char(6, center_x, center_y, hp_name0, low_hp_str, sizeof(low_hp_str), 40, ORANGE, _operate_type);		
//	}

//}

///**
// * @brief ��ͼ��7������Ŀ�궯̬װ�װ�
// * @note ��ͼ������Ļ�������name�ظ���
// * @note ���Ժ��ʵ��������ͨ������
// */
//void referee_Classdef::Draw_Armor(uint8_t enable_cnt, uint8_t vision_state, uint16_t center_x, uint16_t center_y, uint16_t length, uint16_t width, uint16_t line_size, colorType_e _color)
//// void referee_Classdef::Draw_Armor(uint8_t enable_cnt, uint8_t x_angle, uint16_t y_angle, uint16_t length, uint16_t width, uint16_t line_size, colorType_e _color)
//{
//	static uint8_t armor_name0[] = "ar0";
//	// static uint8_t armor_name1[] = "ar1";
//	// static uint8_t armor_name2[] = "ar2";
//	// static uint8_t armor_name3[] = "ar3";
//	// static uint8_t armor_name4[] = "ar4";
//	// static uint8_t armor_name5[] = "ar5";

//	// static uint8_t armor_str[] = "ARMOR";

//	static drawOperate_e _operate_type;

//	//--- ��ʼ��ͼ�㣬�໭����
//	if(enable_cnt || vision_state == false)
//	{
//		_operate_type = ADD_PICTURE;
//		memcpy(data_pack, (uint8_t*)rectangle_drawing(7, _operate_type, 960 - length/2, 540 - width/2, length, width, line_size, _color, armor_name0), DRAWING_PACK);
//	}
//	else
//	{
//		_operate_type = MODIFY_PICTURE;
//		memcpy(data_pack, (uint8_t*)rectangle_drawing(7, _operate_type, center_x - length/2, center_y - width/2, length, width, line_size, _color, armor_name0), DRAWING_PACK);
//	}


//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

//	return;
//}

///**
// * @brief ���Զ���ͼ�㡿���л�����UI���
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Aerial_PitchRuler_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint16_t long_scale_length, uint16_t short_scale_length, colorType_e ruler_color, colorType_e current_color, colorType_e target_color)
//{
//	static uint8_t line_name[] = "600";					//Aerial id + tag

//	uint16_t scale_step = total_length / 30;
//	uint16_t scale_point[7];
//	uint8_t i = 0;									//��������

//	/* ����ÿ����̶��ߵĴ�ֱ���� */
//	scale_point[0] = center_y + total_length/2;		//��һ����̶ȣ�+15��
//	scale_point[6] = center_y - total_length/2;		//���һ����̶ȣ�-15��

//	for(i = 1;i < 6;i++)							//�м��̶�
//		scale_point[i] = scale_point[0] - scale_step*5*i;
//	
//	line_name[1] = '0';
//	line_name[2] = '0';								//��ֱ�߻���
//	memcpy(data_pack, (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x, scale_point[0], center_x, scale_point[6], 3, ruler_color, line_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

//	for(i = 0;i < 7;i++)
//	{
//		line_name[2] += (i + 1);
//		if(i % 2 == 0)								//�����̶��ߣ�����
//			memcpy(&data_pack[DRAWING_PACK*i], (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x-short_scale_length/2, scale_point[i], center_x+short_scale_length/2, scale_point[i], 3, ruler_color, line_name), DRAWING_PACK);
//		else										//ż���̶��ߣ�����
//			memcpy(&data_pack[DRAWING_PACK*i], (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x-long_scale_length/2, scale_point[i], center_x+long_scale_length/2, scale_point[i], 3, ruler_color, line_name), DRAWING_PACK);
//	}
//	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);

//	/* ����ÿ��С�̶��� */
//	for(i = 1;i < 7;i++)
//	{
//		line_name[1] += i;
//		line_name[2] = '0';

//		for(uint8_t j = 0;j < 4;j++)
//		{
//			line_name[2] += j;
//			memcpy(&data_pack[DRAWING_PACK*j], (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x-short_scale_length/4, scale_point[i]+scale_step*(j+1), center_x+short_scale_length/4, scale_point[i]+scale_step*(j+1), 3, ruler_color, line_name), DRAWING_PACK);
//		}
//		//�հ�
//		line_name[2] = '4';
//		memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)null_drawing(0, line_name), DRAWING_PACK);
//		pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);	
//	}

//	/* ���Ʊ�ߴ�̶ȶ�Ӧ���� */
//	line_name[1] = 'c';	
//	line_name[2] = '0';
//	for(i = 0;i < 7;i++)
//	{
//		line_name[2] += i;
//		memcpy(&data_pack[DRAWING_PACK*i], (uint8_t*)float_drawing(_layer, ADD_PICTURE, center_x+long_scale_length, scale_point[i] + 8, 16, 2, ruler_color, 15 - 5*i, line_name), DRAWING_PACK);
//	}
//	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
//	
//	/* ���ƿ��л����˸���Ŀ��ֵ */
//	line_name[1] = 'p';
//	line_name[2] = '0';
//	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, ADD_PICTURE, center_x,center_y , 10, 16, target_color, line_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);	
//	
//	/* ���ƿ��л����˳�ʼ���� */
//	line_name[2] = '1';
//	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, ADD_PICTURE, center_x,center_y , 10, 16, current_color, line_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);	
//}

///**
// * @brief ���Զ���ͼ�㡿���л�����pitch���굱ǰֵ
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Aerial_Pitch_Update(float pitch_angle, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e tag_color)
//{
//	static uint8_t point_str[] = "6p1";
//	
//	pitch_angle = _referee_Constrain((float)pitch_angle, (float)(-15.0), (float)(15.0));
//	
//	/* ���㸡�������λ�ò����� */
//	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, MODIFY_PICTURE, center_x, center_y + pitch_angle*total_length/30, 10, 16, tag_color, point_str), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);	
//	
//}

///**
// * @brief ���Զ���ͼ�㡿���л�����pitch����Ŀ��ֵ
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Aerial_Pitch_Target(float pitch_target, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e tag_color)
//{
//	static uint8_t point_str[] = "6p0";
//	pitch_target = _referee_Constrain((float)pitch_target, (float)(-15.0), (float)(15.0));
//	
//	/* ���㸡�������λ�ò����� */
//	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, MODIFY_PICTURE, center_x, center_y + pitch_target*total_length/30, 10, 16, tag_color, point_str), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);	
//}

///**
// * @brief ���Զ���ͼ�㡿���̻�����̧���߶ȱ�ߣ������Ի���ʮ�����
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Engineer_HighthRuler_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint16_t long_scale_length, uint16_t short_scale_length, uint8_t ruler_tag, colorType_e ruler_color, colorType_e current_color)
//{
//	uint8_t line_name[] = "200";					//Engineer id + tag

//	// uint16_t scale_step = total_length / 20;
//	uint16_t scale_point[21];						//���ˮƽ�̶��ߵĴ�ֱ����
//	uint8_t i = 0;									//��������

//	/* ����ÿ���̶��ߵĴ�ֱ���� */
//	scale_point[0] = center_y + total_length/2;		//��һ���̶�
//	
//	for(i = 1;i < 21;i++)							//�����̶�
//		scale_point[i] = scale_point[0] - total_length/20*i;
//	
//	line_name[1] = '0' + ruler_tag;
//	line_name[2] = 'a';								//��ֱ�߻���
//	memcpy(data_pack, (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x, scale_point[0], center_x, scale_point[20], 3, ruler_color, line_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

//	/* ˮƽ�̶��߻��� */

//	line_name[2] += 1;
//	for(i = 0;i < 21;i++)
//	{
//		line_name[2] += i;
//		if(i % 2 == 0)								//ż����ˮƽ�̶��ߣ�����
//			memcpy(&data_pack[DRAWING_PACK*(i%7)], (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x-long_scale_length/2, scale_point[i], center_x+long_scale_length/2, scale_point[i], 3, ruler_color, line_name), DRAWING_PACK);
//		else										//������ˮƽ�̶��ߣ�����
//			memcpy(&data_pack[DRAWING_PACK*(i%7)], (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x-short_scale_length/2, scale_point[i], center_x+short_scale_length/2, scale_point[i], 3, ruler_color, line_name), DRAWING_PACK);
//	
//		if((i+1)%7 == 0)							//װ���߸�ͼ�������һ��
//			pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
//	}

//	/* ���Ʊ�ߴ�̶ȶ�Ӧ���� */
//	line_name[1] = 'c' + ruler_tag;	
//	line_name[2] = 'a';														//�����Сд��ĸa��ʼ����
//	for(i = 0;i < 7;i++)
//	{
//		line_name[2] += i;
//		memcpy(&data_pack[DRAWING_PACK*i], (uint8_t*)int_drawing(_layer, ADD_PICTURE, center_x+long_scale_length, scale_point[i*2] + 8, 16, 2, ruler_color, (10-i)*10000, line_name), DRAWING_PACK);
//	}
//	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
//	
//	for(i = 7;i < 11;i++)
//	{
//		line_name[2] += (i-6);
//		memcpy(&data_pack[DRAWING_PACK*(i-7)], (uint8_t*)int_drawing(_layer, ADD_PICTURE, center_x+long_scale_length, scale_point[i*2] + 8, 16, 2, ruler_color, (10-i)*10000, line_name), DRAWING_PACK);
//	}
//	//�հ�
//	line_name[2] += 1;
//	memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)null_drawing(0, line_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);	

//	/* ���Ƹ߶ȳ�ʼ���� */
//	line_name[1] = 'h' + ruler_tag;
//	line_name[2] = '1';
//	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, ADD_PICTURE, center_x,center_y , 10, 18, current_color, line_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);	
//}

///**
//* @brief ���Զ���ͼ�㡿���̻����˵�ǰ̧���߶ȸ��꣨������ߣ�
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Engineer_Height_Update(float height, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint8_t ruler_tag, colorType_e tag_color)
//{
//	static uint8_t point_str[] = "2h1";			//Engineer ID + 'h' + ruler_tag
//	
//	height = _referee_Constrain((float)height, (float)(0.0), (float)(1.0));
//	
//	/* ���㸡�������λ�ò����� */
//	point_str[1] = 'h' + ruler_tag;				//�ڼ������
//	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, MODIFY_PICTURE, center_x, center_y + (height - 0.5)*(float)total_length, 10, 16, tag_color, point_str), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);		
//}

///**
//* @brief ���Զ���ͼ�㡿���̻����˵�ǰ̧���߶ȸ��꣨��������ͬʱ���ƣ������ŵ��ӳٴ����Ĳ�ͬ����
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Engineer_HeightMul_Update(float *height, uint8_t _layer, uint16_t *center_x, uint16_t *center_y, uint16_t *total_length, colorType_e tag_color)
//{
//	static uint8_t point_str[] = "2h1";
//	
//	point_str[1] = 'h';
//	point_str[2] = '1';
//	
//	for(uint8_t i = 0;i < 2;i++)
//	{
//		height[i] = _referee_Constrain((float)height[i], (float)(0.0), (float)(1.0));
//		/* ���㸡�������λ�ò����� */
//		point_str[1] = 'h' + i;				//��i�����
//		memcpy(&data_pack[DRAWING_PACK*i], (uint8_t*)circle_drawing(_layer, MODIFY_PICTURE, center_x[i], center_y[i] + (height[i] - 0.5)*(float)total_length[i], 10, 16, tag_color, point_str), DRAWING_PACK);
//		
//	}

//	pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);	
//}

///**
//* @brief ���Զ���ͼ�㡿���̻�����Ŀ��̧���߶ȸ���
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Engineer_Target_Height(float target_height, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint8_t ruler_tag, colorType_e tag_color)
//{
//	uint8_t target_point_str[] = "2t1";
//	
//	target_height = _referee_Constrain((float)target_height, (float)(0.0), (float)(1.0));
//	
//	/* ����Ŀ�긡�������λ�ò����� */
//	target_point_str[1] += ruler_tag;				//�ڼ������
//	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, MODIFY_PICTURE, center_x, center_y + (target_height - 0.5)*(float)total_length, 10, 16, tag_color, target_point_str), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);		
//}

///**
// * @brief ���Զ���ͼ�㡿��ʯUI
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::Mine_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t mine_tag, colorType_e _color, drawOperate_e _operate_type)
//{	
//	uint8_t mine_name[] = "2ma";
//	uint8_t mine_r = 'R';

//	repeat_cnt = 3;
//	
//	/* ���� */
//	mine_name[2] += mine_tag;
//	memcpy(data_pack, (uint8_t*)rectangle_drawing(_layer, _operate_type, center_x - size/2, center_y - size/2, size, size, size/10, _color, mine_name), DRAWING_PACK);
//	
//	while(repeat_cnt--)
//		pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

//	/* �ַ�R */
//	mine_name[2] -= 0x20;
//	
//	repeat_cnt = 3;
//	while(repeat_cnt--)
//		Draw_Char(_layer, center_x - size/10, center_y + size/4, mine_name, &mine_r, 1, size/2, _color, _operate_type);
//}

///**
// * @brief ���Զ���ͼ�㡿���̻����˵�ǰ�洢��ʯ��UI���
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Engineer_MineRe_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size)
//{
//	uint8_t mine_str[] = "Re:0";
//	uint8_t char_str[] = "2m0";
//	
//	Mine_Icon(_layer, start_x, start_y, size, 1, YELLOW, ADD_PICTURE);
//	Draw_Char(_layer, start_x + size, start_y + size*0.3, char_str, mine_str, 4, size*0.5, YELLOW, ADD_PICTURE);	
//}

///**
// * @brief ���Զ���ͼ�㡿���̻����˵�ǰ�洢��ʯ��
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::Engineer_MineRe_Update(uint8_t mine_num, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size)
//{
//	static uint8_t mine_str[] = "Re:0";
//	static uint8_t char_str[] = "2m0";
//	static uint8_t last_mine_num = mine_num;
//	
//	repeat_cnt = 2;													//�ظ���������
//	
//	if(mine_num == last_mine_num)
//		return;
//	else
//	{
//		/* ������Ŀ���� */
//		mine_str[3] = '0' + mine_num;
//		
//		while(repeat_cnt--)
//			Draw_Char(_layer, start_x + size, start_y + size*0.3, char_str, mine_str, 4, size*0.5, YELLOW, MODIFY_PICTURE);
//		
//		last_mine_num = mine_num;
//	}
//}

///**
// * @brief ���Զ���ͼ�㡿���̻������Զ�/�ֶ���״̬ 
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::Engineer_MineMode_Update(uint8_t mode, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size, colorType_e _color)
//{
//	static uint8_t mode_str[] = "AUTO   MODE";
//	static uint8_t mode_name[] = "2M0";
//	static uint8_t last_mode = mode;
//	
//	repeat_cnt = 2;									//�ظ���������
//	
//	if(mode == last_mode)
//		return;
//	else
//	{
//		clean_one_picture(_layer, mode_name);		//���ԭ��ͼ��
//		
//		if(mode)									//�Զ�ģʽ
//			memcpy(mode_str, "AUTO  ", 6);
//		else										//�ֶ�ģʽ
//			memcpy(mode_str, "HANDLE", 6);
//		
//		while(repeat_cnt--)
//			Draw_Char(_layer, start_x, start_y, mode_name, mode_str, sizeof(mode_str), size, _color, MODIFY_PICTURE);
//		
//		last_mode = mode;
//	}
//}

///**
// * @brief ���Զ���ͼ�㡿���̻������Զ���״̬
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::Engineer_AutoMode_Update(uint8_t status, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size)
//{
//	static uint8_t Exchange_name[] = "2e0";
//	static uint8_t last_status = status;	
//	
//	if(status == last_status)
//		return;
//	else
//	{
//		Mine_Icon(_layer, start_x + size/2, start_y - size/2, size, 2, YELLOW, CLEAR_ONE_PICTURE);				//���ԭ��ͼ��	
//		
//		repeat_cnt = 3;
//		while(repeat_cnt--)
//			clean_one_picture(_layer, Exchange_name);
//		
//		switch(status){
//			case 1:Mine_Icon(_layer, start_x + size/2, start_y - size/2, size, 2, YELLOW, ADD_PICTURE);break;	//ȡ���
//			case 2:Mine_Icon(_layer, start_x + size/2, start_y - size/2, size, 2, WHITE, ADD_PICTURE);break;	//ȡ����
//			case 3:																								//�һ�����
//				Mine_Icon(_layer, start_x + size/2, start_y - size/2, size, 2, GREEN, ADD_PICTURE);
//				memcpy(data_pack, (uint8_t*)line_drawing(_layer, ADD_PICTURE, start_x - size/2, start_y - size/2, start_x + size/2, start_y - size/2, size/3, GREEN, Exchange_name), DRAWING_PACK);	
//				
//				repeat_cnt = 3;
//				while(repeat_cnt--)
//					pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);
//			break;	
//		
//			default:break;																						//����������ʱ�����ֳ�����Ч�������ԭ��ͼ��
//		};
//		
//		last_status = status;																					//״̬����
//	}
//}

///**
// * @brief ���Զ���ͼ�㡿���̻����˾�Ԯ��״̬
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::Engineer_RescueMode_Update(uint8_t mode, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size, colorType_e _color)
//{
//	static uint8_t mode_str[10];
//	static uint8_t mode_name[] = "2M1";
//	static uint8_t last_mode = mode;
//	
//	if(mode == last_mode)
//		return;
//	else
//	{
//		repeat_cnt = 2;
//		while(repeat_cnt--)
//			clean_one_picture(_layer, mode_name);		//���ԭ��ͼ��
//		
//		switch(mode){
//			case 1: memcpy(mode_str, "Classical ", 10);break;
//			case 2: memcpy(mode_str, "Steering  ", 10);break;
//			case 3: memcpy(mode_str, "Swipe Card", 10);break;
//			
//			default:																							//����������ʱ�����ֳ�����Ч�������ԭ��ͼ��
//				last_mode = mode;
//				return;
//		};
//		
//		repeat_cnt = 2;																							//�ظ���������
//		while(repeat_cnt--)
//			Draw_Char(_layer, start_x, start_y, mode_name, mode_str, sizeof(mode_str), size, _color, ADD_PICTURE);
//		
//		last_mode = mode;
//	}
//}

///**
// * @brief ���Զ���ͼ�㡿�ڱ��������ƶ��������
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Sentry_PosRuler_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e ruler_color, colorType_e current_color)
//{
//	uint8_t line_name[] = "7r0";

//	//�ڱ����
//	memcpy(data_pack, (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x - total_length/2, center_y, center_x + total_length/2, center_y, 3, ruler_color, line_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

//	//�ڱ���ʼ����λ��
//	line_name[2] += 1;
//	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, ADD_PICTURE, center_x,center_y , 10, 18, current_color, line_name), DRAWING_PACK);
//	line_name[2] += 1;
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x, center_y + 40, center_x, center_y - 40, 6, current_color, line_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
//}

///**
// * @brief ���Զ���ͼ�㡿�ڱ������˵�ǰλ�ø���
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Sentry_Pos_Update(float pos_percent, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e tag_color)
//{
//	static uint8_t pos_name[] = "7r1";
//	static uint16_t current_pos = 0;

//	pos_percent = _referee_Constrain((float)pos_percent, (float)0.0, (float)1.0);
//	current_pos = (pos_percent - 0.5)*(float)total_length + center_x;

//	pos_name[2] = '1';
//	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, MODIFY_PICTURE, current_pos, center_y , 10, 18, tag_color, pos_name), DRAWING_PACK);
//	pos_name[2] += 1;
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, MODIFY_PICTURE, current_pos, center_y + 40, current_pos, center_y - 40, 6, tag_color, pos_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
//}

///**
// * @brief ���Զ���ͼ�㡿�ڱ�������Ѳ���������
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Sentry_Patrol_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t size, uint8_t *keys, colorType_e patrol_color)
//{
//	uint8_t line_name[] = "7a0";

//	/* ���������ɫ */
//	memcpy(data_pack, (uint8_t*)line_drawing(_layer, ADD_PICTURE, center_x - size/2, center_y, center_x + size/2, center_y, size, WHITE, line_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

//	/* �������򷽿� */
//	//������
//	line_name[2] += 1;
//	memcpy(data_pack, (uint8_t*)line_drawing(_layer, ADD_PICTURE, center_x - size/2, center_y, center_x + size/2, center_y, 3, BLACK, line_name), DRAWING_PACK);
//	line_name[2] += 1;
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, ADD_PICTURE, center_x, center_y - size/2, center_x, center_y + size/2, 3, BLACK, line_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);

//	//����
//	line_name[2] += 1;
//	memcpy(data_pack, (uint8_t*)rectangle_drawing(_layer, ADD_PICTURE, center_x - size/2, center_y - size/2, size, size, 3, BLACK, line_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

//	//�ַ�
//	line_name[2] += 1;
//	Draw_Char(_layer, center_x-size*0.4, center_y+size*0.4, line_name, keys, 1, size*0.3, BLACK, ADD_PICTURE);
//	line_name[2] += 1;
//	Draw_Char(_layer, center_x-size*0.4, center_y-size*0.1, line_name, keys+1, 1, size*0.3, BLACK, ADD_PICTURE);
//	line_name[2] += 1;
//	Draw_Char(_layer, center_x+size*0.1, center_y+size*0.4, line_name, keys+2, 1, size*0.3, BLACK, ADD_PICTURE);
//	line_name[2] += 1;
//	Draw_Char(_layer, center_x+size*0.1, center_y-size*0.1, line_name, keys+3, 1, size*0.3, BLACK, ADD_PICTURE);

//	/* ���Ƴ�ʼ�ڱ�Ѳ�����򣬻��������Ͻ� */
//	line_name[2] += 1;
//	memcpy(data_pack, (uint8_t*)line_drawing(_layer, ADD_PICTURE, center_x - size/2, center_y + size/4, center_x, center_y + size/4, size/2, patrol_color, line_name), DRAWING_PACK);
//	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

//}

///**
// * @brief ���Զ���ͼ�㡿�ڱ�����������״̬����
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::Sentry_Patrol_Update(uint8_t tag, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t size, colorType_e _color)
//{
//	static uint8_t patrol_area[] = "7a8";
//	static uint8_t last_patrol_tag = 255;
//	
//	if(last_patrol_tag == tag)
//		return;
//	else
//	{
//		switch (tag)
//		{
//		case 1:				//����
//			memcpy(data_pack, (uint8_t*)line_drawing(_layer, MODIFY_PICTURE, center_x - size/2, center_y + size/4, center_x, center_y + size/4, size/2, _color, patrol_area), DRAWING_PACK);
//			break;
//		case 2:				//����
//			memcpy(data_pack, (uint8_t*)line_drawing(_layer, MODIFY_PICTURE, center_x, center_y + size/4, center_x + size/2, center_y + size/4, size/2, _color, patrol_area), DRAWING_PACK);
//			break;
//		case 3:				//����
//			memcpy(data_pack, (uint8_t*)line_drawing(_layer, MODIFY_PICTURE, center_x - size/2, center_y - size/4, center_x, center_y - size/4, size/2, _color, patrol_area), DRAWING_PACK);
//			break;
//		case 4:				//����
//			memcpy(data_pack, (uint8_t*)line_drawing(_layer, MODIFY_PICTURE, center_x, center_y - size/4, center_x + size/2, center_y - size/4, size/2, _color, patrol_area), DRAWING_PACK);
//			break;	
//		default:
//			break;
//		}
//		
//		repeat_cnt = 3;			//�ظ���������
//		
//		while(repeat_cnt--)
//			pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);
//		
//		last_patrol_tag = tag;
//	}
//}

///**
// * @brief ���Զ���ͼ�㡿�ڱ������˵���״̬���
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Sentry_Bullet_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color)
//{
//	uint8_t bullet_name[] = "7b0";
//	uint8_t bullet_str[] = "Freq:      ";
//	
//	Draw_Char(_layer, start_x, start_y, bullet_name, bullet_str, sizeof(bullet_str), size, _color, ADD_PICTURE);
//}

///**
// * @brief ���Զ���ͼ�㡿�ڱ������˵��跢��״̬����
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::Sentry_Bullet_Update(uint8_t tag, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color)
//{
//	static uint8_t bullet_name[] = "7b0";
//	static uint8_t bullet_str[] = "Freq:      ";
//	
//	static uint8_t last_tag = 255;
//	
//	if(last_tag == tag)												//״̬û���£�ֱ�ӷ���
//		return;
//	else
//	{
//		switch(tag)
//		{
//			case 0:
//				memcpy(&bullet_str[5], "Normal", 6);
//				break;
//			case 1:
//				memcpy(&bullet_str[5], "Low   ", 6);
//				break;
//			case 2:
//				memcpy(&bullet_str[5], "No!!  ", 6);
//				break;			
//		}
//		
//		repeat_cnt = 3;													//�ظ���������
//		while(repeat_cnt--)
//			Draw_Char(_layer, start_x, start_y, bullet_name, bullet_str, sizeof(bullet_str), size, _color, MODIFY_PICTURE);
//		
//		last_tag = tag;
//	}
//}

///**
// * @brief ���Զ���ͼ�㡿���ƻ��ƣ�����������5��
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::Armor(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t armor_tag, colorType_e _color, drawOperate_e _operate_type)
//{
//	uint8_t armor_name[] = "9Aa";				//a-e:��һ�����ƣ�f-j���ڶ������ƣ��Դ����ƣ������Ի��������
//	repeat_cnt = 3;								//�ظ�����������5��ͼ�εİ�������ѡ���ظ���3�ΰ������ϵײ�ÿ�����ظ�3���Ե���2/3��ƽ��������
//	/* ���ƻ��� */

//	if(armor_tag > 5)							//��չ����д��ĸ����֧�ֻ���10������
//	{
//		armor_name[2] -= 0x20;
//		armor_tag -= 5;
//	}

//	armor_name[2] += armor_tag*5;			

//	memcpy(data_pack, (uint8_t*)line_drawing(_layer, _operate_type, center_x - size/2, center_y + size/2, center_x + size/2, center_y + size/2, size/10, _color, armor_name), DRAWING_PACK);
//	armor_name[2]++;							//������
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, _operate_type, center_x - size/2, center_y + size/2, center_x - size/2, center_y - size/2, size/10, _color, armor_name), DRAWING_PACK);
//	armor_name[2]++;							//������
//	memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer, _operate_type, center_x + size/2, center_y + size/2, center_x + size/2, center_y - size/2, size/10, _color, armor_name), DRAWING_PACK);
//	armor_name[2]++;							//��б��
//	memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)line_drawing(_layer, _operate_type, center_x - size/2, center_y - size/2, center_x, center_y - size*3/4, size/10, _color, armor_name), DRAWING_PACK);
//	armor_name[2]++;							//��б��
//	memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)line_drawing(_layer, _operate_type, center_x + size/2, center_y - size/2, center_x, center_y - size*3/4, size/10, _color, armor_name), DRAWING_PACK);

//	while(repeat_cnt--)
//		pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);
//}

///**
// * @brief ���Զ���ͼ�㡿�����ƣ�����������13��
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::Sword(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t sword_tag, colorType_e _color, drawOperate_e _operate_type)
//{
//	uint8_t sword_name[] = "9Sa";				//a-b:��һ�ѽ���c-d���ڶ��ѽ����Դ����ƣ������Ի�ʮ���ѽ�
//	repeat_cnt = 3;								//�ظ�����������2��ͼ�εİ�������ѡ���ظ���2�ΰ������ϵײ�ÿ�����ظ�3���Ե���2/3��ƽ��������
//	
//	/* ������ */
//	sword_name[2] += sword_tag*2;				//���У����µ�����
//	memcpy(data_pack, (uint8_t*)line_drawing(_layer, _operate_type, center_x - size/2, center_y - size/2, center_x + size/2, center_y + size/2, size/10, _color, sword_name), DRAWING_PACK);
//	sword_name[2]++;							//���������ϵ�����
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, _operate_type, center_x - size/2, center_y, center_x, center_y - size/2, size/10, _color, sword_name), DRAWING_PACK);
//	
//	while(repeat_cnt--)
//		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
//}

///**
// * @brief ���Զ���ͼ�㡿ǰ��վͼ����ƣ�����������5��
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::Outpost_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type)
//{
//	uint8_t outpost_name[] = "9oa";												//ǰ��վ��outpost
//	repeat_cnt = 3;																//�ظ�����������5��ͼ�εİ�������ѡ���ظ���3�ΰ������ϵײ�ÿ�����ظ�3���Ե���2/3��ƽ��������

//	//ָ�����Ƶڼ���ǰ��վͼ��
//	outpost_name[2] += 5*tag;

//	//���ߣ�����								
//	memcpy(data_pack, (uint8_t*)line_drawing(_layer, _operate_type, center_x - size/4, center_y - size/2, center_x + size/4, center_y - size/2, size/10, _color, outpost_name), DRAWING_PACK);
//	outpost_name[2]++;															//��б�ߣ����µ�����
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, _operate_type, center_x - size/4, center_y - size/2, center_x - size/8, center_y + size/2, size/10, _color, outpost_name), DRAWING_PACK);
//	outpost_name[2]++;															//��б�ߣ����µ�����
//	memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer, _operate_type, center_x + size/4, center_y - size/2, center_x + size/8, center_y + size/2, size/10, _color, outpost_name), DRAWING_PACK);
//	outpost_name[2]++;															//���ߣ�����
//	memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)line_drawing(_layer, _operate_type, center_x - size/8, center_y + size/2, center_x + size/8, center_y + size/2, size/10, _color, outpost_name), DRAWING_PACK);
//	outpost_name[2]++;															//���ĵ�
//	memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)circle_drawing(_layer, _operate_type, center_x, center_y + size/4 , size/16, size/16, _color, outpost_name), DRAWING_PACK);

//	while(repeat_cnt--)
//		pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);
//}

///**
// * @brief ���Զ���ͼ�㡿�ڱ�ͼ����ƣ�����������5��
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::Sentry_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type)
//{
//	uint8_t sentry_name[] = "9sa";												//�ڱ���sentry
//	repeat_cnt = 3;																//�ظ�����������5��ͼ�εİ�������ѡ���ظ���3�ΰ������ϵײ�ÿ�����ظ�3���Ե���2/3��ƽ��������
//	
//	//ָ�����Ƶڼ���ͼ��
//	sentry_name[2] += 5*tag;

//	//������б�ߣ����ϵ�����							
//	memcpy(data_pack, (uint8_t*)line_drawing(_layer, _operate_type, center_x-size/2, center_y+size/2, center_x-size/4, center_y+size/4, size/10, _color, sentry_name), DRAWING_PACK);
//	sentry_name[2]++;							//������б�ߣ����µ�����
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, _operate_type, center_x+size/4, center_y+size/4, center_x+size/2, center_y+size/2, size/10, _color, sentry_name), DRAWING_PACK);
//	sentry_name[2]++;							//�������ߣ�����
//	memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer, _operate_type, center_x-size/4, center_y+size/4, center_x+size/4, center_y+size/4, size/10, _color, sentry_name), DRAWING_PACK);
//	sentry_name[2]++;							//yaw֧�ܣ��ϵ���
//	memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y+size/4, center_x, center_y-size/4, size/10, _color, sentry_name), DRAWING_PACK);
//	sentry_name[2]++;							//pitch֧�ܣ����µ�����
//	memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)line_drawing(_layer, _operate_type, center_x-size/2, center_y-size/2, center_x+size/2, center_y, size/10, _color, sentry_name), DRAWING_PACK);
//	
//	while(repeat_cnt--)
//		pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);
//}

///**
// * @brief ���Զ���ͼ�㡿����ͼ����ƣ�����������3��
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::Missle_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type)
//{
//	uint8_t missle_name[] = "9ma";
//	repeat_cnt = 5;																//�ظ�����������7��ͼ�εİ�������ѡ���ظ���5�ΰ������ϵײ�ÿ�����ظ�3���Ե���2/3��ƽ��������
//	
//	//ָ�����Ƶڼ���ͼ��
//	missle_name[2] += 7*tag;

//	//����ͷ
//	memcpy(data_pack, (uint8_t*)line_drawing(_layer, _operate_type, center_x+size/4, center_y+size/2, center_x+size/2, center_y+size/2, size/10, _color, missle_name), DRAWING_PACK);
//	missle_name[2]++;
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, _operate_type, center_x+size/2, center_y+size/2, center_x+size/2, center_y+size/4, size/10, _color, missle_name), DRAWING_PACK);

//	//������
//	missle_name[2]++;
//	memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer, _operate_type, center_x-size/4, center_y, center_x+size/4, center_y+size/2, size/10, _color, missle_name), DRAWING_PACK);
//	missle_name[2]++;
//	memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y-size/4, center_x+size/2, center_y+size/4, size/10, _color, missle_name), DRAWING_PACK);
//	//����β
//	missle_name[2]++;
//	memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)line_drawing(_layer, _operate_type, center_x-size/2, center_y, center_x-size/4, center_y, size/10, _color, missle_name), DRAWING_PACK);
//	missle_name[2]++;
//	memcpy(&data_pack[DRAWING_PACK*5], (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y-size/4, center_x, center_y-size/2, size/10, _color, missle_name), DRAWING_PACK);

//	//����б��
//	missle_name[2]++;
//	memcpy(&data_pack[DRAWING_PACK*6], (uint8_t*)line_drawing(_layer, _operate_type, center_x-size/4, center_y-size/4, center_x+size/4, center_y+size/4, size/10, _color, missle_name), DRAWING_PACK);

//	while(repeat_cnt--)
//		pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
//}

///**
// * @brief ���Զ���ͼ�㡿�ߵط���ͼ����ƣ�����������3��
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::High_Land(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type)
//{
//	uint8_t highland_name[] = "9ha";											//�ߵأ�highland
//	uint16_t c_point[4][2];														//�洢�ߵ�һ�������ֱ�߶������
//	uint8_t i;
//	
//	repeat_cnt = 5;																//�ظ�����������7��ͼ�εİ�������ѡ���ظ���5�ΰ������ϵײ�ÿ�����ظ�3���Ե���2/3��ƽ��������
//	
//	c_point[0][0] = center_x;
//	c_point[0][1] = center_y+size/2;	
//	c_point[1][0] = center_x-size/4;
//	c_point[1][1] = center_y;	
//	c_point[2][0] = center_x-size/4;
//	c_point[2][1] = center_y-size/4;	
//	c_point[3][0] = center_x;
//	c_point[3][1] = center_y-size/2;

//	//ָ�����Ƶڼ���ͼ��
//	highland_name[2] += 7*tag;

//	//�ߵ������							
//	for(i = 0;i < 3;i++)
//	{
//		highland_name[2]++;
//		memcpy(&data_pack[DRAWING_PACK*i], (uint8_t*)line_drawing(_layer, _operate_type, c_point[i][0], c_point[i][1], c_point[i+1][0], c_point[i+1][1], size/10, _color, highland_name), DRAWING_PACK);
//	}
//		
//	//�ߵ��Ҳ���
//	for(uint8_t j = 0;j < 4;j++)												//�ߵ������ƽ�ƣ���Ϊ�Ҳ���
//		c_point[j][0] += size/2;

//	for(i = 0;i < 3;i++)
//	{
//		highland_name[2]++;
//		memcpy(&data_pack[DRAWING_PACK*(i+3)], (uint8_t*)line_drawing(_layer, _operate_type, c_point[i][0], c_point[i][1], c_point[i+1][0], c_point[i+1][1], size/10, _color, highland_name), DRAWING_PACK);
//	}

//	//���ˣ����ĵ�
//	highland_name[2]++;											
//	memcpy(&data_pack[DRAWING_PACK*6], (uint8_t*)circle_drawing(_layer, _operate_type, center_x, center_y, size/16, size/16, _color, highland_name), DRAWING_PACK);

//	while(repeat_cnt--)
//		pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
//}

///**
// * @brief ���Զ���ͼ�㡿���ͼ����ƣ�����������5��
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// * @note ���� Referee.Windmill_Icon(7,960,300,80,0,PINK,ADD_PICTURE);
// */
//void referee_Classdef::Windmill_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type)
//{
//	static uint8_t windmill_state = 0;
//	static uint8_t windmill_last_state = 0;
//	static uint8_t modify_cnt[2] = {0,0};

//	uint8_t windmill_name[] = "9wa";								//�糵��w

//	//--- ��ΪRMUC
//	if(GameState.game_type != RMUC)
//	{
//		return;
//	}

//	repeat_cnt = 3;	//�ظ�����������5��ͼ�εİ�������ѡ���ظ���3�ΰ������ϵײ�ÿ�����ظ�3���Ե���2/3��ƽ��������
//	//ָ�����Ƶڼ���ͼ��
//	windmill_name[2] += 5*tag;

//	//---С�� ��� ��ǰ15s
//	if((GameState.stage_remain_time >= 240 && GameState.stage_remain_time <= 375) || GameState.stage_remain_time <= 195)
//	{
//		windmill_state = true;
//	}
//	else
//	{
//		windmill_state = false;
//	}

//	/* �豸���� */
//	if(windmill_state == true && (!windmill_last_state))      //�ź������أ���λ��Ʊ�־
//	{
//		modify_cnt[0] = 3;
//		modify_cnt[1] = 0;
//	}
//	else if(windmill_state == false && (windmill_last_state))  //�ź��½��أ����ɾ����־
//	{
//		modify_cnt[1] = 3;
//		modify_cnt[0] = 0;
//	}
//	else
//	{}

//	if(modify_cnt[0])
//	{
//		modify_cnt[0]--;

//		//˳ʱ�������Ҷ���Ӷ�Ҷ��ʼ							
//		memcpy(data_pack, (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y, center_x, center_y+size/2, size/5, _color, windmill_name), DRAWING_PACK);
//		windmill_name[2]++;						
//		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y, center_x+size/2, center_y+size/8, size/5, _color, windmill_name), DRAWING_PACK);
//		windmill_name[2]++;						
//		memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y, center_x+size/3, center_y-size/2, size/5, _color, windmill_name), DRAWING_PACK);
//		windmill_name[2]++;							
//		memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y, center_x-size/3, center_y-size/2, size/5, _color, windmill_name), DRAWING_PACK);
//		windmill_name[2]++;						
//		memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y, center_x-size/2, center_y+size/8, size/5, _color, windmill_name), DRAWING_PACK);
//		//ԲȦ
//		windmill_name[2]++;
//		memcpy(&data_pack[DRAWING_PACK*5], (uint8_t*)circle_drawing(_layer, _operate_type, center_x, center_y, size/2+20, 8, _color, windmill_name), DRAWING_PACK);
//		
//		pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*6);

//	}
//	else if(modify_cnt[1])
//	{
//		modify_cnt[1]--;

//		//˳ʱ�������Ҷ���Ӷ�Ҷ��ʼ							
//		memcpy(data_pack, (uint8_t*)line_drawing(_layer, CLEAR_ONE_PICTURE, center_x, center_y, center_x, center_y+size/2, size/5, _color, windmill_name), DRAWING_PACK);
//		windmill_name[2]++;						
//		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, CLEAR_ONE_PICTURE, center_x, center_y, center_x+size/2, center_y+size/8, size/5, _color, windmill_name), DRAWING_PACK);
//		windmill_name[2]++;						
//		memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer, CLEAR_ONE_PICTURE, center_x, center_y, center_x+size/3, center_y-size/2, size/5, _color, windmill_name), DRAWING_PACK);
//		windmill_name[2]++;							
//		memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)line_drawing(_layer, CLEAR_ONE_PICTURE, center_x, center_y, center_x-size/3, center_y-size/2, size/5, _color, windmill_name), DRAWING_PACK);
//		windmill_name[2]++;						
//		memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)line_drawing(_layer, CLEAR_ONE_PICTURE, center_x, center_y, center_x-size/2, center_y+size/8, size/5, _color, windmill_name), DRAWING_PACK);
//		//ԲȦ
//		windmill_name[2]++;
//		memcpy(&data_pack[DRAWING_PACK*5], (uint8_t*)circle_drawing(_layer, CLEAR_ONE_PICTURE, center_x, center_y, size/2+20, 8, _color, windmill_name), DRAWING_PACK);
//		
//		pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*6);
//		
//	}


//	// //˳ʱ�������Ҷ���Ӷ�Ҷ��ʼ							
//	// memcpy(data_pack, (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y, center_x, center_y+size/2, size/5, _color, windmill_name), DRAWING_PACK);
//	// windmill_name[2]++;						
//	// memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y, center_x+size/2, center_y+size/8, size/5, _color, windmill_name), DRAWING_PACK);
//	// windmill_name[2]++;						
//	// memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y, center_x+size/3, center_y-size/2, size/5, _color, windmill_name), DRAWING_PACK);
//	// windmill_name[2]++;							
//	// memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y, center_x-size/3, center_y-size/2, size/5, _color, windmill_name), DRAWING_PACK);
//	// windmill_name[2]++;						
//	// memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)line_drawing(_layer, _operate_type, center_x, center_y, center_x-size/2, center_y+size/8, size/5, _color, windmill_name), DRAWING_PACK);

//	// windmill_name[2]++;
//	// memcpy(&data_pack[DRAWING_PACK*5], (uint8_t*)circle_drawing(_layer, _operate_type, center_x, center_y, size/2+20, 8, _color, windmill_name), DRAWING_PACK);
//	
//	// while(repeat_cnt--)
//	// 	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*6);


//	windmill_last_state = windmill_state;
//}				

///**
// * @brief ���Զ���ͼ�㡿����ͼ����ƣ�����������5��
// * @note ��ͼ������Ļ�������name�ظ���
// * @note �������UI�����ʴ󣬹ʺ����ڲ��������ظ��������ԣ�ȷ�����ݰ�һ���ܷ��͵��ͻ���
// */
//void referee_Classdef::FlyingSlope_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type)
//{
//	uint8_t flying_name[] = "9fa";												//���£�flying
//	repeat_cnt = 3;																//�ظ�����������5��ͼ�εİ�������ѡ���ظ���3�ΰ������ϵײ�ÿ�����ظ�3���Ե���2/3��ƽ��������
//	
//	//ָ�����Ƶڼ���ͼ��
//	flying_name[2] += 5*tag;

//	//������						
//	memcpy(data_pack, (uint8_t*)line_drawing(_layer, _operate_type, center_x-size/2, center_y-size/2, center_x+size/2, center_y-size/2, size/10, _color, flying_name), DRAWING_PACK);
//	flying_name[2]++;						
//	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, _operate_type, center_x-size/2, center_y-size/2, center_x+size/2, center_y, size/10, _color, flying_name), DRAWING_PACK);
//	flying_name[2]++;						
//	//���Ƽ�ͷ
//	memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer, _operate_type, center_x-size/4, center_y, center_x+size/4, center_y+size/4, size/10, _color, flying_name), DRAWING_PACK);
//	flying_name[2]++;							//�Ӽ�ͷ�׳�����б��
//	memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)line_drawing(_layer, _operate_type, center_x+size/4, center_y+size/4, center_x+size/8, center_y+size/4, size/10, _color, flying_name), DRAWING_PACK);
//	flying_name[2]++;						
//	memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)line_drawing(_layer, _operate_type, center_x+size/4, center_y+size/4, center_x+size*3.0/16.0, center_y+size/8, size/10, _color, flying_name), DRAWING_PACK);
//	
//	while(repeat_cnt--)
//		pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);

//}
//		
///**
// * @brief ���Զ���ͼ�㡿���Ʋ���UI�Ŀ�ܣ�Event��ʾ�ֺ�Suggestion��ʾ��
// * @note ��ͼ������Ļ�������name�ظ���
// * @note 
// */
//void referee_Classdef::Radar_Strategy_Frame(uint16_t *frame_pos_x, uint16_t *frame_pos_y)
//{
//	uint8_t frame_name[] = "9e0";
//	uint8_t event_str[] = "Events:";
//	uint8_t suggestion_str[] = "Suggestions:";

//	/* ������ʾ�� */
//	Draw_Char(9, frame_pos_x[0], frame_pos_y[0], frame_name, event_str, sizeof(event_str), 20, YELLOW, ADD_PICTURE);
//	frame_name[2] += 1;
//	Draw_Char(9, frame_pos_x[1], frame_pos_y[1], frame_name, suggestion_str, sizeof(suggestion_str), 20, YELLOW, ADD_PICTURE);
//}

///**
// * @brief ��ͼ��9�����Ʋ���UI��ͨ�ö�̬��ʶ��ǰ�ڡ��ڱ������ڡ��ߵأ����Լ�����-������ʶ
// * @note ��ͼ������Ļ�������name�ظ���
// * @note ע�⣬��UIֻ���״�վ���Է����仯ʱռ�ô���
// */
//void referee_Classdef::Radar_CStrategy_Update(uint8_t protect, uint8_t attack, uint8_t comment_startegy, uint16_t *pos_x, uint16_t *pos_y)
//{	
//	static uint8_t last_protect = 0;
//	static uint8_t last_attack = 0;
//	static uint8_t last_comment_startegy = 0;
//	
//	uint8_t i;													//��������
//	uint8_t dir;												//�������Ʒ�����Ϊ�ͻ��˺췽�����˺������������Ǿ���Գ����е�
//	uint16_t protect_robots_x;									//��Ҫ����UI�Ļ�����ͼ���Ӧ����
//	uint16_t attack_robots_x;									//��Ҫ����UI�Ļ�����ͼ���Ӧ����
//	
//	/* ����-����ͼ����� */
//	//��������
//	if(robot_client_ID.robot_where)								//����Ϊ���������趨����UI������Ϊ��������
//	{
//		protect_robots_x = 1230;								//1230Ϊ�ұ�Ӣ��,dir = 1
//		attack_robots_x = 710;									//710Ϊ���Ӣ��,dir = 0
//	}
//	else														//�췽Ϊ����
//	{
//		protect_robots_x = 710;
//		attack_robots_x = 1230;
//	}

//	
//	//���ر�ʶ����
//	robot_client_ID.robot_where? dir = 1:-1;
//	
//	for(i = 0;i < 5;i++)
//	{
//		if(((protect >> i) % 2) && !((last_protect >> i) % 2))	//�ܱ��������б䶯����Ϊ���ӵı䶯��������ͼ�����
//			Armor(9, protect_robots_x+(dir)*i*120, 860, 40, i, WHITE, ADD_PICTURE);
//		else if(!((protect >> i) % 2) && ((last_protect >> i) % 2))
//			Armor(9, protect_robots_x+(dir)*i*120, 860, 40, i, WHITE, CLEAR_ONE_PICTURE);
//	}

//	//������ʶ����
//		robot_client_ID.robot_where? dir = -1:1;
//	
//	for(i = 0;i < 5;i++)
//	{
//		if(((attack >> i) % 2) && !((last_attack >> i) % 2))	//�ܱ��������б䶯����Ϊ���ӵı䶯��������ͼ�����
//			Sword(9, attack_robots_x+(dir)*i*120, 860, 40, i, WHITE, ADD_PICTURE);
//		else if(!((attack >> i) % 2) && ((last_attack >> i) % 2))
//			Sword(9, attack_robots_x+(dir)*i*120, 860, 40, i, WHITE, CLEAR_ONE_PICTURE);
//	}

//	/* ͨ�ò�������� */
//	for(i = 0;i < 4;i++)
//	{	
//		if(((comment_startegy >> i) % 2) && !((last_comment_startegy >> i) % 2))						//�������ӣ������ͼ��
//		{
//			switch(i){
//				case 0:Outpost_Icon(9, pos_x[0], pos_y[0], 40, 0, PINK, ADD_PICTURE);break;
//				case 1:Sentry_Icon(9, pos_x[1], pos_y[1], 40, 0, PINK, ADD_PICTURE);break;
//				case 2:Missle_Icon(9, pos_x[2], pos_y[2], 40, 0, PINK, ADD_PICTURE);break;
//				case 3:High_Land(9, pos_x[3], pos_y[3], 40, 0, PINK, ADD_PICTURE);break;
//				default:break;
//			};
//		}
//		else if(!((comment_startegy >> i) % 2) && ((last_comment_startegy >> i) % 2))					//���Լ��٣�ɾ��ͼ��
//		{
//			switch(i){
//				case 0:Outpost_Icon(9, pos_x[0], pos_y[0], 40, 0, PINK, CLEAR_ONE_PICTURE);break;
//				case 1:Sentry_Icon(9, pos_x[1], pos_y[1], 40, 0, PINK, CLEAR_ONE_PICTURE);break;
//				case 2:Missle_Icon(9, pos_x[2], pos_y[2], 40, 0, PINK, CLEAR_ONE_PICTURE);break;
//				case 3:High_Land(9, pos_x[3], pos_y[3], 40, 0, PINK, CLEAR_ONE_PICTURE);break;
//				default:break;
//			};
//		}		
//	}
//	
//	/* ���¸��ֱ�־λ */
//	last_protect = protect;
//	last_attack = attack;
//	last_comment_startegy = comment_startegy;
//}

///**
// * @brief ���Զ���ͼ�㡿���ݵ�ǰ�ĳ���ID�����Ʋ���UI��ר�ö�̬��ʶ�������ַ���
// * @note ��ͼ������Ļ�������name�ظ���
// * @note ������Ҫ����ר�ñ�ʶ�������øú�������
// */
//void referee_Classdef::Radar_SStrategy_Update(uint16_t special_startegy, uint16_t *pos_x, uint16_t *pos_y)
//{
//	static uint16_t last_special_startegy = special_startegy;

//	uint8_t just_kill[] = "just kill!";				//��Ѫ��ʾ
//	uint8_t kill_name[] = "9k0";

//	uint16_t mask = 0x01;							//���룬���ڹ��˳��������ʵ�λ
//	uint8_t i;										//��������
//	drawOperate_e drawing_type;						//����״̬�����ͼ�� or ɾ��ͼ��

//	/* ����Ӣ��ר��UI���������� ǰ�ڽ��� ��Ѫע�� */
//	if(robot_client_ID.local == robot_client_ID.hero)
//	{
//		for(i = 0;i < 4;i++)
//		{
//			if((last_special_startegy^special_startegy) & mask)
//			{
//				if(special_startegy & mask)						//�����أ������ͼ��
//					drawing_type = ADD_PICTURE;
//				else											//�½��أ���ɾ��ͼ��
//					drawing_type = CLEAR_ONE_PICTURE;
//			}
//			else
//				continue;

//			switch (i)
//			{
//			case 0:				//��������
//				FlyingSlope_Icon(9, pos_x[0], pos_y[0], 40, 0, WHITE, drawing_type);		
//				break;
//			case 1:				//����ǰ��վ										
//				Sword(9, pos_x[1], pos_y[1], 40, 6, WHITE, drawing_type);					
//				Outpost_Icon(9, pos_x[1] + 40, pos_y[1], 40, 1, WHITE, drawing_type);
//				break;
//			case 2:				//��Ѫע��
//				repeat_cnt = 3;
//				while(repeat_cnt--)
//					Draw_Char(9, pos_x[2], pos_y[2], kill_name, just_kill, sizeof(just_kill), 20, WHITE, drawing_type);
//				break;
//			default:
//				break;
//			}
//			mask <<= 1;
//		}
//	}
//	/* ���ƹ���ר��UI������Դ�� ǰ��վ���� */
//	else if(robot_client_ID.local == robot_client_ID.engineer)
//	{
//		uint8_t money_str[] = "make money!";
//		uint8_t money_name[] = "9m0";

//		for(i = 0;i < 4;i++)
//		{
//			if((last_special_startegy^special_startegy) & mask)
//			{
//				if(special_startegy & mask)						//�����أ������ͼ��
//					drawing_type = ADD_PICTURE;
//				else											//�½��أ���ɾ��ͼ��
//					drawing_type = CLEAR_ONE_PICTURE;
//			}
//			else
//				continue;

//			switch (i)
//			{
//			case 0:				//ǰ��վ����
//				Armor(9, pos_x[0], pos_y[0], 40, 6, WHITE, drawing_type);					
//				Outpost_Icon(9, pos_x[0] + 40, pos_y[0], 40, 1, WHITE, drawing_type);				
//				break;
//						
//			case 1:				//����Դ��
//				repeat_cnt = 3;
//				while(repeat_cnt--)
//					Draw_Char(9, pos_x[1], pos_y[1], money_name, money_str, sizeof(money_str), 20, WHITE, drawing_type);
//				break;
//			default:
//				break;
//			}

//			mask <<= 1;
//		}
//	}
//	/* ���Ʋ���ר��UI�������ʾ ���ڱ� �ظߵ� ��Ѫע�� */
//	else
//	{
//		for(i = 0;i < 4;i++)
//		{
//			if((last_special_startegy^special_startegy) & mask)
//			{
//				if(special_startegy & mask)						//�����أ������ͼ��
//					drawing_type = ADD_PICTURE;
//				else											//�½��أ���ɾ��ͼ��
//					drawing_type = CLEAR_ONE_PICTURE;
//			}
//			else
//				continue;

//			switch (i)
//			{
//			case 0:				//�����ʾ
//				Windmill_Icon(9, pos_x[0], pos_y[0], 40, 0, GREEN, drawing_type);
//				break;
//			case 1:				//���ڱ�
//				Sword(9, pos_x[1], pos_y[1], 40, 6, WHITE, drawing_type);
//				Sentry_Icon(9, pos_x[1]+40, pos_y[1], 40, 1, WHITE, drawing_type);
//				break;
//			case 2:				//�ظߵ�
//				Armor(9, pos_x[2], pos_y[2], 40, 1, WHITE, drawing_type);
//				High_Land(9, pos_x[2]+40, pos_y[2], 40, 1, WHITE, drawing_type);
//				break;
//			case 3:				//��Ѫע��
//				repeat_cnt = 3;
//				while(repeat_cnt--)
//					Draw_Char(9, pos_x[3], pos_y[3], kill_name, just_kill, sizeof(just_kill), 20, WHITE, drawing_type);
//				break;
//			default:
//				break;
//			}
//			
//			mask <<= 1;
//		}
//	}

//	last_special_startegy = special_startegy;
//}
///************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
