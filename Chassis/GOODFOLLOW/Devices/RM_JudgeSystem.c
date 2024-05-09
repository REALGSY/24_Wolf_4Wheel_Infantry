/**
 ****************************************************************************************************
 * @file       RM_JudgeSystem.h
 * @brief      
 * @note       None
 * @autor      Autism_Huang
 * @Version    V1.0.0
 * @Date       2021.3.3     
 ****************************************************************************************************
 */
#include "RM_JudgeSystem.h"
#include "DR16_Remote.h"
#include "DEV_SupCap.h"
void JudgeSystem_USART_Receive_DMA(UART_HandleTypeDef *huartx);
void JudgeSystem_Handler(UART_HandleTypeDef *huart);
void Check_Judge(void);

JudgeSystem_FUN_t JudgeSystem_FUN = JudgeSystem_FUNGroundInit;
#undef JudgeSystem_FUNGroundInit
RM_Judge_t RM_Judge;
/****************************************自定义DMA-串口接收****************************************/
/**
 * @brief USART_DMA接收开启和重定向
 * 
 * @param huart 
 * @param pData 
 * @param Size 
 * @return  
 */
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{

    /*检测当前huart状态*/
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        /*输入的地址或者数据有问题的话*/
        if ((pData == NULL) || (Size == 0))
        {
            return HAL_ERROR;
        }

        /*huart里面对应的Rx变量重定向*/
        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        /*开启huart1上的RX_DMA*/
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

        /*只开启对应DMA上面的Rx功能（如果是开启Tx的话就是USART_CR3_DMAT）*/
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    }
    else
    {
        return HAL_BUSY;
    }

    return HAL_OK;
}

/**
  * @Data    2021/3/26
  * @brief   裁判系统处理函数 
  * @param   UART_HandleTypeDef *huart
  * @retval  void
  */
void JudgeSystem_USART_Receive_DMA(UART_HandleTypeDef *huartx)
{
    __HAL_UART_CLEAR_IDLEFLAG(huartx);
    __HAL_UART_ENABLE(huartx);
    __HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);
    // assert(JUDGESYSTEM_PACKSIZE == 341u);
    USART_Receive_DMA_NO_IT(huartx, JudgeSystem_rxBuff, JUDGESYSTEM_PACKSIZE);
}

uint16_t DMA_Counter;
void JudgeSystem_Handler(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_DMA_DISABLE(huart->hdmarx);

    DMA_Counter = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    Judge_GetMessage(JUDGESYSTEM_PACKSIZE - DMA_Counter);

    __HAL_DMA_SET_COUNTER(huart->hdmarx, JUDGESYSTEM_PACKSIZE);
    __HAL_DMA_ENABLE(huart->hdmarx);

    RM_Judge.InfoUpdateFrame++;
}

/****************************************2020赛季裁判系统******************************************/
#if JUDGE_VERSION == JUDGE_20

#define Judge_SendDataLengthPC 28
#define Judge_SendDataLengthROBOT 9

RM_Judge_t RM_Judge; //裁判系统数据对象。

uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE]; //接收buff

ext_game_state_t ext_game_state;
ext_game_result_t ext_game_result;
ext_game_robot_HP_t ext_game_robot_HP;
ext_dart_status_t ext_dart_status;
ext_ICRA_buff_debuff_zone_status_t ext_ICRA_buff_debuff_zone_status;
ext_supply_projectile_action_t ext_supply_projectile_action;
ext_referee_warning_t ext_referee_warning;
ext_dart_remaining_time_t ext_dart_remaining_time;
ext_game_robot_state_t ext_game_robot_state;
ext_power_heat_data_t ext_power_heat_data;
ext_game_robot_pos_t ext_game_robot_pos;
ext_buff_musk_t ext_buff_musk;
aerial_robot_energy_t aerial_robot_energy;
ext_robot_hurt_t ext_robot_hurt;
ext_shoot_data_t ext_shoot_data;
ext_bullet_remaining_t ext_bullet_remaining;
ext_rfid_status_t ext_rfid_status;
ext_dart_client_cmd_t ext_dart_client_cmd;
ext_CommunatianData_t CommuData; //队友通信信息
ext_SendClientData_t ShowData;   //客户端信息

Judge_Monitor_t Judge_Monitor;
Judge_SendPC_t Judge_SendPC;
Judge_SendRobot_t Judge_SendRobot;

uint8_t Judge_Self_ID;        //当前机器人的ID
uint16_t Judge_SelfClient_ID; //发送者机器人对应的客户端ID

/***********************************   DJI提供的CRC校检函数  Begin  ***********************************/
uint16_t wExpected;

//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
    {
        0x00,
        0x5e,
        0xbc,
        0xe2,
        0x61,
        0x3f,
        0xdd,
        0x83,
        0xc2,
        0x9c,
        0x7e,
        0x20,
        0xa3,
        0xfd,
        0x1f,
        0x41,
        0x9d,
        0xc3,
        0x21,
        0x7f,
        0xfc,
        0xa2,
        0x40,
        0x1e,
        0x5f,
        0x01,
        0xe3,
        0xbd,
        0x3e,
        0x60,
        0x82,
        0xdc,
        0x23,
        0x7d,
        0x9f,
        0xc1,
        0x42,
        0x1c,
        0xfe,
        0xa0,
        0xe1,
        0xbf,
        0x5d,
        0x03,
        0x80,
        0xde,
        0x3c,
        0x62,
        0xbe,
        0xe0,
        0x02,
        0x5c,
        0xdf,
        0x81,
        0x63,
        0x3d,
        0x7c,
        0x22,
        0xc0,
        0x9e,
        0x1d,
        0x43,
        0xa1,
        0xff,
        0x46,
        0x18,
        0xfa,
        0xa4,
        0x27,
        0x79,
        0x9b,
        0xc5,
        0x84,
        0xda,
        0x38,
        0x66,
        0xe5,
        0xbb,
        0x59,
        0x07,
        0xdb,
        0x85,
        0x67,
        0x39,
        0xba,
        0xe4,
        0x06,
        0x58,
        0x19,
        0x47,
        0xa5,
        0xfb,
        0x78,
        0x26,
        0xc4,
        0x9a,
        0x65,
        0x3b,
        0xd9,
        0x87,
        0x04,
        0x5a,
        0xb8,
        0xe6,
        0xa7,
        0xf9,
        0x1b,
        0x45,
        0xc6,
        0x98,
        0x7a,
        0x24,
        0xf8,
        0xa6,
        0x44,
        0x1a,
        0x99,
        0xc7,
        0x25,
        0x7b,
        0x3a,
        0x64,
        0x86,
        0xd8,
        0x5b,
        0x05,
        0xe7,
        0xb9,
        0x8c,
        0xd2,
        0x30,
        0x6e,
        0xed,
        0xb3,
        0x51,
        0x0f,
        0x4e,
        0x10,
        0xf2,
        0xac,
        0x2f,
        0x71,
        0x93,
        0xcd,
        0x11,
        0x4f,
        0xad,
        0xf3,
        0x70,
        0x2e,
        0xcc,
        0x92,
        0xd3,
        0x8d,
        0x6f,
        0x31,
        0xb2,
        0xec,
        0x0e,
        0x50,
        0xaf,
        0xf1,
        0x13,
        0x4d,
        0xce,
        0x90,
        0x72,
        0x2c,
        0x6d,
        0x33,
        0xd1,
        0x8f,
        0x0c,
        0x52,
        0xb0,
        0xee,
        0x32,
        0x6c,
        0x8e,
        0xd0,
        0x53,
        0x0d,
        0xef,
        0xb1,
        0xf0,
        0xae,
        0x4c,
        0x12,
        0x91,
        0xcf,
        0x2d,
        0x73,
        0xca,
        0x94,
        0x76,
        0x28,
        0xab,
        0xf5,
        0x17,
        0x49,
        0x08,
        0x56,
        0xb4,
        0xea,
        0x69,
        0x37,
        0xd5,
        0x8b,
        0x57,
        0x09,
        0xeb,
        0xb5,
        0x36,
        0x68,
        0x8a,
        0xd4,
        0x95,
        0xcb,
        0x29,
        0x77,
        0xf4,
        0xaa,
        0x48,
        0x16,
        0xe9,
        0xb7,
        0x55,
        0x0b,
        0x88,
        0xd6,
        0x34,
        0x6a,
        0x2b,
        0x75,
        0x97,
        0xc9,
        0x4a,
        0x14,
        0xf6,
        0xa8,
        0x74,
        0x2a,
        0xc8,
        0x96,
        0x15,
        0x4b,
        0xa9,
        0xf7,
        0xb6,
        0xe8,
        0x0a,
        0x54,
        0xd7,
        0x89,
        0x6b,
        0x35,
};

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return;
    ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;

const uint16_t wCRC_Table[256] =
    {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while (dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum((uint8_t *)pchMessage, dwLength - 2, CRC_INIT);
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}
/***********************************  DJI提供的CRC校检函数 END  ***********************************/

/**
  * @brief  裁判系统初始化
  * @param  None
  * @retval None
  */
//void JudgeSystem_Init(void) {
////	USART3_QuickInit(115200);
////	USART3_RXDMA_Config((uint32_t)JudgeSystem_rxBuff, JUDGESYSTEM_PACKSIZE);
//}

/**
  * @brief  获取裁判系统信息
  * @param  dataLength 	获取到的数据长度
  * @retval None
  */
void Judge_getInfo(uint16_t dataLength)
{
    for (int n = 0; n < dataLength;)
    {
        if (JudgeSystem_rxBuff[n] == JUDGESYSTEM_FRAMEHEADER) //如果buff=帧头 0xA5
        {
            switch (JudgeSystem_rxBuff[n + 5] | JudgeSystem_rxBuff[n + 6] << 8)
            {

            case JudgeInfoType_Game_state: //比赛数据状态  0x0001
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Game_state))
                {
                    memcpy(ext_game_state.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[3]));
                    n += JudgeInfoLength_Game_state;
                    ext_game_state.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Game_result: //比赛结果  0x0002
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Game_result))
                {
                    memcpy(ext_game_result.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeInfoLength_Game_result;
                    ext_game_result.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Game_robot_HP: //机器人血量数据 0x0003
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Game_robot_HP))
                {
                    memcpy(ext_game_robot_HP.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[32]));
                    n += JudgeInfoLength_Game_robot_HP;
                    ext_game_robot_HP.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Dart_status: //飞镖发射状态  0x0004
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Dart_status))
                {
                    memcpy(ext_dart_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[3]));
                    n += JudgeInfoLength_Dart_status;
                    ext_dart_status.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_ICRA_buff: //人工智能挑战赛加成与惩罚区状态  0x0005
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_ICRA_buff))
                {
                    memcpy(ext_ICRA_buff_debuff_zone_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[3]));
                    n += JudgeInfoLength_ICRA_buff;
                    ext_ICRA_buff_debuff_zone_status.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Event_data: //场地事件数据  0x0101
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Event_data))
                {
                    memcpy(ext_dart_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[4]));
                    n += JudgeInfoLength_Event_data;
                    ext_dart_status.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Supply_projectile_action: //补给站动作标识数据  0x0102
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Supply_projectile_action))
                {
                    memcpy(ext_supply_projectile_action.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[4]));
                    n += JudgeInfoLength_Supply_projectile_action;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Referee_warning: //裁判警告数据    0x0104
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Referee_warning))
                {
                    memcpy(ext_referee_warning.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[2]));
                    n += JudgeInfoLength_Referee_warning;
                    ext_referee_warning.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Dart_remaining_time: //飞镖发射口倒计时    0x0105
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Dart_remaining_time))
                {
                    memcpy(ext_referee_warning.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeInfoLength_Dart_remaining_time;
                    ext_referee_warning.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Game_robot_state: //机器人状态数据 0x0201
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Game_robot_state))
                {
                    memcpy(ext_game_robot_state.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[18]));
                    n += JudgeInfoLength_Game_robot_state;
                    ext_game_robot_state.infoUpdateFlag = 1;
                    /*************************************************更新等级数据********************************************/
                    //						Lever_Determine(ext_game_robot_state.data.robot_level);
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Power_heat_data: //实时功率热量数据 0x0202
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Power_heat_data))
                {
                    memcpy(ext_power_heat_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[16]));
                    n += JudgeInfoLength_Power_heat_data;
                    ext_power_heat_data.infoUpdateFlag = 1;
                    /*************************************************剩余底盘缓冲功率********************************************/
                    //              powerBufferPool_t.r_w = ext_power_heat_data.data.chassis_power_buffer;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Game_robot_pos: //机器人位置数据  0x0203
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Game_robot_pos))
                {
                    memcpy(ext_game_robot_pos.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[16]));
                    n += JudgeInfoLength_Game_robot_pos;
                    ext_game_robot_pos.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Buff_musk: //机器人增益数据  0x0204
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Buff_musk))
                {
                    memcpy(ext_buff_musk.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeInfoLength_Buff_musk;
                    ext_buff_musk.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Robot_energy: //空中机器人能量状态数据  0x0205
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Robot_energy))
                {
                    memcpy(aerial_robot_energy.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[3]));
                    n += JudgeInfoLength_Robot_energy;
                    aerial_robot_energy.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Robot_hurt: //伤害状态数据  0x0206
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Robot_hurt))
                {
                    memcpy(ext_robot_hurt.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeInfoLength_Robot_hurt;
                    ext_robot_hurt.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType_Shoot_data: //实时射击数据   0x0207
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Shoot_data))
                {
                    memcpy(ext_shoot_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[6]));
                    n += JudgeInfoLength_Shoot_data;
                    ext_shoot_data.infoUpdateFlag = 1;
                    //if (ext_shoot_data.data.bullet_type == 1)//小弹丸射击数据
                    //{
                    //	Shooting.realSmallSpeed = ext_shoot_data.data.bullet_speed;
                    //}
                    /*Debug_addData((float)shoot.Fric.output, 1);
					Debug_addData(ext_shoot_data.data.bullet_speed, 2);
					Debug_show(2);*/
                    //if(ext_shoot_data.data.bullet_type == 2)//大弹丸射击数据
                    //{
                    //  Shooting.realLargeSpeed = ext_shoot_data.data.bullet_speed;
                    //}
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType__Bullet_remaining: //弹丸剩余发射数  0x0208
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Bullet_remaining))
                {
                    memcpy(ext_bullet_remaining.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[2]));
                    n += JudgeInfoLength_Bullet_remaining;
                    ext_bullet_remaining.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType___Rfid_status: //RFID状态     0x0209
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Rfid_status))
                {
                    memcpy(ext_rfid_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[2]));
                    n += JudgeInfoLength_Rfid_status;
                    ext_rfid_status.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            case JudgeInfoType___Dart_client: //飞镖客户端指令数据     0x020A
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeInfoLength_Dart_client))
                {
                    memcpy(ext_dart_client_cmd.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[2]));
                    n += JudgeInfoLength_Dart_client;
                    ext_dart_client_cmd.infoUpdateFlag = 1;
                }
                else
                {
                    n++;
                }

                break;

            default:
                n++;
                break;
            }
        }
        else
        {
            n++;
        }
    }
    //帧率++
    Judge_Monitor.infoUpdateFrame++;
}

/**
  * @brief  向裁用户界面发送信息
  * @param
  * @retval None
  */
//void Judge_sendPC(float Userdata1, float Userdata2, float Userdata3, uint8_t Userdata4)
//{
//	static uint8_t counter = 0;
//	uint8_t sendbuff[Judge_SendDataLengthPC];
//	counter++;

//	/*--FrameHeader_Begin--*/
//	sendbuff[0] = 0xA5;			//帧起始字节 (1byte)
//	sendbuff[1] = 0x0D;			//数据段长度低位 13 (1byte)
//	sendbuff[2] = 0x00;			//数据段长度高位 13 (1byte)
//	sendbuff[3] = counter;		//包序号 (1byte)
//	sendbuff[4] = Get_CRC8_Check_Sum(sendbuff, 4, CRC8_INIT);		//帧头 CRC8 校验 (1byte)
//	/*--FrameHeader_End--*/

////交互数据接收信息：0x0301。发送频率：上限 10Hz
//	/*--CmdID_Begin--*/
//	sendbuff[5] = 0x01; //CmdID低位 (2byte)
//	sendbuff[6] = 0x03; //CmdID高位 (2byte)
//	/*--CmdID_End--*/

//	Judge_SendPC.data.data1 = Userdata1;
//	Judge_SendPC.data.data2 = Userdata2;
//	Judge_SendPC.data.data3 = Userdata3;
//	Judge_SendPC.data.mask = Userdata4;

//	memcpy(&sendbuff[7], Judge_SendPC.data.dataBuff, sizeof(uint8_t[13]));

//	//将CRC16校验添加到帧尾
//	Append_CRC16_Check_Sum(sendbuff, 22);  //FrameTail(2-Byte, CRC16, 整包校验 )
//	/*--FrameTail_End--*/

////	for (int i = 0; i < 22; i++) {
////		USART_sendChar(USART3, sendbuff[i]);
////	}

//}

/**
  * @brief  上传自定义数据
  * @param  void
  * @retval void
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
#define send_max_len 40 //200
unsigned char CliendTxBuffer[send_max_len];
void Judge_sendPC(void)
{
    static uint8_t datalength;

    determine_ID(); //判断发送者ID和其对应的客户端ID

    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_custom_data_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301;

    ShowData.dataFrameHeader.data_cmd_id = 0xD180; //发给客户端的cmd,官方固定
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.send_ID = Judge_Self_ID;           //发送者的ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID; //客户端的ID，只能为发送者机器人对应的客户端

    /*- 自定义内容 -*/
    //	ShowData.clientData.data1 = (float)Capvoltage_Percent();//电容剩余电量
    //	ShowData.clientData.data2 = (float)Base_Angle_Measure();//吊射角度测
    //	ShowData.clientData.data3 = GIMBAL_PITCH_Judge_Angle();//云台抬头角度

    //打包写入数据段
    memcpy(
        CliendTxBuffer + 5,
        (uint8_t *)&ShowData.CmdID,
        (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.clientData)));

    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码

    datalength = sizeof(ShowData);

    HAL_UART_Transmit(&huart3, CliendTxBuffer, datalength, 0xFF);
}

/**
* @brief  机器人之间互传信息
  * @param
  * @retval None
  */
#define Teammate_max_len 40 //200

unsigned char TeammateTxBuffer[Teammate_max_len];
bool Send_Color = 0;
//_Bool First_Time_Send_Commu = 0;
uint16_t send_time = 0;
//参数cmd是机器人信息交互的协议：在0x0200-0x02FF之间选择 (令0x02FF是无用数据id)
void Judge_sendRobot(uint16_t cmd)
{
    static uint8_t datalength;

    Send_Color = is_red_or_blue();   //判断发送给兵种的颜色,107哨兵(蓝),7哨兵(红)；103'g'步兵蓝  3步兵红
    memset(TeammateTxBuffer, 0, 40); //初始化TeammateTxBuffer
    CommuData.txFrameHeader.SOF = 0xA5;
    CommuData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
    CommuData.txFrameHeader.Seq = 0x00;
    memcpy(TeammateTxBuffer, &CommuData.txFrameHeader, sizeof(xFrameHeader));
    Append_CRC8_Check_Sum(TeammateTxBuffer, sizeof(xFrameHeader));

    CommuData.CmdID = 0x0301;
    CommuData.dataFrameHeader.send_ID = Judge_Self_ID;
    CommuData.dataFrameHeader.data_cmd_id = cmd; //裁判协议：在0x0200-0x02FF之间选择

    switch (cmd)
    {
    case 0x0255:
        //发给步兵
        Send_Infantry();
        CommuData.interactData.data[0] = '1'; //发送的内容  注意大小不可以超过变量的变量类型
        break;
    case 0x256:
        Send_Infantry();
        CommuData.interactData.data[0] = '0';
        break;
    default:
        CommuData.dataFrameHeader.receiver_ID = 88; //随便给个ID，不发送
        break;
    }
    //CommuData.interactData.data[0] = '1';//发送的内容  注意大小不可以超过变量的变量类型
    //CommuData.interactData.data[1] = '2';//发送的内容  注意大小不可以超过变量的变量类型

    memcpy(TeammateTxBuffer + 5, (uint8_t *)&CommuData.CmdID, (sizeof(CommuData.CmdID) + sizeof(CommuData.dataFrameHeader) + sizeof(CommuData.interactData)));
    Append_CRC16_Check_Sum(TeammateTxBuffer, sizeof(CommuData));

    datalength = sizeof(CommuData);
    if (cmd)
    {
        HAL_UART_Transmit(&huart3, TeammateTxBuffer, datalength, 0xFF);
    }
}

/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
_Bool Color;
_Bool is_red_or_blue(void)
{
    Judge_Self_ID = ext_game_robot_state.data.robot_id; //读取当前机器人ID

    if (ext_game_robot_state.data.robot_id > 10)
    {
        return 0; //蓝方
    }
    else
    {
        return 1; //红方
    }
}

/**
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void determine_ID(void)
{
    Color = is_red_or_blue();
    if (Color == BLUE)
    {
        Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID - 10); //计算客户端ID
    }
    else if (Color == RED)
    {
        Judge_SelfClient_ID = 0x0100 + Judge_Self_ID; //计算客户端ID
    }
}

/**
  * @brief  选择发送对象
  * @param  void
  * @retval Send_Color
  * @attention  
  */
void Send_Infantry(void)
{
    if (Send_Color == 0)
    {
        CommuData.dataFrameHeader.receiver_ID = 103;
    }
    else if (Send_Color == 1)
    {
        CommuData.dataFrameHeader.receiver_ID = 3;
    }
}

void Send_Hero(void)
{
    if (Send_Color == 0)
    {
        CommuData.dataFrameHeader.receiver_ID = 101;
    }
    else if (Send_Color == 1)
    {
        CommuData.dataFrameHeader.receiver_ID = 1;
    }
}

void Send_Sentry(void)
{
    if (Send_Color == 0)
    {
        CommuData.dataFrameHeader.receiver_ID = 107;
    }
    else if (Send_Color == 1)
    {
        CommuData.dataFrameHeader.receiver_ID = 7;
    }
}

void Send_Engineering(void)
{
    if (Send_Color == 0)
    {
        CommuData.dataFrameHeader.receiver_ID = 102;
    }
    else if (Send_Color == 1)
    {
        CommuData.dataFrameHeader.receiver_ID = 2;
    }
}

/*************************************2021赛季裁判系统*********************************************/
#elif JUDGE_VERSION == JUDGE_21

#define Judge_SendDataLengthPC 28
#define Judge_SendDataLengthROBOT 9

RM_Judge_t JudgeSystem;
Judge_SendRobot_t Judge_SendRobot;

ext_game_status_t ext_game_status;
ext_game_result_t ext_game_result;
ext_game_robot_HP_t ext_game_robot_HP;
ext_dart_status_t ext_dart_status;
ext_ICRA_buff_debuff_zone_status_t ext_ICRA_buff_debuff_zone_status;
ext_event_data_t ext_even_data;
ext_supply_projectile_action_t ext_supply_projectile_action;
ext_referee_warning_t ext_referee_warning;
ext_dart_remaining_time_t ext_dart_remaining_time;
ext_game_robot_status_t ext_game_robot_state;
ext_power_heat_data_t ext_power_heat_data;
ext_game_robot_pos_t ext_game_robot_pos;
ext_buff_t Buff;
aerial_robot_energy_t aerial_robot_energy;
ext_robot_hurt_t ext_robot_hurt;
ext_shoot_data_t ext_shoot_data;
ext_bullet_remaining_t ext_bullet_remaining;
ext_rfid_status_t ext_rfid_status;
ext_dart_client_cmd_t ext_dart_client_cmd;

ext_CommunatianData_t CommuData; //队友通信信息
/*客户端定义*/
/*准心及指示灯*/
ext_SendClientData_t ShowData; //客户端信息
ext_ShowCrossHair_t Ex_ShowData;
ext_DeleteClientData_t DeleteClient; //删除客户端
/*射速等级*/
ext_ShootLevelData_t ShowshootLv;
ext_ShootLevelData_t ShootLvInit;
/*超级电容容量*/
ext_Cap_Energyvalue_t Cap_Energyvalue;
ext_Cap_Energy_t Cap_Energyshow;
/*工程抬升高度*/
ext_UpliftHeightData_t Uplift_Height;
ext_UpliftHeightData_t Uplift_HeightValue;
/*工程夹子角度*/
ext_ClipAngeleData_t Clip_Angle;
ext_ClipAngeleData_t Clip_AngleValue;
/*哨兵状态指示*/
ext_SentryStatusData_t SentryStatus_Data;
ext_SentryStatusData_t Sentry_Status;
ext_CommunatianData_t Sentry_CommuData;
/**飞镖状态指示**/
ext_DartStatusData_t DartStatus_Data;
ext_DartStatusData_t Dart_Status;
ext_CommunatianData_t Dart_CommuData;
/*车距框*/
ext_CarDistance_t Car_Distance;
/*机器人状态灯信息*/
ext_LedMeaning_t Led_Meaning;
ext_LedMeaning_t Led_Yellow;
ext_LedMeaning_t Led_Green;
ext_LedMeaning_t Led_Orange;
ext_LedMeaning_t Led_Purple;
ext_LedMeaning_t Led_Pink;
//test
ext_MapCommunate_t MapCommunate;
ext_robot_command_t Robot_Command;

uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE]; //接收buff
uint8_t Robot_Commute[26];
uint8_t Map_Commute[15];
uint8_t Judge_Self_ID;        //当前机器人ID
uint16_t Judge_SelfClient_ID; //发送者机器人对应的客户端ID
uint8_t SenrtyMode_Flag = 0;
uint8_t DartMode_Flag = 0;
int YellowFlag = 0;
int GreenFlag = 0;
int OrangeFlag = 0;
int PurpleFlag = 0;
int PinkFlag = 0;

/**************************************大疆CRC校验代码************************************************/
/*         Start            */
uint16_t wExpected;

//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
    {
        0x00,
        0x5e,
        0xbc,
        0xe2,
        0x61,
        0x3f,
        0xdd,
        0x83,
        0xc2,
        0x9c,
        0x7e,
        0x20,
        0xa3,
        0xfd,
        0x1f,
        0x41,
        0x9d,
        0xc3,
        0x21,
        0x7f,
        0xfc,
        0xa2,
        0x40,
        0x1e,
        0x5f,
        0x01,
        0xe3,
        0xbd,
        0x3e,
        0x60,
        0x82,
        0xdc,
        0x23,
        0x7d,
        0x9f,
        0xc1,
        0x42,
        0x1c,
        0xfe,
        0xa0,
        0xe1,
        0xbf,
        0x5d,
        0x03,
        0x80,
        0xde,
        0x3c,
        0x62,
        0xbe,
        0xe0,
        0x02,
        0x5c,
        0xdf,
        0x81,
        0x63,
        0x3d,
        0x7c,
        0x22,
        0xc0,
        0x9e,
        0x1d,
        0x43,
        0xa1,
        0xff,
        0x46,
        0x18,
        0xfa,
        0xa4,
        0x27,
        0x79,
        0x9b,
        0xc5,
        0x84,
        0xda,
        0x38,
        0x66,
        0xe5,
        0xbb,
        0x59,
        0x07,
        0xdb,
        0x85,
        0x67,
        0x39,
        0xba,
        0xe4,
        0x06,
        0x58,
        0x19,
        0x47,
        0xa5,
        0xfb,
        0x78,
        0x26,
        0xc4,
        0x9a,
        0x65,
        0x3b,
        0xd9,
        0x87,
        0x04,
        0x5a,
        0xb8,
        0xe6,
        0xa7,
        0xf9,
        0x1b,
        0x45,
        0xc6,
        0x98,
        0x7a,
        0x24,
        0xf8,
        0xa6,
        0x44,
        0x1a,
        0x99,
        0xc7,
        0x25,
        0x7b,
        0x3a,
        0x64,
        0x86,
        0xd8,
        0x5b,
        0x05,
        0xe7,
        0xb9,
        0x8c,
        0xd2,
        0x30,
        0x6e,
        0xed,
        0xb3,
        0x51,
        0x0f,
        0x4e,
        0x10,
        0xf2,
        0xac,
        0x2f,
        0x71,
        0x93,
        0xcd,
        0x11,
        0x4f,
        0xad,
        0xf3,
        0x70,
        0x2e,
        0xcc,
        0x92,
        0xd3,
        0x8d,
        0x6f,
        0x31,
        0xb2,
        0xec,
        0x0e,
        0x50,
        0xaf,
        0xf1,
        0x13,
        0x4d,
        0xce,
        0x90,
        0x72,
        0x2c,
        0x6d,
        0x33,
        0xd1,
        0x8f,
        0x0c,
        0x52,
        0xb0,
        0xee,
        0x32,
        0x6c,
        0x8e,
        0xd0,
        0x53,
        0x0d,
        0xef,
        0xb1,
        0xf0,
        0xae,
        0x4c,
        0x12,
        0x91,
        0xcf,
        0x2d,
        0x73,
        0xca,
        0x94,
        0x76,
        0x28,
        0xab,
        0xf5,
        0x17,
        0x49,
        0x08,
        0x56,
        0xb4,
        0xea,
        0x69,
        0x37,
        0xd5,
        0x8b,
        0x57,
        0x09,
        0xeb,
        0xb5,
        0x36,
        0x68,
        0x8a,
        0xd4,
        0x95,
        0xcb,
        0x29,
        0x77,
        0xf4,
        0xaa,
        0x48,
        0x16,
        0xe9,
        0xb7,
        0x55,
        0x0b,
        0x88,
        0xd6,
        0x34,
        0x6a,
        0x2b,
        0x75,
        0x97,
        0xc9,
        0x4a,
        0x14,
        0xf6,
        0xa8,
        0x74,
        0x2a,
        0xc8,
        0x96,
        0x15,
        0x4b,
        0xa9,
        0xf7,
        0xb6,
        0xe8,
        0x0a,
        0x54,
        0xd7,
        0x89,
        0x6b,
        0x35,
};

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return;
    ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;

const uint16_t wCRC_Table[256] =
    {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while (dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum((uint8_t *)pchMessage, dwLength - 2, CRC_INIT);
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}
/*整数型转化为字符型*/
void int_into_char(char *Data, int16_t i)
{
    char data[10];
    sprintf(data, "%d", i);
    strcpy(Data, data);
}

/*           END            */
/**************************************大疆CRC校验代码************************************************/

/*****************************************************
 ** @brief  获取裁判系统信息                        **
 ** @param  Data_Length 	获取到的数据长度          **
 ** **************************************************
 */
void Judge_GetMessage(uint16_t Data_Length)
{
    for (int n = 0; n < Data_Length;)
    {
        if (JudgeSystem_rxBuff[n] == JUDGE_FRAME_HEADER)
        {
            switch (JudgeSystem_rxBuff[n + 5] | JudgeSystem_rxBuff[n + 6] << 8)
            {
            case Judge_Game_StatusData: //比赛状态数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Game_StatusData))
                {
                    memcpy(ext_game_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[11]));
                    n += JudgeLength_Game_StatusData;
                    ext_game_status.infoUpdateFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Game_ResultData: //比赛结果
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Game_ResultData))
                {
                    memcpy(ext_game_result.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeLength_Game_ResultData;
                    ext_game_result.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_HP: //机器人血量数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_HP))
                {
                    memcpy(&ext_game_robot_HP.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[32]));
                    n += JudgeLength_Robot_HP;
                    ext_game_robot_HP.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Dart_Launch: //飞镖发射状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Launch))
                {
                    memcpy(&ext_dart_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[3]));
                    n += JudgeLength_Dart_Launch;
                    ext_dart_status.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_AI_ChallengeBuff: //AI加成与惩罚
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_AI_ChallengeBuff))
                {
                    memcpy(&ext_ICRA_buff_debuff_zone_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[11]));
                    n += JudgeLength_AI_ChallengeBuff;
                    ext_ICRA_buff_debuff_zone_status.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Event_Data: //场地事件数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Event_Data))
                {
                    memcpy(&ext_even_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[4]));
                    n += JudgeLength_Event_Data;
                    ext_even_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Supply_Station: //补给站动作标识
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Supply_Station))
                {
                    memcpy(&ext_supply_projectile_action.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[4]));
                    n += JudgeLength_Supply_Station;
                    ext_supply_projectile_action.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Referee_Warning: //裁判系统警告信息
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Referee_Warning))
                {
                    memcpy(&ext_referee_warning.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[2]));
                    n += JudgeLength_Referee_Warning;
                    ext_referee_warning.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Dart_Countdown: //飞镖发射口倒计时
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Countdown))
                {
                    memcpy(&ext_dart_remaining_time.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeLength_Dart_Countdown;
                    ext_dart_remaining_time.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_State: //比赛机器人状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_State))
                {
                    memcpy(&ext_game_robot_state.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[27]));
                    n += JudgeLength_Robot_State;
                    ext_game_robot_state.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Power_Heat: //实时功率热量
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Power_Heat))
                {
                    memcpy(&ext_power_heat_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[16]));
                    n += JudgeLength_Power_Heat;
                    ext_power_heat_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_Position: //机器人位置
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Position))
                {
                    memcpy(&ext_game_robot_pos.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[16]));
                    n += JudgeLength_Robot_Position;
                    ext_game_robot_pos.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_Buff: //机器人增益
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Buff))
                {
                    memcpy(&Buff.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeLength_Robot_Buff;
                    Buff.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Aerial_Energy: //空中机器人能量状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Aerial_Energy))
                {
                    memcpy(&aerial_robot_energy.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeLength_Aerial_Energy;
                    aerial_robot_energy.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Injury_State: //伤害状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Injury_State))
                {
                    memcpy(&ext_robot_hurt.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeLength_Injury_State;
                    ext_robot_hurt.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_RealTime_Shoot: //实时射击数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_RealTime_Shoot))
                {
                    memcpy(&ext_shoot_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[7]));
                    n += JudgeLength_RealTime_Shoot;
                    ext_shoot_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Remaining_Rounds: //子弹剩余数
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Remaining_Rounds))
                {
                    memcpy(&ext_bullet_remaining.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[6]));
                    n += JudgeLength_Remaining_Rounds;
                    ext_bullet_remaining.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_RFID: //机器人RFID状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_RFID))
                {
                    memcpy(&ext_rfid_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[4]));
                    n += JudgeLength_Robot_RFID;
                    ext_rfid_status.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Dart_Client: //飞镖机器人客户端指令数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Client))
                {
                    memcpy(&ext_dart_client_cmd.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[12]));
                    n += JudgeLength_Dart_Client;
                    ext_dart_client_cmd.InfoUpdataFlag = 1;
                }
                else
                    n++; //26
                break;
            case Judge_Robot_Communicate: //机器人信息交互(还有一种写法就是直接case内容ID 不case命令码)
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Commute))
                {
                    memcpy(&Robot_Commute, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[26]));
                    n += JudgeLength_Robot_Commute;
                }
                else
                    n++;
                break;
            default:
                n++;
                break;
            }
        }
        else
            n++;
    }
    JudgeSystem.InfoUpdateFrame++;
}
/**
* @brief  机器人之间互传信息
  * @param
  * @retval None
  */
#define Teammate_max_len 40 //200

unsigned char TeammateTxBuffer[Teammate_max_len];
bool Send_Color = 0;
uint16_t send_time = 0;
//参数cmd是机器人信息交互的协议：在0x0200-0x02FF之间选择 (令0x02FF是无用数据id)
void Judge_sendRobot(uint16_t cmd)
{
    static uint8_t datalength;
    Send_Color = is_red_or_blue();   //判断发送给兵种的颜色,107哨兵(蓝),7哨兵(红)；103'g'步兵蓝  3步兵红
    memset(TeammateTxBuffer, 0, 40); //初始化TeammateTxBuffer

    CommuData.txFrameHeader.SOF = 0xA5;
    CommuData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
    CommuData.txFrameHeader.Seq = 0x00;
    memcpy(TeammateTxBuffer, &CommuData.txFrameHeader, sizeof(xFrameHeader));
    Append_CRC8_Check_Sum(TeammateTxBuffer, sizeof(xFrameHeader));

    CommuData.CmdID = 0x0301;
    CommuData.dataFrameHeader.sender_ID = Judge_Self_ID;
    CommuData.dataFrameHeader.data_cmd_id = cmd; //裁判协议：在0x0200-0x02FF之间选择

    switch (cmd)
    {
    case 0x0255:
        //发给步兵
        Send_Dart();
        //Send_Infantry();
        //Send_Sentry();
        CommuData.interactData.data[0] = '1'; //发送的内容  注意大小不可以超过变量的变量类型
        break;
    case 0x256:
        Send_Infantry();
        //CommuData.interactData.data[0] = '0';
        //strcpy(CommuData.interactData.data,"Holy_shit");
        break;
    default:
        CommuData.dataFrameHeader.receiver_ID = 88; //随便给个ID，不发送
        break;
    }

    memcpy(TeammateTxBuffer + 5, (uint8_t *)&CommuData.CmdID, (sizeof(CommuData.CmdID) + sizeof(CommuData.dataFrameHeader) + sizeof(CommuData.interactData)));
    Append_CRC16_Check_Sum(TeammateTxBuffer, sizeof(CommuData));
    datalength = sizeof(CommuData);
    HAL_UART_Transmit(&huart3, TeammateTxBuffer, datalength, 0xFF);
}

/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
_Bool Color;
_Bool is_red_or_blue(void)
{
    Judge_Self_ID = ext_game_robot_state.data.robot_id; //读取当前机器人ID

    if (ext_game_robot_state.data.robot_id > 10)
    {
        return 0; //蓝方
    }
    else
    {
        return 1; //红方
    }
}

/**
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void determine_ID(void)
{
    Color = is_red_or_blue();
    if (Color == BLUE)
    {
        Judge_SelfClient_ID = 0x0100 + ext_game_robot_state.data.robot_id; //计算客户端ID
    }
    else if (Color == RED)
    {
        Judge_SelfClient_ID = 0x0100 + ext_game_robot_state.data.robot_id; //计算客户端ID
    }
}
/*******************************************************************************
/    内容ID          长度（头结构+内容数据段）            功能说明
/ 0x0200~0x02FF             6+n                         己方机器人间通信           
/    0x0100                 6+2                         客户端删除图形
/    0x0101                 6+15                        客户端绘制一个图形
/    0x0102                 6+30                        客户端绘制二个图形
/    0x0103                 6+75                        客户端绘制五个图形
/    0x0104                 6+105                       客户端绘制七个图形
/    0x0110                 6+45                        客户端绘制字符图形
********************************************************************************/
/*
	uint8_t graphic_name[3]; //图形名,在删除，修改等操作中，作为客户端的索引。
	uint32_t operate_tpye:3; //图形操作：0：空操作；1：增加；2：修改；3：删除；
uint32_t graphic_tpye:3; //图形类型：0：直线；1：矩形；2：整圆；3：椭圆；4：圆弧；5:浮点数 6：整型数 7：字符
	uint32_t layer:4; //图层数，0~9
	uint32_t color:4; //颜色：0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	uint32_t start_angle:9; //：起始角度，单位：°，范围[0,360]；
	uint32_t end_angle:9; //终止角度，单位：°，范围[0,360]。
	uint32_t width:10; //线宽；0-9
	uint32_t start_x:11; //起点x坐标；
	uint32_t start_y:11; //起点y坐标。
	uint32_t radius:10; //字体大小或者半径；
	uint32_t end_x:11; //终点x坐标；
	uint32_t end_y:11; //终点y坐标。
*/
unsigned char DeleteBuffer[20];

void UI_DeleteAll(void)
{
    static uint8_t datalength;
    DeleteClient.txFrameHeader.SOF = 0xA5;
    DeleteClient.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_graphic_delete_t);
    memcpy(DeleteBuffer, &DeleteClient.txFrameHeader, sizeof(xFrameHeader));
    Append_CRC8_Check_Sum(DeleteBuffer, sizeof(xFrameHeader));

    DeleteClient.CmdID = 0x0301;

    determine_ID();

    DeleteClient.dataFrameHeader.data_cmd_id = 0x0100;
    DeleteClient.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    DeleteClient.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    DeleteClient.cilentData.operate_tpye = 2;

    memcpy(DeleteBuffer + 5,
           (uint8_t *)&DeleteClient.CmdID,
           (sizeof(DeleteClient.CmdID) + sizeof(DeleteClient.dataFrameHeader) + sizeof(DeleteClient.cilentData)));
    Append_CRC16_Check_Sum(DeleteBuffer, sizeof(DeleteClient)); //写入数据段CRC16校验码
    datalength = sizeof(DeleteClient);
    HAL_UART_Transmit(&huart3, DeleteBuffer, datalength, 0xFF);
}

#define send_max_len 120
unsigned char CliendTxBuffer[send_max_len];
/*显示准心*/
void Standard_CrossHair(void)
{
    static uint8_t datalength;

    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID

    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;                           //客户端绘制一个图形
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[0].graphic_name[0] = 1;
    ShowData.cilentData[0].graphic_tpye = 0;
    ShowData.cilentData[0].operate_tpye = 1;
    ShowData.cilentData[0].layer = 0;
    ShowData.cilentData[0].color = 2;
    ShowData.cilentData[0].start_angle = 0;
    ShowData.cilentData[0].end_angle = 0;
    ShowData.cilentData[0].width = 1;
    ShowData.cilentData[0].start_x = 920; //原840
    ShowData.cilentData[0].start_y = 680;
    ShowData.cilentData[0].radius = 0;
    ShowData.cilentData[0].end_x = 1000; //原1080
    ShowData.cilentData[0].end_y = 680;

    ShowData.cilentData[1].graphic_name[0] = 2;
    ShowData.cilentData[1].graphic_tpye = 0;
    ShowData.cilentData[1].operate_tpye = 1;
    ShowData.cilentData[1].layer = 0;
    ShowData.cilentData[1].color = 2;
    ShowData.cilentData[1].start_angle = 0;
    ShowData.cilentData[1].end_angle = 0;
    ShowData.cilentData[1].width = 1;
    ShowData.cilentData[1].start_x = 580; //原 860
    ShowData.cilentData[1].start_y = 540; //原：560
    ShowData.cilentData[1].radius = 0;
    ShowData.cilentData[1].end_x = 1340; //原1060
    ShowData.cilentData[1].end_y = 540;

    ShowData.cilentData[2].graphic_name[0] = 3;
    ShowData.cilentData[2].graphic_tpye = 0;
    ShowData.cilentData[2].operate_tpye = 1;
    ShowData.cilentData[2].layer = 0;
    ShowData.cilentData[2].color = 2;
    ShowData.cilentData[2].start_angle = 0;
    ShowData.cilentData[2].end_angle = 0;
    ShowData.cilentData[2].width = 1;
    ShowData.cilentData[2].start_x = 880; //原890
    ShowData.cilentData[2].start_y = 440;
    ShowData.cilentData[2].radius = 0;
    ShowData.cilentData[2].end_x = 1040; //原1030
    ShowData.cilentData[2].end_y = 440;

    ShowData.cilentData[3].graphic_name[0] = 4;
    ShowData.cilentData[3].graphic_tpye = 0;
    ShowData.cilentData[3].operate_tpye = 1;
    ShowData.cilentData[3].layer = 0;
    ShowData.cilentData[3].color = 2;
    ShowData.cilentData[3].start_angle = 0;
    ShowData.cilentData[3].end_angle = 0;
    ShowData.cilentData[3].width = 1;
    ShowData.cilentData[3].start_x = 960;
    ShowData.cilentData[3].start_y = 700;
    ShowData.cilentData[3].radius = 0;
    ShowData.cilentData[3].end_x = 960;
    ShowData.cilentData[3].end_y = 250;

    //打包写入数据库
    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, datalength, 0xFF);
}
unsigned char CrossHairBuffer[90];
void Ex_CrossHair(void)
{
    static uint8_t crosslength;

    Ex_ShowData.txFrameHeader.SOF = 0xA5;
    Ex_ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 5 * sizeof(ext_client_custom_graphic_single_t);
    Ex_ShowData.txFrameHeader.Seq = 0;
    memcpy(CrossHairBuffer, &Ex_ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CrossHairBuffer, sizeof(xFrameHeader));              //写入帧头CRC8校验码

    Ex_ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID

    //ID已经是自动读取的了
    Ex_ShowData.dataFrameHeader.data_cmd_id = 0x0103;                           //客户端绘制两个个图形
    Ex_ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    Ex_ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    Ex_ShowData.cilentData[0].graphic_name[1] = 9;
    Ex_ShowData.cilentData[0].graphic_tpye = 0;
    Ex_ShowData.cilentData[0].operate_tpye = 1;
    Ex_ShowData.cilentData[0].layer = 0;
    Ex_ShowData.cilentData[0].color = 2;
    Ex_ShowData.cilentData[0].start_angle = 0;
    Ex_ShowData.cilentData[0].end_angle = 0;
    Ex_ShowData.cilentData[0].width = 1;
    Ex_ShowData.cilentData[0].start_x = 900; //原840
    Ex_ShowData.cilentData[0].start_y = 400;
    Ex_ShowData.cilentData[0].radius = 0;
    Ex_ShowData.cilentData[0].end_x = 1020; //原1080
    Ex_ShowData.cilentData[0].end_y = 400;

    Ex_ShowData.cilentData[1].graphic_name[1] = 10;
    Ex_ShowData.cilentData[1].graphic_tpye = 0;
    Ex_ShowData.cilentData[1].operate_tpye = 1;
    Ex_ShowData.cilentData[1].layer = 0;
    Ex_ShowData.cilentData[1].color = 2;
    Ex_ShowData.cilentData[1].start_angle = 0;
    Ex_ShowData.cilentData[1].end_angle = 0;
    Ex_ShowData.cilentData[1].width = 1;
    Ex_ShowData.cilentData[1].start_x = 920; //原 860
    Ex_ShowData.cilentData[1].start_y = 360; //原：560
    Ex_ShowData.cilentData[1].radius = 0;
    Ex_ShowData.cilentData[1].end_x = 1000; //原1060
    Ex_ShowData.cilentData[1].end_y = 360;

    Ex_ShowData.cilentData[2].graphic_name[1] = 11;
    Ex_ShowData.cilentData[2].graphic_tpye = 0;
    Ex_ShowData.cilentData[2].operate_tpye = 1;
    Ex_ShowData.cilentData[2].layer = 0;
    Ex_ShowData.cilentData[2].color = 2;
    Ex_ShowData.cilentData[2].start_angle = 0;
    Ex_ShowData.cilentData[2].end_angle = 0;
    Ex_ShowData.cilentData[2].width = 1;
    Ex_ShowData.cilentData[2].start_x = 940; //原 860
    Ex_ShowData.cilentData[2].start_y = 310; //原：560
    Ex_ShowData.cilentData[2].radius = 0;
    Ex_ShowData.cilentData[2].end_x = 980; //原1060
    Ex_ShowData.cilentData[2].end_y = 310;

    Ex_ShowData.cilentData[2].graphic_name[1] = 10;
    Ex_ShowData.cilentData[2].graphic_tpye = 0;
    Ex_ShowData.cilentData[2].operate_tpye = 1;
    Ex_ShowData.cilentData[2].layer = 0;
    Ex_ShowData.cilentData[2].color = 2;
    Ex_ShowData.cilentData[2].start_angle = 0;
    Ex_ShowData.cilentData[2].end_angle = 0;
    Ex_ShowData.cilentData[2].width = 1;
    Ex_ShowData.cilentData[2].start_x = 945; //原 860
    Ex_ShowData.cilentData[2].start_y = 280; //原：560
    Ex_ShowData.cilentData[2].radius = 0;
    Ex_ShowData.cilentData[2].end_x = 975; //原1060
    Ex_ShowData.cilentData[2].end_y = 280;

    //打包写入数据库
    memcpy(CrossHairBuffer + 5,
           (uint8_t *)&Ex_ShowData.CmdID,
           (sizeof(Ex_ShowData.CmdID) + sizeof(Ex_ShowData.dataFrameHeader) + sizeof(Ex_ShowData.cilentData)));
    Append_CRC16_Check_Sum(CrossHairBuffer, sizeof(Ex_ShowData)); //写入数据段CRC16校验码
    crosslength = sizeof(Ex_ShowData);
    HAL_UART_Transmit(&huart3, CrossHairBuffer, crosslength, 0xFF);
}
void Show_CrossHair(void)
{
    Standard_CrossHair();
    Ex_CrossHair();
}
/*机器人状态指示*/
uint8_t Status_Datalength = 0;
void RobotStatus_LEDYellow(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 9;
    ShowData.cilentData[4].graphic_tpye = 2;
    ShowData.cilentData[4].operate_tpye = 1;
    ShowData.cilentData[4].layer = 1;
    ShowData.cilentData[4].color = 1;
    ShowData.cilentData[4].width = 10;
    ShowData.cilentData[4].start_x = 1900;
    ShowData.cilentData[4].start_y = 800;
    ShowData.cilentData[4].radius = 10;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}
void RobotStatus_YellowDelete(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 9;
    ShowData.cilentData[4].operate_tpye = 3;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}

void RobotStatus_LEDGreen(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 10;
    ShowData.cilentData[4].graphic_tpye = 2;
    ShowData.cilentData[4].operate_tpye = 1;
    ShowData.cilentData[4].layer = 1;
    ShowData.cilentData[4].color = 2;
    ShowData.cilentData[4].width = 10;
    ShowData.cilentData[4].start_x = 1900;
    ShowData.cilentData[4].start_y = 750;
    ShowData.cilentData[4].radius = 10;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}
void RobotStatus_GreenDelete(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 10;
    ShowData.cilentData[4].operate_tpye = 3;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}

void RobotStatus_LEDOrange(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 11;
    ShowData.cilentData[4].graphic_tpye = 2;
    ShowData.cilentData[4].operate_tpye = 1;
    ShowData.cilentData[4].layer = 1;
    ShowData.cilentData[4].color = 3;
    ShowData.cilentData[4].width = 10;
    ShowData.cilentData[4].start_x = 1900;
    ShowData.cilentData[4].start_y = 700;
    ShowData.cilentData[4].radius = 10;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}
void RobotStatus_OrangeDelete(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 11;
    ShowData.cilentData[4].operate_tpye = 3;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}

void RobotStatus_LEDPurple(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 12;
    ShowData.cilentData[4].graphic_tpye = 2;
    ShowData.cilentData[4].operate_tpye = 1;
    ShowData.cilentData[4].layer = 1;
    ShowData.cilentData[4].color = 4;
    ShowData.cilentData[4].width = 10;
    ShowData.cilentData[4].start_x = 1900;
    ShowData.cilentData[4].start_y = 650;
    ShowData.cilentData[4].radius = 10;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}
void RobotStatus_PurpleDelete(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 12;
    ShowData.cilentData[4].operate_tpye = 3;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}

void RobotStatus_LEDPink(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 13;
    ShowData.cilentData[4].graphic_tpye = 2;
    ShowData.cilentData[4].operate_tpye = 1;
    ShowData.cilentData[4].layer = 1;
    ShowData.cilentData[4].color = 5;
    ShowData.cilentData[4].width = 10;
    ShowData.cilentData[4].start_x = 1900;
    ShowData.cilentData[4].start_y = 600;
    ShowData.cilentData[4].radius = 10;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}
void RobotStatus_PinkDelete(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 13;
    ShowData.cilentData[4].operate_tpye = 3;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}
void RobotStatus_LEDCyan(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 14;
    ShowData.cilentData[4].graphic_tpye = 2;
    ShowData.cilentData[4].operate_tpye = 1;
    ShowData.cilentData[4].layer = 1;
    ShowData.cilentData[4].color = 6;
    ShowData.cilentData[4].width = 10;
    ShowData.cilentData[4].start_x = 1900;
    ShowData.cilentData[4].start_y = 550;
    ShowData.cilentData[4].radius = 10;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}
void RobotStatus_CyanDelete(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 14;
    ShowData.cilentData[4].operate_tpye = 3;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}
void RobotStatus_LEDBlack(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 15;
    ShowData.cilentData[4].graphic_tpye = 2;
    ShowData.cilentData[4].operate_tpye = 1;
    ShowData.cilentData[4].layer = 1;
    ShowData.cilentData[4].color = 7;
    ShowData.cilentData[4].width = 10;
    ShowData.cilentData[4].start_x = 1900;
    ShowData.cilentData[4].start_y = 500;
    ShowData.cilentData[4].radius = 10;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}
void RobotStatus_BlackDelete(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 15;
    ShowData.cilentData[4].operate_tpye = 3;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}

void RobotStatus_LEDWhite(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 22;
    ShowData.cilentData[4].graphic_tpye = 2;
    ShowData.cilentData[4].operate_tpye = 1;
    ShowData.cilentData[4].layer = 1;
    ShowData.cilentData[4].color = 8;
    ShowData.cilentData[4].width = 10;
    ShowData.cilentData[4].start_x = 1900;
    ShowData.cilentData[4].start_y = 450;
    ShowData.cilentData[4].radius = 10;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}
void RobotStatus_WhiteDelete(void)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 22;
    ShowData.cilentData[4].operate_tpye = 3;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
}

//机器人指示灯字符显示
unsigned char ColorMeanBuffer[100];
unsigned char YellowMeanBuffer[100];
void LEDYellow_Mean(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Yellow.txFrameHeader.SOF = 0xA5;
    Led_Yellow.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Yellow.txFrameHeader.Seq = 0;
    memcpy(YellowMeanBuffer, &Led_Yellow.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(YellowMeanBuffer, sizeof(xFrameHeader));

    Led_Yellow.CmdID = 0x0301;
    determine_ID();
    Led_Yellow.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Yellow.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Yellow.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Yellow.cilentData.grapic_data_struct.graphic_name[1] = 1;
    Led_Yellow.cilentData.grapic_data_struct.layer = 1;
    Led_Yellow.cilentData.grapic_data_struct.operate_tpye = 1;
    Led_Yellow.cilentData.grapic_data_struct.graphic_tpye = 7;
    Led_Yellow.cilentData.grapic_data_struct.color = 1;
    Led_Yellow.cilentData.grapic_data_struct.start_angle = 20;
    Led_Yellow.cilentData.grapic_data_struct.end_angle = 12; //需要按照自己显示的字符长度修改
    Led_Yellow.cilentData.grapic_data_struct.width = 1;
    Led_Yellow.cilentData.grapic_data_struct.start_x = 1500;
    Led_Yellow.cilentData.grapic_data_struct.start_y = 830;

    strcpy(Led_Yellow.cilentData.data, LED_Init);
    strcpy(Led_Yellow.cilentData.data, LED_YELLOWMeaning);

    memcpy(YellowMeanBuffer + 5,
           (uint8_t *)&Led_Yellow.CmdID,
           (sizeof(Led_Yellow.CmdID) + sizeof(Led_Yellow.dataFrameHeader) + sizeof(Led_Yellow.cilentData)));
    Append_CRC16_Check_Sum(YellowMeanBuffer, sizeof(Led_Yellow));
    LEDMEAN_Datalength = sizeof(Led_Yellow);
    HAL_UART_Transmit(&huart3, YellowMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDYellow_MeanUpdate(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Yellow.txFrameHeader.SOF = 0xA5;
    Led_Yellow.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Yellow.txFrameHeader.Seq = 0;
    memcpy(YellowMeanBuffer, &Led_Yellow.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(YellowMeanBuffer, sizeof(xFrameHeader));

    Led_Yellow.CmdID = 0x0301;
    determine_ID();
    Led_Yellow.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Yellow.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Yellow.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Yellow.cilentData.grapic_data_struct.graphic_name[1] = 1;
    Led_Yellow.cilentData.grapic_data_struct.operate_tpye = 2;

    if (ext_game_robot_state.data.shooter_id1_17mm_speed_limit != 0)
    {
        switch (ext_game_robot_state.data.shooter_id1_17mm_speed_limit)
        {
        case 15:
            strcpy(Led_Yellow.cilentData.data, "Fric:15 ");
            break;
        case 18:
            strcpy(Led_Yellow.cilentData.data, "Fric:18 ");
            break;
        case 22:
            strcpy(Led_Yellow.cilentData.data, "Fric:22 ");
            break;
        case 30:
            strcpy(Led_Yellow.cilentData.data, "Fric:30 ");
            break;
        }
    }
    else
    {
        strcpy(Led_Yellow.cilentData.data, "Fric:PowerOFF ");
    }

    memcpy(YellowMeanBuffer + 5,
           (uint8_t *)&Led_Yellow.CmdID,
           (sizeof(Led_Yellow.CmdID) + sizeof(Led_Yellow.dataFrameHeader) + sizeof(Led_Yellow.cilentData)));
    Append_CRC16_Check_Sum(YellowMeanBuffer, sizeof(Led_Yellow));
    LEDMEAN_Datalength = sizeof(Led_Yellow);
    HAL_UART_Transmit(&huart3, YellowMeanBuffer, LEDMEAN_Datalength, 0xFF);
}

void LEDYellow_MeanDelete(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Meaning.txFrameHeader.SOF = 0xA5;
    Led_Meaning.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Meaning.txFrameHeader.Seq = 0;
    memcpy(ColorMeanBuffer, &Led_Meaning.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(ColorMeanBuffer, sizeof(xFrameHeader));

    Led_Meaning.CmdID = 0x0301;
    determine_ID();
    Led_Meaning.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Meaning.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Meaning.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Meaning.cilentData.grapic_data_struct.graphic_name[1] = 1;
    Led_Meaning.cilentData.grapic_data_struct.operate_tpye = 3;

    memcpy(ColorMeanBuffer + 5,
           (uint8_t *)&Led_Meaning.CmdID,
           (sizeof(Led_Meaning.CmdID) + sizeof(Led_Meaning.dataFrameHeader) + sizeof(Led_Meaning.cilentData)));
    Append_CRC16_Check_Sum(ColorMeanBuffer, sizeof(Led_Meaning));
    LEDMEAN_Datalength = sizeof(Led_Meaning);
    HAL_UART_Transmit(&huart3, ColorMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
unsigned char GreenMeanBuffer[100];
void LEDGreen_Mean(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Green.txFrameHeader.SOF = 0xA5;
    Led_Green.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Green.txFrameHeader.Seq = 0;
    memcpy(GreenMeanBuffer, &Led_Green.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(GreenMeanBuffer, sizeof(xFrameHeader));

    Led_Green.CmdID = 0x0301;
    determine_ID();
    Led_Green.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Green.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Green.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Green.cilentData.grapic_data_struct.graphic_name[1] = 2;
    Led_Green.cilentData.grapic_data_struct.layer = 1;
    Led_Green.cilentData.grapic_data_struct.operate_tpye = 1;
    Led_Green.cilentData.grapic_data_struct.graphic_tpye = 7;
    Led_Green.cilentData.grapic_data_struct.color = 2;
    Led_Green.cilentData.grapic_data_struct.start_angle = 25;
    Led_Green.cilentData.grapic_data_struct.end_angle = 25; //同一10
    Led_Green.cilentData.grapic_data_struct.width = 1;
    Led_Green.cilentData.grapic_data_struct.start_x = 10; //1850
    Led_Green.cilentData.grapic_data_struct.start_y = 580;  //640

    strcpy(Led_Green.cilentData.data, LED_Init);
    strcpy(Led_Green.cilentData.data, LED_GREENMeaning);
    memcpy(GreenMeanBuffer + 5,
           (uint8_t *)&Led_Green.CmdID,
           (sizeof(Led_Green.CmdID) + sizeof(Led_Green.dataFrameHeader) + sizeof(Led_Green.cilentData)));
    Append_CRC16_Check_Sum(GreenMeanBuffer, sizeof(Led_Green));
    LEDMEAN_Datalength = sizeof(Led_Green);
    HAL_UART_Transmit(&huart3, GreenMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDGreen_MeanUpdate(void)
{
//    uint8_t LEDMEAN_Datalength;
//    Led_Green.txFrameHeader.SOF = 0xA5;
//    Led_Green.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//    Led_Green.txFrameHeader.Seq = 0;
//    memcpy(GreenMeanBuffer, &Led_Green.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
//    Append_CRC8_Check_Sum(GreenMeanBuffer, sizeof(xFrameHeader));

//    Led_Green.CmdID = 0x0301;
//    determine_ID();
//    Led_Green.dataFrameHeader.data_cmd_id = 0x0110;
//    Led_Green.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
//    Led_Green.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

//    Led_Green.cilentData.grapic_data_struct.graphic_name[1] = 2;
//    Led_Green.cilentData.grapic_data_struct.operate_tpye = 2;

//    switch (Robot.Magazine)
//    {
//    case Magazine_Open:
//        strcpy(Led_Green.cilentData.data, "Magazine:Open ");
//        break;

//    case Magazine_Close:
//        strcpy(Led_Green.cilentData.data, "Magazine:Close ");
//        break;

//    default:
//        break;
//    }

//    memcpy(GreenMeanBuffer + 5,
//           (uint8_t *)&Led_Green.CmdID,
//           (sizeof(Led_Green.CmdID) + sizeof(Led_Green.dataFrameHeader) + sizeof(Led_Green.cilentData)));
//    Append_CRC16_Check_Sum(GreenMeanBuffer, sizeof(Led_Green));
//    LEDMEAN_Datalength = sizeof(Led_Green);
//    HAL_UART_Transmit(&huart3, GreenMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDGreen_MeanDelete(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Meaning.txFrameHeader.SOF = 0xA5;
    Led_Meaning.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Meaning.txFrameHeader.Seq = 0;
    memcpy(ColorMeanBuffer, &Led_Meaning.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(ColorMeanBuffer, sizeof(xFrameHeader));

    Led_Meaning.CmdID = 0x0301;
    determine_ID();
    Led_Meaning.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Meaning.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Meaning.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Meaning.cilentData.grapic_data_struct.graphic_name[1] = 2;
    Led_Meaning.cilentData.grapic_data_struct.operate_tpye = 3;

    memcpy(ColorMeanBuffer + 5,
           (uint8_t *)&Led_Meaning.CmdID,
           (sizeof(Led_Meaning.CmdID) + sizeof(Led_Meaning.dataFrameHeader) + sizeof(Led_Meaning.cilentData)));
    Append_CRC16_Check_Sum(ColorMeanBuffer, sizeof(Led_Meaning));
    LEDMEAN_Datalength = sizeof(Led_Meaning);
    HAL_UART_Transmit(&huart3, ColorMeanBuffer, LEDMEAN_Datalength, 0xFF);
}

unsigned char OrangeMeanBuffer[100];
void LEDOrange_Mean(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Orange.txFrameHeader.SOF = 0xA5;
    Led_Orange.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Orange.txFrameHeader.Seq = 0;
    memcpy(OrangeMeanBuffer, &Led_Orange.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(OrangeMeanBuffer, sizeof(xFrameHeader));

    Led_Orange.CmdID = 0x0301;
    determine_ID();
    Led_Orange.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Orange.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Orange.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Orange.cilentData.grapic_data_struct.graphic_name[1] = 3;
    Led_Orange.cilentData.grapic_data_struct.layer = 1;
    Led_Orange.cilentData.grapic_data_struct.operate_tpye = 1;
    Led_Orange.cilentData.grapic_data_struct.graphic_tpye = 7;
    Led_Orange.cilentData.grapic_data_struct.color = 3;
    Led_Orange.cilentData.grapic_data_struct.start_angle = 20;
    Led_Orange.cilentData.grapic_data_struct.end_angle = 25;
    Led_Orange.cilentData.grapic_data_struct.width = 1;
    Led_Orange.cilentData.grapic_data_struct.start_x = 1500; //1850
    Led_Orange.cilentData.grapic_data_struct.start_y = 750;  //640

    strcpy(Led_Orange.cilentData.data, LED_Init);
    strcpy(Led_Orange.cilentData.data, LED_ORANGEMeaning);

    memcpy(OrangeMeanBuffer + 5,
           (uint8_t *)&Led_Orange.CmdID,
           (sizeof(Led_Orange.CmdID) + sizeof(Led_Orange.dataFrameHeader) + sizeof(Led_Orange.cilentData)));
    Append_CRC16_Check_Sum(OrangeMeanBuffer, sizeof(Led_Orange));
    LEDMEAN_Datalength = sizeof(Led_Orange);
    HAL_UART_Transmit(&huart3, OrangeMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDOrange_MeanUpdate(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Orange.txFrameHeader.SOF = 0xA5;
    Led_Orange.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Orange.txFrameHeader.Seq = 0;
    memcpy(OrangeMeanBuffer, &Led_Orange.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(OrangeMeanBuffer, sizeof(xFrameHeader));

    Led_Orange.CmdID = 0x0301;
    determine_ID();
    Led_Orange.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Orange.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Orange.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Orange.cilentData.grapic_data_struct.graphic_name[1] = 3;
    Led_Orange.cilentData.grapic_data_struct.operate_tpye = 2;

    switch (Robot.ChassisWorkMode)
    {
    case ChassisWorkMode_CloudFollow:
        strcpy(Led_Orange.cilentData.data, "ChassisMode:CloudFollow  ");
        break;

    case ChassisWorkMode_Lock:
        strcpy(Led_Orange.cilentData.data, "ChassisMode:Lock  ");
        break;

    case ChassisWorkMode_Disable:
        strcpy(Led_Orange.cilentData.data, "ChassisMode:Disable    ");
        break;

    default:
        break;
    }

    memcpy(OrangeMeanBuffer + 5,
           (uint8_t *)&Led_Orange.CmdID,
           (sizeof(Led_Orange.CmdID) + sizeof(Led_Orange.dataFrameHeader) + sizeof(Led_Orange.cilentData)));
    Append_CRC16_Check_Sum(OrangeMeanBuffer, sizeof(Led_Orange));
    LEDMEAN_Datalength = sizeof(Led_Orange);
    HAL_UART_Transmit(&huart3, OrangeMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDOrange_MeanDelete(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Orange.txFrameHeader.SOF = 0xA5;
    Led_Orange.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Orange.txFrameHeader.Seq = 0;
    memcpy(ColorMeanBuffer, &Led_Orange.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(ColorMeanBuffer, sizeof(xFrameHeader));

    Led_Orange.CmdID = 0x0301;
    determine_ID();
    Led_Orange.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Orange.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Orange.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Orange.cilentData.grapic_data_struct.graphic_name[1] = 3;
    Led_Orange.cilentData.grapic_data_struct.operate_tpye = 3;

    memcpy(ColorMeanBuffer + 5,
           (uint8_t *)&Led_Orange.CmdID,
           (sizeof(Led_Orange.CmdID) + sizeof(Led_Orange.dataFrameHeader) + sizeof(Led_Orange.cilentData)));
    Append_CRC16_Check_Sum(ColorMeanBuffer, sizeof(Led_Orange));
    LEDMEAN_Datalength = sizeof(Led_Orange);
    HAL_UART_Transmit(&huart3, ColorMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
unsigned char PurpleMeanBuffer[100];
void LEDPurple_Mean(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Purple.txFrameHeader.SOF = 0xA5;
    Led_Purple.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Purple.txFrameHeader.Seq = 0;
    memcpy(PurpleMeanBuffer, &Led_Purple.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(PurpleMeanBuffer, sizeof(xFrameHeader));

    Led_Purple.CmdID = 0x0301;
    determine_ID();
    Led_Purple.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Purple.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Purple.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Purple.cilentData.grapic_data_struct.graphic_name[1] = 4;
    Led_Purple.cilentData.grapic_data_struct.layer = 1;
    Led_Purple.cilentData.grapic_data_struct.operate_tpye = 1;
    Led_Purple.cilentData.grapic_data_struct.graphic_tpye = 7;
    Led_Purple.cilentData.grapic_data_struct.color = 1;
    Led_Purple.cilentData.grapic_data_struct.start_angle = 25;
    Led_Purple.cilentData.grapic_data_struct.end_angle = 25;
    Led_Purple.cilentData.grapic_data_struct.width = 1;
    Led_Purple.cilentData.grapic_data_struct.start_x = 1450; //1850
    Led_Purple.cilentData.grapic_data_struct.start_y = 510;  //640

    strcpy(Led_Purple.cilentData.data, LED_Init);
    strcpy(Led_Purple.cilentData.data, LED_PURPLEMeaning);

    memcpy(PurpleMeanBuffer + 5,
           (uint8_t *)&Led_Purple.CmdID,
           (sizeof(Led_Purple.CmdID) + sizeof(Led_Purple.dataFrameHeader) + sizeof(Led_Purple.cilentData)));
    Append_CRC16_Check_Sum(PurpleMeanBuffer, sizeof(Led_Purple));
    LEDMEAN_Datalength = sizeof(Led_Purple);
    HAL_UART_Transmit(&huart3, PurpleMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDPurple_MeanUpdate(void)
{
//    uint8_t LEDMEAN_Datalength;
//    Led_Purple.txFrameHeader.SOF = 0xA5;
//    Led_Purple.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//    Led_Purple.txFrameHeader.Seq = 0;
//    memcpy(PurpleMeanBuffer, &Led_Purple.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
//    Append_CRC8_Check_Sum(PurpleMeanBuffer, sizeof(xFrameHeader));

//    Led_Purple.CmdID = 0x0301;
//    determine_ID();
//    Led_Purple.dataFrameHeader.data_cmd_id = 0x0110;
//    Led_Purple.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
//    Led_Purple.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

//    Led_Purple.cilentData.grapic_data_struct.graphic_name[1] = 4;
//    Led_Purple.cilentData.grapic_data_struct.layer = 1;
//    Led_Purple.cilentData.grapic_data_struct.operate_tpye = 2;

//    if (VisionData.OffLineFlag)
//    {
//        strcpy(Led_Purple.cilentData.data, LED_PURPLEMeaning);
//    }
//    else
//    {
//        switch (Robot.Attack_ShootTarget)
//        {
//        case ShootTarget_default:
//            strcpy(Led_Purple.cilentData.data, "VisionMode:Default ");
//            break;

//        case ShootTarget_BIG_WHEEL:
//            strcpy(Led_Purple.cilentData.data, "VisionMode:BIG_WHEEL ");
//            break;

//        case ShootTarget_Sentry:
//            strcpy(Led_Purple.cilentData.data, "VisionMode:Sentry ");
//            break;

//        case ShootTarget_Spin:
//            strcpy(Led_Purple.cilentData.data, "VisionMode:Spin   ");
//            break;

//        case ShootTarget_Self_aiming:
//            strcpy(Led_Purple.cilentData.data, "VisionMode:Self_aiming ");
//            break;

//        case ShootTarget_Video:
//            strcpy(Led_Purple.cilentData.data, "VisionMode:Video ");
//            break;

//        case ShootTarget_UAV:
//            strcpy(Led_Purple.cilentData.data, "VisionMode:UAV ");
//            break;

//        case ShootTarget_OurSentry:
//            strcpy(Led_Purple.cilentData.data, "VisionMode:OurSentry ");
//            break;

//        case ShootTarget_Radar:
//            strcpy(Led_Purple.cilentData.data, "VisionMode:Radar ");
//            break;
//        }
//    }

//    memcpy(PurpleMeanBuffer + 5,
//           (uint8_t *)&Led_Purple.CmdID,
//           (sizeof(Led_Purple.CmdID) + sizeof(Led_Purple.dataFrameHeader) + sizeof(Led_Purple.cilentData)));
//    Append_CRC16_Check_Sum(PurpleMeanBuffer, sizeof(Led_Purple));
//    LEDMEAN_Datalength = sizeof(Led_Purple);
//    HAL_UART_Transmit(&huart3, PurpleMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDPurple_MeanDelete(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Purple.txFrameHeader.SOF = 0xA5;
    Led_Purple.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Purple.txFrameHeader.Seq = 0;
    memcpy(ColorMeanBuffer, &Led_Purple.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(ColorMeanBuffer, sizeof(xFrameHeader));

    Led_Purple.CmdID = 0x0301;
    determine_ID();
    Led_Purple.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Purple.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Purple.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Purple.cilentData.grapic_data_struct.graphic_name[1] = 4;
    Led_Purple.cilentData.grapic_data_struct.operate_tpye = 3;

    memcpy(ColorMeanBuffer + 5,
           (uint8_t *)&Led_Purple.CmdID,
           (sizeof(Led_Purple.CmdID) + sizeof(Led_Purple.dataFrameHeader) + sizeof(Led_Purple.cilentData)));
    Append_CRC16_Check_Sum(ColorMeanBuffer, sizeof(Led_Purple));
    LEDMEAN_Datalength = sizeof(Led_Purple);
    HAL_UART_Transmit(&huart3, ColorMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
unsigned char PinkMeanBuffer[100];
void LEDPink_Mean(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Pink.txFrameHeader.SOF = 0xA5;
    Led_Pink.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Pink.txFrameHeader.Seq = 0;
    memcpy(PinkMeanBuffer, &Led_Pink.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(PinkMeanBuffer, sizeof(xFrameHeader));

    Led_Pink.CmdID = 0x0301;
    determine_ID();
    Led_Pink.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Pink.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Pink.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Pink.cilentData.grapic_data_struct.graphic_name[1] = 5;
    Led_Pink.cilentData.grapic_data_struct.layer = 1;
    Led_Pink.cilentData.grapic_data_struct.operate_tpye = 1;
    Led_Pink.cilentData.grapic_data_struct.graphic_tpye = 7;
    Led_Pink.cilentData.grapic_data_struct.color = 1;
    Led_Pink.cilentData.grapic_data_struct.start_angle = 50;
    Led_Pink.cilentData.grapic_data_struct.end_angle = 16;
    Led_Pink.cilentData.grapic_data_struct.width = 5;
    Led_Pink.cilentData.grapic_data_struct.start_x = 730; //1850
    Led_Pink.cilentData.grapic_data_struct.start_y = 220;  //640

    strcpy(Led_Pink.cilentData.data, LED_Init);
    strcpy(Led_Pink.cilentData.data, LED_PINKMeaning);

    memcpy(PinkMeanBuffer + 5,
           (uint8_t *)&Led_Pink.CmdID,
           (sizeof(Led_Pink.CmdID) + sizeof(Led_Pink.dataFrameHeader) + sizeof(Led_Pink.cilentData)));
    Append_CRC16_Check_Sum(PinkMeanBuffer, sizeof(Led_Pink));
    LEDMEAN_Datalength = sizeof(Led_Pink);
    HAL_UART_Transmit(&huart3, PinkMeanBuffer, LEDMEAN_Datalength, 0xFF);
}

uint8_t star = 0;

void LEDPink_MeanUpdate(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Pink.txFrameHeader.SOF = 0xA5;
    Led_Pink.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Pink.txFrameHeader.Seq = 0;
    memcpy(PinkMeanBuffer, &Led_Pink.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(PinkMeanBuffer, sizeof(xFrameHeader));

    Led_Pink.CmdID = 0x0301;
    determine_ID();
    Led_Pink.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Pink.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Pink.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Pink.cilentData.grapic_data_struct.graphic_name[1] = 5;
    Led_Pink.cilentData.grapic_data_struct.operate_tpye = 2;
	
		if(rc.roll !=0)
		{
			strcpy(Led_Pink.cilentData.data, "   Spin       ");

		}
		else
		{
			if(star <= 40)
			{
				strcpy(Led_Pink.cilentData.data, "   NO!!       ");
				star ++;
			}
			else if (star > 40)
			{
				strcpy(Led_Pink.cilentData.data, "          ");
				star ++;
			}
			if(star >= 80 )
			{
				star = 0;
			}
		}

    memcpy(PinkMeanBuffer + 5,
           (uint8_t *)&Led_Pink.CmdID,
           (sizeof(Led_Pink.CmdID) + sizeof(Led_Pink.dataFrameHeader) + sizeof(Led_Pink.cilentData)));
    Append_CRC16_Check_Sum(PinkMeanBuffer, sizeof(Led_Pink));
    LEDMEAN_Datalength = sizeof(Led_Pink);
    HAL_UART_Transmit(&huart3, PinkMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDPink_MeanDelete(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Pink.txFrameHeader.SOF = 0xA5;
    Led_Pink.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Pink.txFrameHeader.Seq = 0;
    memcpy(ColorMeanBuffer, &Led_Pink.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(ColorMeanBuffer, sizeof(xFrameHeader));

    Led_Pink.CmdID = 0x0301;
    determine_ID();
    Led_Pink.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Pink.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Pink.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Pink.cilentData.grapic_data_struct.graphic_name[1] = 5;
    Led_Pink.cilentData.grapic_data_struct.operate_tpye = 3;

    memcpy(ColorMeanBuffer + 5,
           (uint8_t *)&Led_Pink.CmdID,
           (sizeof(Led_Pink.CmdID) + sizeof(Led_Pink.dataFrameHeader) + sizeof(Led_Pink.cilentData)));
    Append_CRC16_Check_Sum(ColorMeanBuffer, sizeof(Led_Pink));
    LEDMEAN_Datalength = sizeof(Led_Pink);
    HAL_UART_Transmit(&huart3, ColorMeanBuffer, LEDMEAN_Datalength, 0xFF);
}

void LEDCyan_Mean(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Meaning.txFrameHeader.SOF = 0xA5;
    Led_Meaning.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Meaning.txFrameHeader.Seq = 0;
    memcpy(ColorMeanBuffer, &Led_Meaning.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(ColorMeanBuffer, sizeof(xFrameHeader));

    Led_Meaning.CmdID = 0x0301;
    determine_ID();
    Led_Meaning.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Meaning.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Meaning.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Meaning.cilentData.grapic_data_struct.graphic_name[1] = 6;
    Led_Meaning.cilentData.grapic_data_struct.layer = 1;
    Led_Meaning.cilentData.grapic_data_struct.operate_tpye = 1;
    Led_Meaning.cilentData.grapic_data_struct.graphic_tpye = 7;
    Led_Meaning.cilentData.grapic_data_struct.color = 6;
    Led_Meaning.cilentData.grapic_data_struct.start_angle = 20;
    Led_Meaning.cilentData.grapic_data_struct.end_angle = 1;
    Led_Meaning.cilentData.grapic_data_struct.width = 3;
    Led_Meaning.cilentData.grapic_data_struct.start_x = 1700; //1850
    Led_Meaning.cilentData.grapic_data_struct.start_y = 560;  //640

    strcpy(Led_Meaning.cilentData.data, LED_Init);
    strcpy(Led_Meaning.cilentData.data, LED_CYANMeaning);

    memcpy(ColorMeanBuffer + 5,
           (uint8_t *)&Led_Meaning.CmdID,
           (sizeof(Led_Meaning.CmdID) + sizeof(Led_Meaning.dataFrameHeader) + sizeof(Led_Meaning.cilentData)));
    Append_CRC16_Check_Sum(ColorMeanBuffer, sizeof(Led_Meaning));
    LEDMEAN_Datalength = sizeof(Led_Meaning);
    HAL_UART_Transmit(&huart3, ColorMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDCyan_MeanDelete(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Meaning.txFrameHeader.SOF = 0xA5;
    Led_Meaning.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Meaning.txFrameHeader.Seq = 0;
    memcpy(ColorMeanBuffer, &Led_Meaning.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(ColorMeanBuffer, sizeof(xFrameHeader));

    Led_Meaning.CmdID = 0x0301;
    determine_ID();
    Led_Meaning.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Meaning.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Meaning.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Meaning.cilentData.grapic_data_struct.graphic_name[1] = 6;
    Led_Meaning.cilentData.grapic_data_struct.operate_tpye = 3;

    memcpy(ColorMeanBuffer + 5,
           (uint8_t *)&Led_Meaning.CmdID,
           (sizeof(Led_Meaning.CmdID) + sizeof(Led_Meaning.dataFrameHeader) + sizeof(Led_Meaning.cilentData)));
    Append_CRC16_Check_Sum(ColorMeanBuffer, sizeof(Led_Meaning));
    LEDMEAN_Datalength = sizeof(Led_Meaning);
    HAL_UART_Transmit(&huart3, ColorMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDBlack_Mean(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Meaning.txFrameHeader.SOF = 0xA5;
    Led_Meaning.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Meaning.txFrameHeader.Seq = 0;
    memcpy(ColorMeanBuffer, &Led_Meaning.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(ColorMeanBuffer, sizeof(xFrameHeader));

    Led_Meaning.CmdID = 0x0301;
    determine_ID();
    Led_Meaning.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Meaning.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Meaning.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Meaning.cilentData.grapic_data_struct.graphic_name[1] = 7;
    Led_Meaning.cilentData.grapic_data_struct.layer = 1;
    Led_Meaning.cilentData.grapic_data_struct.operate_tpye = 1;
    Led_Meaning.cilentData.grapic_data_struct.graphic_tpye = 7;
    Led_Meaning.cilentData.grapic_data_struct.color = 7;
    Led_Meaning.cilentData.grapic_data_struct.start_angle = 20;
    Led_Meaning.cilentData.grapic_data_struct.end_angle = 1;
    Led_Meaning.cilentData.grapic_data_struct.width = 3;
    Led_Meaning.cilentData.grapic_data_struct.start_x = 1700; //1850
    Led_Meaning.cilentData.grapic_data_struct.start_y = 510;  //640

    strcpy(Led_Meaning.cilentData.data, LED_Init);
    strcpy(Led_Meaning.cilentData.data, LED_BLACKMeaning);

    memcpy(ColorMeanBuffer + 5,
           (uint8_t *)&Led_Meaning.CmdID,
           (sizeof(Led_Meaning.CmdID) + sizeof(Led_Meaning.dataFrameHeader) + sizeof(Led_Meaning.cilentData)));
    Append_CRC16_Check_Sum(ColorMeanBuffer, sizeof(Led_Meaning));
    LEDMEAN_Datalength = sizeof(Led_Meaning);
    HAL_UART_Transmit(&huart3, ColorMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDBlack_MeanDelete(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Meaning.txFrameHeader.SOF = 0xA5;
    Led_Meaning.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Meaning.txFrameHeader.Seq = 0;
    memcpy(ColorMeanBuffer, &Led_Meaning.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(ColorMeanBuffer, sizeof(xFrameHeader));

    Led_Meaning.CmdID = 0x0301;
    determine_ID();
    Led_Meaning.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Meaning.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Meaning.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Meaning.cilentData.grapic_data_struct.graphic_name[1] = 7;
    Led_Meaning.cilentData.grapic_data_struct.operate_tpye = 3;

    memcpy(ColorMeanBuffer + 5,
           (uint8_t *)&Led_Meaning.CmdID,
           (sizeof(Led_Meaning.CmdID) + sizeof(Led_Meaning.dataFrameHeader) + sizeof(Led_Meaning.cilentData)));
    Append_CRC16_Check_Sum(ColorMeanBuffer, sizeof(Led_Meaning));
    LEDMEAN_Datalength = sizeof(Led_Meaning);
    HAL_UART_Transmit(&huart3, ColorMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDWhite_Mean(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Meaning.txFrameHeader.SOF = 0xA5;
    Led_Meaning.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Meaning.txFrameHeader.Seq = 0;
    memcpy(ColorMeanBuffer, &Led_Meaning.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(ColorMeanBuffer, sizeof(xFrameHeader));

    Led_Meaning.CmdID = 0x0301;
    determine_ID();
    Led_Meaning.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Meaning.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Meaning.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Meaning.cilentData.grapic_data_struct.graphic_name[1] = 8;
    Led_Meaning.cilentData.grapic_data_struct.layer = 1;
    Led_Meaning.cilentData.grapic_data_struct.operate_tpye = 1;
    Led_Meaning.cilentData.grapic_data_struct.graphic_tpye = 7;
    Led_Meaning.cilentData.grapic_data_struct.color = 8;
    Led_Meaning.cilentData.grapic_data_struct.start_angle = 20;
    Led_Meaning.cilentData.grapic_data_struct.end_angle = 1;
    Led_Meaning.cilentData.grapic_data_struct.width = 3;
    Led_Meaning.cilentData.grapic_data_struct.start_x = 1700; //1850
    Led_Meaning.cilentData.grapic_data_struct.start_y = 460;  //640

    strcpy(Led_Meaning.cilentData.data, LED_Init);
    strcpy(Led_Meaning.cilentData.data, LED_WHITEMeaning);

    memcpy(ColorMeanBuffer + 5,
           (uint8_t *)&Led_Meaning.CmdID,
           (sizeof(Led_Meaning.CmdID) + sizeof(Led_Meaning.dataFrameHeader) + sizeof(Led_Meaning.cilentData)));
    Append_CRC16_Check_Sum(ColorMeanBuffer, sizeof(Led_Meaning));
    LEDMEAN_Datalength = sizeof(Led_Meaning);
    HAL_UART_Transmit(&huart3, ColorMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
void LEDWhite_MeanDelete(void)
{
    uint8_t LEDMEAN_Datalength;
    Led_Meaning.txFrameHeader.SOF = 0xA5;
    Led_Meaning.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Led_Meaning.txFrameHeader.Seq = 0;
    memcpy(ColorMeanBuffer, &Led_Meaning.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(ColorMeanBuffer, sizeof(xFrameHeader));

    Led_Meaning.CmdID = 0x0301;
    determine_ID();
    Led_Meaning.dataFrameHeader.data_cmd_id = 0x0110;
    Led_Meaning.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Led_Meaning.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Led_Meaning.cilentData.grapic_data_struct.graphic_name[1] = 8;
    Led_Meaning.cilentData.grapic_data_struct.operate_tpye = 3;

    memcpy(ColorMeanBuffer + 5,
           (uint8_t *)&Led_Meaning.CmdID,
           (sizeof(Led_Meaning.CmdID) + sizeof(Led_Meaning.dataFrameHeader) + sizeof(Led_Meaning.cilentData)));
    Append_CRC16_Check_Sum(ColorMeanBuffer, sizeof(Led_Meaning));
    LEDMEAN_Datalength = sizeof(Led_Meaning);
    HAL_UART_Transmit(&huart3, ColorMeanBuffer, LEDMEAN_Datalength, 0xFF);
}
/*********************机器人状态信息总函数**************************/
/**黄色**/
void Yellow_ShowOn(void)
{
    LEDYellow_Mean();
}
void Yellow_DeleteAll(void)
{
    RobotStatus_YellowDelete();
    LEDYellow_MeanDelete();
}
/**绿色**/
void Green_ShowOn(void)
{
    LEDGreen_Mean();
}
void Green_DeleteAll(void)
{
    RobotStatus_GreenDelete();
    LEDGreen_MeanDelete();
}
/**橙色**/
void Orange_ShowOn(void)
{
    LEDOrange_Mean();
}
void Orange_DeleteAll(void)
{
    RobotStatus_OrangeDelete();
    LEDOrange_MeanDelete();
}
/**紫色**/
void Purple_ShowOn(void)
{
    LEDPurple_Mean();
}
void Purple_DeleteAll(void)
{
    RobotStatus_PurpleDelete();
    LEDPurple_MeanDelete();
}
/**粉色**/
void Pink_ShowOn(void)
{
    LEDPink_Mean();
}
void Pink_DeleteAll(void)
{
    RobotStatus_PinkDelete();
    LEDPink_MeanDelete();
}
/**青色**/
void Cyan_ShowOn(void)
{
    RobotStatus_LEDCyan();
    LEDCyan_Mean();
}
void Cyan_DeleteAll(void)
{
    RobotStatus_CyanDelete();
    LEDCyan_MeanDelete();
}
/**黑色**/
void Black_ShowOn(void)
{
    RobotStatus_LEDBlack();
    LEDBlack_Mean();
}
void Black_DeleteAll(void)
{
    RobotStatus_BlackDelete();
    LEDBlack_MeanDelete();
}
/**白色**/
void White_ShowOn(void)
{
    RobotStatus_LEDWhite();
    LEDWhite_Mean();
}
void White_DeleteAll(void)
{
    RobotStatus_WhiteDelete();
    LEDWhite_MeanDelete();
}

/*******************************************************************/
unsigned char CliendShowBuffer[100];
/*显示射速等级*/
uint8_t ShootLV_Datalength = 0;
void Show_ShootLevel(void)
{
    ShowshootLv.txFrameHeader.SOF = 0xA5;
    ShowshootLv.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    ShowshootLv.txFrameHeader.Seq = 0;
    memcpy(CliendShowBuffer, &ShowshootLv.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendShowBuffer, sizeof(xFrameHeader));

    ShowshootLv.CmdID = 0x0301;
    determine_ID();
    ShowshootLv.dataFrameHeader.data_cmd_id = 0x0110;
    ShowshootLv.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    ShowshootLv.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    ShowshootLv.cilentData.grapic_data_struct.graphic_name[0] = 16;
    ShowshootLv.cilentData.grapic_data_struct.layer = 2;
    ShowshootLv.cilentData.grapic_data_struct.operate_tpye = 1;
    ShowshootLv.cilentData.grapic_data_struct.graphic_tpye = 7;
    ShowshootLv.cilentData.grapic_data_struct.color = 8;
    ShowshootLv.cilentData.grapic_data_struct.start_angle = 25;
    ShowshootLv.cilentData.grapic_data_struct.end_angle = 9;
    ShowshootLv.cilentData.grapic_data_struct.width = 2;
    ShowshootLv.cilentData.grapic_data_struct.start_x = 10;
    ShowshootLv.cilentData.grapic_data_struct.start_y = 850;

    strcpy(ShowshootLv.cilentData.data, "Robot_LV:");

    memcpy(CliendShowBuffer + 5,
           (uint8_t *)&ShowshootLv.CmdID,
           (sizeof(ShowshootLv.CmdID) + sizeof(ShowshootLv.dataFrameHeader) + sizeof(ShowshootLv.cilentData)));
    Append_CRC16_Check_Sum(CliendShowBuffer, sizeof(ShowshootLv));
    ShootLV_Datalength = sizeof(ShowshootLv);
    HAL_UART_Transmit(&huart3, CliendShowBuffer, ShootLV_Datalength, 0xFF);
}
void ShootLv_Init(void)
{
    uint8_t ShootInit_Datalength;
    ShootLvInit.txFrameHeader.SOF = 0xA5;
    ShootLvInit.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    ShootLvInit.txFrameHeader.Seq = 0;
    memcpy(CliendShowBuffer, &ShootLvInit.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendShowBuffer, sizeof(xFrameHeader));

    ShootLvInit.CmdID = 0x0301;
    determine_ID();
    ShootLvInit.dataFrameHeader.data_cmd_id = 0x0110;
    ShootLvInit.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    ShootLvInit.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    ShootLvInit.cilentData.grapic_data_struct.graphic_name[0] = 17;
    ShootLvInit.cilentData.grapic_data_struct.layer = 2;
    ShootLvInit.cilentData.grapic_data_struct.operate_tpye = 1;
    ShootLvInit.cilentData.grapic_data_struct.graphic_tpye = 7;
    ShootLvInit.cilentData.grapic_data_struct.color = 0;
    ShootLvInit.cilentData.grapic_data_struct.start_angle = 25;
    ShootLvInit.cilentData.grapic_data_struct.end_angle = 1;
    ShootLvInit.cilentData.grapic_data_struct.width = 3;
    ShootLvInit.cilentData.grapic_data_struct.start_x = 220;
    ShootLvInit.cilentData.grapic_data_struct.start_y = 850;

    strcpy(ShootLvInit.cilentData.data, "0");

    memcpy(CliendShowBuffer + 5,
           (uint8_t *)&ShootLvInit.CmdID,
           (sizeof(ShootLvInit.CmdID) + sizeof(ShootLvInit.dataFrameHeader) + sizeof(ShootLvInit.cilentData)));
    Append_CRC16_Check_Sum(CliendShowBuffer, sizeof(ShootLvInit));
    ShootInit_Datalength = sizeof(ShootLvInit);
    HAL_UART_Transmit(&huart3, CliendShowBuffer, ShootInit_Datalength, 0xFF);
}

void ShootLv_show(void)
{
    Show_ShootLevel();
    ShootLv_Init();
}

void ShootLv_Updata(void)
{
    uint8_t ShootInit_Datalength;
    ShootLvInit.txFrameHeader.SOF = 0xA5;
    ShootLvInit.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    ShootLvInit.txFrameHeader.Seq = 0;
    memcpy(CliendShowBuffer, &ShootLvInit.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendShowBuffer, sizeof(xFrameHeader));

    ShootLvInit.CmdID = 0x0301;
    determine_ID();
    ShootLvInit.dataFrameHeader.data_cmd_id = 0x0110;
    ShootLvInit.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    ShootLvInit.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    ShootLvInit.cilentData.grapic_data_struct.graphic_name[0] = 17;
    ShootLvInit.cilentData.grapic_data_struct.layer = 2;
    ShootLvInit.cilentData.grapic_data_struct.operate_tpye = 2;
    ShootLvInit.cilentData.grapic_data_struct.graphic_tpye = 7;
    ShootLvInit.cilentData.grapic_data_struct.color = 0;
    ShootLvInit.cilentData.grapic_data_struct.start_angle = 25;
    ShootLvInit.cilentData.grapic_data_struct.end_angle = 1;
    ShootLvInit.cilentData.grapic_data_struct.width = 3;
    ShootLvInit.cilentData.grapic_data_struct.start_x = 220;
    ShootLvInit.cilentData.grapic_data_struct.start_y = 850;
    int_into_char(ShootLvInit.cilentData.data, ext_game_robot_state.data.robot_level);
    memcpy(CliendShowBuffer + 5,
           (uint8_t *)&ShootLvInit.CmdID,
           (sizeof(ShootLvInit.CmdID) + sizeof(ShootLvInit.dataFrameHeader) + sizeof(ShootLvInit.cilentData)));
    Append_CRC16_Check_Sum(CliendShowBuffer, sizeof(ShootLvInit));
    ShootInit_Datalength = sizeof(ShootLvInit);
    HAL_UART_Transmit(&huart3, CliendShowBuffer, ShootInit_Datalength, 0xFF);
}
/*超级电容
将int_into_char的第二个参数填上自己需要的参数
*/
unsigned char Cap_EnergyBuff[100];
void Capacitance_Value(void)
{
    static uint8_t value_datalength;
    Cap_Energyvalue.txFrameHeader.SOF = 0xA5;
    Cap_Energyvalue.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Cap_Energyvalue.txFrameHeader.Seq = 0;
    memcpy(Cap_EnergyBuff, &Cap_Energyvalue.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(Cap_EnergyBuff, sizeof(xFrameHeader));

    Cap_Energyvalue.CmdID = 0x0301;
    determine_ID();
    Cap_Energyvalue.dataFrameHeader.data_cmd_id = 0x0110;
    Cap_Energyvalue.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Cap_Energyvalue.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Cap_Energyvalue.cilentData.grapic_data_struct.graphic_name[0] = 19;
    Cap_Energyvalue.cilentData.grapic_data_struct.layer = 2;
    Cap_Energyvalue.cilentData.grapic_data_struct.operate_tpye = 1;
    Cap_Energyvalue.cilentData.grapic_data_struct.graphic_tpye = 7;
    Cap_Energyvalue.cilentData.grapic_data_struct.color = 1;
    Cap_Energyvalue.cilentData.grapic_data_struct.start_angle = 40;
    Cap_Energyvalue.cilentData.grapic_data_struct.end_angle = 4;
    Cap_Energyvalue.cilentData.grapic_data_struct.width = 4;
    Cap_Energyvalue.cilentData.grapic_data_struct.start_x = 250;
    Cap_Energyvalue.cilentData.grapic_data_struct.start_y = 650;
    int_into_char(Cap_Energyvalue.cilentData.data, Supcap_Rec.RecvData.Pack.cap_cell);
    strcat(Cap_Energyvalue.cilentData.data, "%");

    memcpy(Cap_EnergyBuff + 5,
           (uint8_t *)&Cap_Energyvalue.CmdID,
           (sizeof(Cap_Energyvalue.CmdID) + sizeof(Cap_Energyvalue.dataFrameHeader) + sizeof(Cap_Energyvalue.cilentData)));
    Append_CRC16_Check_Sum(Cap_EnergyBuff, sizeof(Cap_Energyvalue));
    value_datalength = sizeof(Cap_Energyvalue);
    HAL_UART_Transmit(&huart3, Cap_EnergyBuff, value_datalength, 0xFF);
}
//Cap_energy
void Cap_Energy(void)
{
    static uint8_t Cap_datalength;
    Cap_Energyshow.txFrameHeader.SOF = 0xA5;
    Cap_Energyshow.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Cap_Energyshow.txFrameHeader.Seq = 0;
    memcpy(Cap_EnergyBuff, &Cap_Energyshow.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(Cap_EnergyBuff, sizeof(xFrameHeader));

    Cap_Energyshow.CmdID = 0x0301;
    determine_ID();
    Cap_Energyshow.dataFrameHeader.data_cmd_id = 0x0110;
    Cap_Energyshow.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Cap_Energyshow.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Cap_Energyshow.cilentData.grapic_data_struct.graphic_name[0] = 20;
    Cap_Energyshow.cilentData.grapic_data_struct.layer = 2;
    Cap_Energyshow.cilentData.grapic_data_struct.operate_tpye = 1;
    Cap_Energyshow.cilentData.grapic_data_struct.graphic_tpye = 7;
    Cap_Energyshow.cilentData.grapic_data_struct.color = 8;
    Cap_Energyshow.cilentData.grapic_data_struct.start_angle = 25;
    Cap_Energyshow.cilentData.grapic_data_struct.end_angle = 12;
    Cap_Energyshow.cilentData.grapic_data_struct.width = 2;
    Cap_Energyshow.cilentData.grapic_data_struct.start_x = 10;
    Cap_Energyshow.cilentData.grapic_data_struct.start_y = 700;

    strcpy(Cap_Energyshow.cilentData.data, "CAP_ENGERGY:");

    memcpy(Cap_EnergyBuff + 5,
           (uint8_t *)&Cap_Energyshow.CmdID,
           (sizeof(Cap_Energyshow.CmdID) + sizeof(Cap_Energyshow.dataFrameHeader) + sizeof(Cap_Energyshow.cilentData)));
    Append_CRC16_Check_Sum(Cap_EnergyBuff, sizeof(Cap_Energyshow));
    Cap_datalength = sizeof(Cap_Energyshow);
    HAL_UART_Transmit(&huart3, Cap_EnergyBuff, Cap_datalength, 0xFF);
}
void CapEnergy_Show(void)
{
    Capacitance_Value();
    Cap_Energy();
}
void Cap_EnergyUpdata(void)
{
    uint8_t CapUpdata_Datalength;
    Cap_Energyvalue.txFrameHeader.SOF = 0xA5;
    Cap_Energyvalue.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Cap_Energyvalue.txFrameHeader.Seq = 0;
    memcpy(Cap_EnergyBuff, &Cap_Energyvalue.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(Cap_EnergyBuff, sizeof(xFrameHeader));

    Cap_Energyvalue.CmdID = 0x0301;
    determine_ID();
    Cap_Energyvalue.dataFrameHeader.data_cmd_id = 0x0110;
    Cap_Energyvalue.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Cap_Energyvalue.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Cap_Energyvalue.cilentData.grapic_data_struct.graphic_name[0] = 19;
    Cap_Energyvalue.cilentData.grapic_data_struct.layer = 2;
    Cap_Energyvalue.cilentData.grapic_data_struct.operate_tpye = 2;
    Cap_Energyvalue.cilentData.grapic_data_struct.graphic_tpye = 7;
    Cap_Energyvalue.cilentData.grapic_data_struct.color = 1;
    Cap_Energyvalue.cilentData.grapic_data_struct.start_angle = 40;
    Cap_Energyvalue.cilentData.grapic_data_struct.end_angle = 4;
    Cap_Energyvalue.cilentData.grapic_data_struct.width = 4;
    Cap_Energyvalue.cilentData.grapic_data_struct.start_x = 250;
    Cap_Energyvalue.cilentData.grapic_data_struct.start_y = 650;
//    if (Robot.Attack_ShootTarget == ShootTarget_BIG_WHEEL)
//    {
//        int_into_char(Cap_Energyvalue.cilentData.data, GetVision_CP() * 10);
//    }
//    else
//    {
        int_into_char(Cap_Energyvalue.cilentData.data, Supcap_Rec.RecvData.Pack.cap_cell);
//    }
    strcat(Cap_Energyvalue.cilentData.data, "%");

    memcpy(Cap_EnergyBuff + 5,
           (uint8_t *)&Cap_Energyvalue.CmdID,
           (sizeof(Cap_Energyvalue.CmdID) + sizeof(Cap_Energyvalue.dataFrameHeader) + sizeof(Cap_Energyvalue.cilentData)));
    Append_CRC16_Check_Sum(Cap_EnergyBuff, sizeof(Cap_Energyvalue));
    CapUpdata_Datalength = sizeof(Cap_Energyvalue);
    HAL_UART_Transmit(&huart3, Cap_EnergyBuff, CapUpdata_Datalength, 0xFF);
}
/*工程抬升高度*/
unsigned char Uplift_HeightBuff[100];
void UpliftHeight(void)
{
    static uint8_t Uplift_datalength;
    Uplift_Height.txFrameHeader.SOF = 0xA5;
    Uplift_Height.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Uplift_Height.txFrameHeader.Seq = 0;
    memcpy(Uplift_HeightBuff, &Uplift_Height.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(Uplift_HeightBuff, sizeof(xFrameHeader));

    Uplift_Height.CmdID = 0x0301;
    determine_ID();
    Uplift_Height.dataFrameHeader.data_cmd_id = 0x0110;
    Uplift_Height.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Uplift_Height.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Uplift_Height.cilentData.grapic_data_struct.graphic_name[1] = 16;
    Uplift_Height.cilentData.grapic_data_struct.layer = 2;
    Uplift_Height.cilentData.grapic_data_struct.operate_tpye = 1;
    Uplift_Height.cilentData.grapic_data_struct.graphic_tpye = 7;
    Uplift_Height.cilentData.grapic_data_struct.color = 8;
    Uplift_Height.cilentData.grapic_data_struct.start_angle = 25;
    Uplift_Height.cilentData.grapic_data_struct.end_angle = 14;
    Uplift_Height.cilentData.grapic_data_struct.width = 2;
    Uplift_Height.cilentData.grapic_data_struct.start_x = 10;
    Uplift_Height.cilentData.grapic_data_struct.start_y = 850;

    strcpy(Uplift_Height.cilentData.data, "Uplift_Height:");

    memcpy(Uplift_HeightBuff + 5,
           (uint8_t *)&Uplift_Height.CmdID,
           (sizeof(Uplift_Height.CmdID) + sizeof(Uplift_Height.dataFrameHeader) + sizeof(Uplift_Height.cilentData)));
    Append_CRC16_Check_Sum(Uplift_HeightBuff, sizeof(Uplift_Height));
    Uplift_datalength = sizeof(Uplift_Height);
    HAL_UART_Transmit(&huart3, Uplift_HeightBuff, Uplift_datalength, 0xFF);
}
void UpliftHeight_Value(void)
{
    static uint8_t Uplift_datalength;
    Uplift_HeightValue.txFrameHeader.SOF = 0xA5;
    Uplift_HeightValue.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Uplift_HeightValue.txFrameHeader.Seq = 0;
    memcpy(Uplift_HeightBuff, &Uplift_HeightValue.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(Uplift_HeightBuff, sizeof(xFrameHeader));

    Uplift_HeightValue.CmdID = 0x0301;
    determine_ID();
    Uplift_HeightValue.dataFrameHeader.data_cmd_id = 0x0110;
    Uplift_HeightValue.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Uplift_HeightValue.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Uplift_HeightValue.cilentData.grapic_data_struct.graphic_name[1] = 17;
    Uplift_HeightValue.cilentData.grapic_data_struct.layer = 2;
    Uplift_HeightValue.cilentData.grapic_data_struct.operate_tpye = 1;
    Uplift_HeightValue.cilentData.grapic_data_struct.graphic_tpye = 7;
    Uplift_HeightValue.cilentData.grapic_data_struct.color = 0;
    Uplift_HeightValue.cilentData.grapic_data_struct.start_angle = 25;
    Uplift_HeightValue.cilentData.grapic_data_struct.end_angle = 4;
    Uplift_HeightValue.cilentData.grapic_data_struct.width = 2;
    Uplift_HeightValue.cilentData.grapic_data_struct.start_x = 340;
    Uplift_HeightValue.cilentData.grapic_data_struct.start_y = 800;

    strcpy(Uplift_HeightValue.cilentData.data, "0");
    strcat(Uplift_HeightValue.cilentData.data, "cm");

    memcpy(Uplift_HeightBuff + 5,
           (uint8_t *)&Uplift_HeightValue.CmdID,
           (sizeof(Uplift_HeightValue.CmdID) + sizeof(Uplift_HeightValue.dataFrameHeader) + sizeof(Uplift_HeightValue.cilentData)));
    Append_CRC16_Check_Sum(Uplift_HeightBuff, sizeof(Uplift_HeightValue));
    Uplift_datalength = sizeof(Uplift_HeightValue);
    HAL_UART_Transmit(&huart3, Uplift_HeightBuff, Uplift_datalength, 0xFF);
}
void UpliftHeight_Show(void)
{
    UpliftHeight();
    UpliftHeight_Value();
}
void UpliftHeight_Updata(void)
{
    static uint8_t Uplift_datalength;
    Uplift_HeightValue.txFrameHeader.SOF = 0xA5;
    Uplift_HeightValue.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Uplift_HeightValue.txFrameHeader.Seq = 0;
    memcpy(Uplift_HeightBuff, &Uplift_HeightValue.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(Uplift_HeightBuff, sizeof(xFrameHeader));

    Uplift_HeightValue.CmdID = 0x0301;
    determine_ID();
    Uplift_HeightValue.dataFrameHeader.data_cmd_id = 0x0110;
    Uplift_HeightValue.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Uplift_HeightValue.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Uplift_HeightValue.cilentData.grapic_data_struct.graphic_name[1] = 17;
    Uplift_HeightValue.cilentData.grapic_data_struct.layer = 2;
    Uplift_HeightValue.cilentData.grapic_data_struct.operate_tpye = 2;
    Uplift_HeightValue.cilentData.grapic_data_struct.graphic_tpye = 7;
    Uplift_HeightValue.cilentData.grapic_data_struct.color = 0;
    Uplift_HeightValue.cilentData.grapic_data_struct.start_angle = 25;
    Uplift_HeightValue.cilentData.grapic_data_struct.end_angle = 4;
    Uplift_HeightValue.cilentData.grapic_data_struct.width = 2;
    Uplift_HeightValue.cilentData.grapic_data_struct.start_x = 340;
    Uplift_HeightValue.cilentData.grapic_data_struct.start_y = 800;

    int_into_char(Uplift_HeightValue.cilentData.data, 100); /*第二个参数改高度值*/
    strcat(Uplift_HeightValue.cilentData.data, "cm");

    memcpy(Uplift_HeightBuff + 5,
           (uint8_t *)&Uplift_HeightValue.CmdID,
           (sizeof(Uplift_HeightValue.CmdID) + sizeof(Uplift_HeightValue.dataFrameHeader) + sizeof(Uplift_HeightValue.cilentData)));
    Append_CRC16_Check_Sum(Uplift_HeightBuff, sizeof(Uplift_HeightValue));
    Uplift_datalength = sizeof(Uplift_HeightValue);
    HAL_UART_Transmit(&huart3, Uplift_HeightBuff, Uplift_datalength, 0xFF);
}
unsigned char Clip_AngleBuff[100];
/*工程爪子角度*/
void ClipAngle(void)
{
    static uint8_t Clip_datalength;
    Clip_Angle.txFrameHeader.SOF = 0xA5;
    Clip_Angle.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Clip_Angle.txFrameHeader.Seq = 0;
    memcpy(Clip_AngleBuff, &Clip_Angle.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(Clip_AngleBuff, sizeof(xFrameHeader));

    Clip_Angle.CmdID = 0x0301;
    determine_ID();
    Clip_Angle.dataFrameHeader.data_cmd_id = 0x0110;
    Clip_Angle.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Clip_Angle.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Clip_Angle.cilentData.grapic_data_struct.graphic_name[1] = 12;
    Clip_Angle.cilentData.grapic_data_struct.layer = 2;
    Clip_Angle.cilentData.grapic_data_struct.operate_tpye = 1;
    Clip_Angle.cilentData.grapic_data_struct.graphic_tpye = 7;
    Clip_Angle.cilentData.grapic_data_struct.color = 8;
    Clip_Angle.cilentData.grapic_data_struct.start_angle = 25;
    Clip_Angle.cilentData.grapic_data_struct.end_angle = 11;
    Clip_Angle.cilentData.grapic_data_struct.width = 2;
    Clip_Angle.cilentData.grapic_data_struct.start_x = 10;
    Clip_Angle.cilentData.grapic_data_struct.start_y = 700;

    strcpy(Clip_Angle.cilentData.data, "Clip_Angle:");

    memcpy(Clip_AngleBuff + 5,
           (uint8_t *)&Clip_Angle.CmdID,
           (sizeof(Clip_Angle.CmdID) + sizeof(Clip_Angle.dataFrameHeader) + sizeof(Clip_Angle.cilentData)));
    Append_CRC16_Check_Sum(Clip_AngleBuff, sizeof(Clip_Angle));
    Clip_datalength = sizeof(Clip_Angle);
    HAL_UART_Transmit(&huart3, Clip_AngleBuff, Clip_datalength, 0xFF);
}
void ClipAngle_Value(void)
{
    static uint8_t Clip_datalength;
    Clip_AngleValue.txFrameHeader.SOF = 0xA5;
    Clip_AngleValue.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Clip_AngleValue.txFrameHeader.Seq = 0;
    memcpy(Clip_AngleBuff, &Clip_AngleValue.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(Clip_AngleBuff, sizeof(xFrameHeader));

    Clip_AngleValue.CmdID = 0x0301;
    determine_ID();
    Clip_AngleValue.dataFrameHeader.data_cmd_id = 0x0110;
    Clip_AngleValue.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Clip_AngleValue.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Clip_AngleValue.cilentData.grapic_data_struct.graphic_name[1] = 13;
    Clip_AngleValue.cilentData.grapic_data_struct.layer = 2;
    Clip_AngleValue.cilentData.grapic_data_struct.operate_tpye = 1;
    Clip_AngleValue.cilentData.grapic_data_struct.graphic_tpye = 7;
    Clip_AngleValue.cilentData.grapic_data_struct.color = 5;
    Clip_AngleValue.cilentData.grapic_data_struct.start_angle = 25;
    Clip_AngleValue.cilentData.grapic_data_struct.end_angle = 11;
    Clip_AngleValue.cilentData.grapic_data_struct.width = 2;
    Clip_AngleValue.cilentData.grapic_data_struct.start_x = 285;
    Clip_AngleValue.cilentData.grapic_data_struct.start_y = 650;

    strcpy(Clip_AngleValue.cilentData.data, "0");
    strcat(Clip_AngleValue.cilentData.data, " deg");
    memcpy(Clip_AngleBuff + 5,
           (uint8_t *)&Clip_AngleValue.CmdID,
           (sizeof(Clip_AngleValue.CmdID) + sizeof(Clip_AngleValue.dataFrameHeader) + sizeof(Clip_AngleValue.cilentData)));
    Append_CRC16_Check_Sum(Clip_AngleBuff, sizeof(Clip_AngleValue));
    Clip_datalength = sizeof(Clip_AngleValue);
    HAL_UART_Transmit(&huart3, Clip_AngleBuff, Clip_datalength, 0xFF);
}
void ClipAngle_Show(void)
{
    ClipAngle();
    ClipAngle_Value();
}
void Clip_AngleUpdata(void)
{
    static uint8_t Clip_datalength;
    Clip_AngleValue.txFrameHeader.SOF = 0xA5;
    Clip_AngleValue.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Clip_AngleValue.txFrameHeader.Seq = 0;
    memcpy(Clip_AngleBuff, &Clip_AngleValue.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(Clip_AngleBuff, sizeof(xFrameHeader));

    Clip_AngleValue.CmdID = 0x0301;
    determine_ID();
    Clip_AngleValue.dataFrameHeader.data_cmd_id = 0x0110;
    Clip_AngleValue.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Clip_AngleValue.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Clip_AngleValue.cilentData.grapic_data_struct.graphic_name[1] = 13;
    Clip_AngleValue.cilentData.grapic_data_struct.layer = 2;
    Clip_AngleValue.cilentData.grapic_data_struct.operate_tpye = 2;
    Clip_AngleValue.cilentData.grapic_data_struct.graphic_tpye = 7;
    Clip_AngleValue.cilentData.grapic_data_struct.color = 5;
    Clip_AngleValue.cilentData.grapic_data_struct.start_angle = 25;
    Clip_AngleValue.cilentData.grapic_data_struct.end_angle = 11;
    Clip_AngleValue.cilentData.grapic_data_struct.width = 2;
    Clip_AngleValue.cilentData.grapic_data_struct.start_x = 285;
    Clip_AngleValue.cilentData.grapic_data_struct.start_y = 650;

    int_into_char(Clip_AngleValue.cilentData.data, 90); /*第二个参数改角度值*/
    strcat(Clip_AngleValue.cilentData.data, " deg");
    memcpy(Clip_AngleBuff + 5,
           (uint8_t *)&Clip_AngleValue.CmdID,
           (sizeof(Clip_AngleValue.CmdID) + sizeof(Clip_AngleValue.dataFrameHeader) + sizeof(Clip_AngleValue.cilentData)));
    Append_CRC16_Check_Sum(Clip_AngleBuff, sizeof(Clip_AngleValue));
    Clip_datalength = sizeof(Clip_AngleValue);
    HAL_UART_Transmit(&huart3, Clip_AngleBuff, Clip_datalength, 0xFF);
}

unsigned char Car_DistanBuff[150];
/*显示车距框*/
void Car_DistanceFrame(void)
{
    static uint8_t distanlength;

    Car_Distance.txFrameHeader.SOF = 0xA5;
    Car_Distance.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 5 * sizeof(ext_client_custom_graphic_single_t);
    Car_Distance.txFrameHeader.Seq = 0;
    memcpy(Car_DistanBuff, &Car_Distance.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(Car_DistanBuff, sizeof(xFrameHeader));               //写入帧头CRC8校验码

    Car_Distance.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID

    //ID已经是自动读取的了
    Car_Distance.dataFrameHeader.data_cmd_id = 0x0103;                           //客户端绘制一个图形
    Car_Distance.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    Car_Distance.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    Car_Distance.cilentData[0].graphic_name[0] = 1;
    Car_Distance.cilentData[0].graphic_tpye = 0;
    Car_Distance.cilentData[0].operate_tpye = 1;
    Car_Distance.cilentData[0].layer = 5;
    Car_Distance.cilentData[0].color = 2;
    Car_Distance.cilentData[0].start_angle = 0;
    Car_Distance.cilentData[0].end_angle = 0;
    Car_Distance.cilentData[0].width = 3;
    Car_Distance.cilentData[0].start_x = 450;
    Car_Distance.cilentData[0].start_y = 0;
    Car_Distance.cilentData[0].radius = 0;
    Car_Distance.cilentData[0].end_x = 600;
    Car_Distance.cilentData[0].end_y = 200;

    Car_Distance.cilentData[1].graphic_name[0] = 2;
    Car_Distance.cilentData[1].graphic_tpye = 0;
    Car_Distance.cilentData[1].operate_tpye = 1;
    Car_Distance.cilentData[1].layer = 5;
    Car_Distance.cilentData[1].color = 2;
    Car_Distance.cilentData[1].start_angle = 0;
    Car_Distance.cilentData[1].end_angle = 0;
    Car_Distance.cilentData[1].width = 3;
    Car_Distance.cilentData[1].start_x = 1480;
    Car_Distance.cilentData[1].start_y = 0;
    Car_Distance.cilentData[1].radius = 0;
    Car_Distance.cilentData[1].end_x = 1330;
    Car_Distance.cilentData[1].end_y = 200;

    Car_Distance.cilentData[2].graphic_name[0] = 3;
    Car_Distance.cilentData[2].graphic_tpye = 0;
    Car_Distance.cilentData[2].operate_tpye = 1;
    Car_Distance.cilentData[2].layer = 5;
    Car_Distance.cilentData[2].color = 2;
    Car_Distance.cilentData[2].start_angle = 0;
    Car_Distance.cilentData[2].end_angle = 0;
    Car_Distance.cilentData[2].width = 3;
    Car_Distance.cilentData[2].start_x = 600;
    Car_Distance.cilentData[2].start_y = 200;
    Car_Distance.cilentData[2].radius = 0;
    Car_Distance.cilentData[2].end_x = 800;
    Car_Distance.cilentData[2].end_y = 200;

    Car_Distance.cilentData[3].graphic_name[0] = 4;
    Car_Distance.cilentData[3].graphic_tpye = 0;
    Car_Distance.cilentData[3].operate_tpye = 1;
    Car_Distance.cilentData[3].layer = 5;
    Car_Distance.cilentData[3].color = 2;
    Car_Distance.cilentData[3].start_angle = 0;
    Car_Distance.cilentData[3].end_angle = 0;
    Car_Distance.cilentData[3].width = 3;
    Car_Distance.cilentData[3].start_x = 1330;
    Car_Distance.cilentData[3].start_y = 200;
    Car_Distance.cilentData[3].radius = 0;
    Car_Distance.cilentData[3].end_x = 1130;
    Car_Distance.cilentData[3].end_y = 200;

    memcpy(Car_DistanBuff + 5,
           (uint8_t *)&Car_Distance.CmdID,
           (sizeof(Car_Distance.CmdID) + sizeof(Car_Distance.dataFrameHeader) + sizeof(Car_Distance.cilentData)));
    Append_CRC16_Check_Sum(Car_DistanBuff, sizeof(Car_Distance)); //写入数据段CRC16校验码
    distanlength = sizeof(Car_Distance);
    HAL_UART_Transmit(&huart3, Car_DistanBuff, distanlength, 0xFF);
}

unsigned char SentryBuff[100];
void Sentry_Init(void)
{
    static uint8_t Sentry_datalength;
    Sentry_Status.txFrameHeader.SOF = 0xA5;
    Sentry_Status.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Sentry_Status.txFrameHeader.Seq = 0;
    memcpy(SentryBuff, &Sentry_Status.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(SentryBuff, sizeof(xFrameHeader));

    Sentry_Status.CmdID = 0x0301;
    determine_ID();
    Sentry_Status.dataFrameHeader.data_cmd_id = 0x0110;
    Sentry_Status.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Sentry_Status.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Sentry_Status.cilentData.grapic_data_struct.graphic_name[0] = 21;
    Sentry_Status.cilentData.grapic_data_struct.layer = 2;
    Sentry_Status.cilentData.grapic_data_struct.operate_tpye = 1;
    Sentry_Status.cilentData.grapic_data_struct.graphic_tpye = 7;
    Sentry_Status.cilentData.grapic_data_struct.color = 1;
    Sentry_Status.cilentData.grapic_data_struct.start_angle = 25;
    Sentry_Status.cilentData.grapic_data_struct.end_angle = 12;
    Sentry_Status.cilentData.grapic_data_struct.width = 2;
    Sentry_Status.cilentData.grapic_data_struct.start_x = 10;
    Sentry_Status.cilentData.grapic_data_struct.start_y = 550;

    strcpy(Sentry_Status.cilentData.data, "Senrty_Mode:");

    memcpy(SentryBuff + 5,
           (uint8_t *)&Sentry_Status.CmdID,
           (sizeof(Sentry_Status.CmdID) + sizeof(Sentry_Status.dataFrameHeader) + sizeof(Sentry_Status.cilentData)));
    Append_CRC16_Check_Sum(SentryBuff, sizeof(Sentry_Status));
    Sentry_datalength = sizeof(Sentry_Status);
    HAL_UART_Transmit(&huart3, SentryBuff, Sentry_datalength, 0xFF);
}
void Sentry_ModeShow(void)
{
    static uint8_t Sentry_datalength;
    SentryStatus_Data.txFrameHeader.SOF = 0xA5;
    SentryStatus_Data.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    SentryStatus_Data.txFrameHeader.Seq = 0;
    memcpy(SentryBuff, &SentryStatus_Data.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(SentryBuff, sizeof(xFrameHeader));

    SentryStatus_Data.CmdID = 0x0301;
    determine_ID();
    SentryStatus_Data.dataFrameHeader.data_cmd_id = 0x0110;
    SentryStatus_Data.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    SentryStatus_Data.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    SentryStatus_Data.cilentData.grapic_data_struct.graphic_name[0] = 22;
    SentryStatus_Data.cilentData.grapic_data_struct.layer = 2;
    SentryStatus_Data.cilentData.grapic_data_struct.operate_tpye = 1;
    SentryStatus_Data.cilentData.grapic_data_struct.graphic_tpye = 7;
    SentryStatus_Data.cilentData.grapic_data_struct.color = 5;
    SentryStatus_Data.cilentData.grapic_data_struct.start_angle = 20;
    SentryStatus_Data.cilentData.grapic_data_struct.end_angle = 12;
    SentryStatus_Data.cilentData.grapic_data_struct.width = 2;
    SentryStatus_Data.cilentData.grapic_data_struct.start_x = 10;
    SentryStatus_Data.cilentData.grapic_data_struct.start_y = 500;

    strcpy(SentryStatus_Data.cilentData.data, "Normal");

    memcpy(SentryBuff + 5,
           (uint8_t *)&SentryStatus_Data.CmdID,
           (sizeof(SentryStatus_Data.CmdID) + sizeof(SentryStatus_Data.dataFrameHeader) + sizeof(SentryStatus_Data.cilentData)));
    Append_CRC16_Check_Sum(SentryBuff, sizeof(SentryStatus_Data));
    Sentry_datalength = sizeof(SentryStatus_Data);
    HAL_UART_Transmit(&huart3, SentryBuff, Sentry_datalength, 0xFF);
}
void SentryStatus_Show(void)
{
    Sentry_Init();
    Sentry_ModeShow();
}
void SentryStatus_Updata(void)
{
    static uint8_t Sentry_datalength;
    SentryStatus_Data.txFrameHeader.SOF = 0xA5;
    SentryStatus_Data.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    SentryStatus_Data.txFrameHeader.Seq = 0;
    memcpy(SentryBuff, &SentryStatus_Data.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(SentryBuff, sizeof(xFrameHeader));

    SentryStatus_Data.CmdID = 0x0301;
    determine_ID();

    SentryStatus_Data.dataFrameHeader.data_cmd_id = 0x0110;
    SentryStatus_Data.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    SentryStatus_Data.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    SentryStatus_Data.cilentData.grapic_data_struct.graphic_name[0] = 22;
    SentryStatus_Data.cilentData.grapic_data_struct.operate_tpye = 2;
    if (SenrtyMode_Flag == 1)
    {
        strcpy(SentryStatus_Data.cilentData.data, "Normal");
    }
    else if (SenrtyMode_Flag == 2)
    {
        strcpy(SentryStatus_Data.cilentData.data, "Normal");
    }
    else if (SenrtyMode_Flag == 3)
    {
        strcpy(SentryStatus_Data.cilentData.data, "Normal");
    }
    else if (SenrtyMode_Flag == 4)
    {
        strcpy(SentryStatus_Data.cilentData.data, "Normal");
    }
    memcpy(SentryBuff + 5,
           (uint8_t *)&SentryStatus_Data.CmdID,
           (sizeof(SentryStatus_Data.CmdID) + sizeof(SentryStatus_Data.dataFrameHeader) + sizeof(SentryStatus_Data.cilentData)));
    Append_CRC16_Check_Sum(SentryBuff, sizeof(SentryStatus_Data));
    Sentry_datalength = sizeof(SentryStatus_Data);
    HAL_UART_Transmit(&huart3, SentryBuff, Sentry_datalength, 0xFF);
}

unsigned char DartBuff[100];
void Dart_Init(void)
{
    static uint8_t Dart_datalength;
    Dart_Status.txFrameHeader.SOF = 0xA5;
    Dart_Status.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    Dart_Status.txFrameHeader.Seq = 0;
    memcpy(DartBuff, &Dart_Status.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(DartBuff, sizeof(xFrameHeader));

    Dart_Status.CmdID = 0x0301;
    determine_ID();
    Dart_Status.dataFrameHeader.data_cmd_id = 0x0110;
    Dart_Status.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    Dart_Status.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    Dart_Status.cilentData.grapic_data_struct.graphic_name[0] = 23;
    Dart_Status.cilentData.grapic_data_struct.layer = 2;
    Dart_Status.cilentData.grapic_data_struct.operate_tpye = 1;
    Dart_Status.cilentData.grapic_data_struct.graphic_tpye = 7;
    Dart_Status.cilentData.grapic_data_struct.color = 1;
    Dart_Status.cilentData.grapic_data_struct.start_angle = 25;
    Dart_Status.cilentData.grapic_data_struct.end_angle = 12;
    Dart_Status.cilentData.grapic_data_struct.width = 2;
    Dart_Status.cilentData.grapic_data_struct.start_x = 10;
    Dart_Status.cilentData.grapic_data_struct.start_y = 550;

    strcpy(Dart_Status.cilentData.data, "Dart_Mode:");

    memcpy(DartBuff + 5,
           (uint8_t *)&Dart_Status.CmdID,
           (sizeof(Dart_Status.CmdID) + sizeof(Dart_Status.dataFrameHeader) + sizeof(Dart_Status.cilentData)));
    Append_CRC16_Check_Sum(DartBuff, sizeof(Dart_Status));
    Dart_datalength = sizeof(Dart_Status);
    HAL_UART_Transmit(&huart3, DartBuff, Dart_datalength, 0xFF);
}
void Dart_ModeShow(void)
{
    static uint8_t Dart_datalength;
    DartStatus_Data.txFrameHeader.SOF = 0xA5;
    DartStatus_Data.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    DartStatus_Data.txFrameHeader.Seq = 0;
    memcpy(DartBuff, &DartStatus_Data.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(DartBuff, sizeof(xFrameHeader));

    DartStatus_Data.CmdID = 0x0301;
    determine_ID();
    DartStatus_Data.dataFrameHeader.data_cmd_id = 0x0110;
    DartStatus_Data.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    DartStatus_Data.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    DartStatus_Data.cilentData.grapic_data_struct.graphic_name[0] = 24;
    DartStatus_Data.cilentData.grapic_data_struct.layer = 2;
    DartStatus_Data.cilentData.grapic_data_struct.operate_tpye = 1;
    DartStatus_Data.cilentData.grapic_data_struct.graphic_tpye = 7;
    DartStatus_Data.cilentData.grapic_data_struct.color = 1;
    DartStatus_Data.cilentData.grapic_data_struct.start_angle = 25;
    DartStatus_Data.cilentData.grapic_data_struct.end_angle = 12;
    DartStatus_Data.cilentData.grapic_data_struct.width = 2;
    DartStatus_Data.cilentData.grapic_data_struct.start_x = 10;
    DartStatus_Data.cilentData.grapic_data_struct.start_y = 500;

    strcpy(DartStatus_Data.cilentData.data, "Shoot");

    memcpy(DartBuff + 5,
           (uint8_t *)&DartStatus_Data.CmdID,
           (sizeof(DartStatus_Data.CmdID) + sizeof(DartStatus_Data.dataFrameHeader) + sizeof(DartStatus_Data.cilentData)));
    Append_CRC16_Check_Sum(DartBuff, sizeof(DartStatus_Data));
    Dart_datalength = sizeof(DartStatus_Data);
    HAL_UART_Transmit(&huart3, DartBuff, Dart_datalength, 0xFF);
}
void DartStatus_Show(void)
{
    Dart_Init();
    Dart_ModeShow();
}
void DartStatus_Updata(void)
{
    static uint8_t Dart_datalength;
    DartStatus_Data.txFrameHeader.SOF = 0xA5;
    DartStatus_Data.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
    DartStatus_Data.txFrameHeader.Seq = 0;
    memcpy(DartBuff, &DartStatus_Data.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(DartBuff, sizeof(xFrameHeader));

    DartStatus_Data.CmdID = 0x0301;
    determine_ID();
    DartStatus_Data.dataFrameHeader.data_cmd_id = 0x0110;
    DartStatus_Data.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;
    DartStatus_Data.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;

    DartStatus_Data.cilentData.grapic_data_struct.graphic_name[0] = 24;
    DartStatus_Data.cilentData.grapic_data_struct.layer = 2;
    DartStatus_Data.cilentData.grapic_data_struct.operate_tpye = 2;

    if (DartMode_Flag == 1)
    {
        strcpy(DartStatus_Data.cilentData.data, "Shoot");
    }
    else if (DartMode_Flag == 2)
    {
        strcpy(DartStatus_Data.cilentData.data, "Shoot");
    }
    else if (DartMode_Flag == 3)
    {
        strcpy(DartStatus_Data.cilentData.data, "Shoot");
    }

    memcpy(DartBuff + 5,
           (uint8_t *)&DartStatus_Data.CmdID,
           (sizeof(DartStatus_Data.CmdID) + sizeof(DartStatus_Data.dataFrameHeader) + sizeof(DartStatus_Data.cilentData)));
    Append_CRC16_Check_Sum(DartBuff, sizeof(DartStatus_Data));
    Dart_datalength = sizeof(DartStatus_Data);
    HAL_UART_Transmit(&huart3, DartBuff, Dart_datalength, 0xFF);
}

unsigned char Radar_ReceiveBuffer[40];
/*客户端接收信息*/
void Client_MapSend()
{
    static uint8_t Map_datalength;
    MapCommunate.txFrameHeader.SOF = 0xA5;
    MapCommunate.txFrameHeader.DataLength = sizeof(ext_client_map_command_t);
    MapCommunate.txFrameHeader.Seq = 0;
    memcpy(Radar_ReceiveBuffer, &MapCommunate.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(Radar_ReceiveBuffer, sizeof(xFrameHeader));

    MapCommunate.CmdID = 0x0305;

    MapCommunate.interactData.target_robot_ID = 4;
    MapCommunate.interactData.target_position_x = 5;
    MapCommunate.interactData.target_position_y = 6;
    MapCommunate.interactData.reserved = 1;

    memcpy(Radar_ReceiveBuffer + 5,
           (uint8_t *)&MapCommunate.CmdID,
           (sizeof(MapCommunate.CmdID) + sizeof(MapCommunate.interactData)));
    Append_CRC16_Check_Sum(Radar_ReceiveBuffer, sizeof(MapCommunate)); //写入数据段CRC16校验码
    Map_datalength = sizeof(MapCommunate);
    HAL_UART_Transmit(&huart3, Radar_ReceiveBuffer, Map_datalength, 0xFF);
}

/**
  * @brief  选择发送对象
  * @param  void
  * @retval Send_Color
  * @attention  
  */
void Send_Infantry(void)
{
    if (Send_Color == 0)
    {
        CommuData.dataFrameHeader.receiver_ID = 104;
    }
    else if (Send_Color == 1)
    {
        CommuData.dataFrameHeader.receiver_ID = 4;
    }
}

void Send_Hero(void)
{
    if (Send_Color == 0)
    {
        CommuData.dataFrameHeader.receiver_ID = 101;
    }
    else if (Send_Color == 1)
    {
        CommuData.dataFrameHeader.receiver_ID = 1;
    }
}

void Send_Sentry(void)
{
    if (Send_Color == 0)
    {
        CommuData.dataFrameHeader.receiver_ID = 107;
    }
    else if (Send_Color == 1)
    {
        CommuData.dataFrameHeader.receiver_ID = 7;
    }
}

void Send_Engineering(void)
{
    if (Send_Color == 0)
    {
        CommuData.dataFrameHeader.receiver_ID = 102;
    }
    else if (Send_Color == 1)
    {
        CommuData.dataFrameHeader.receiver_ID = 2;
    }
}

void Send_Dart(void)
{
    if (Send_Color == 0)
    {
        CommuData.dataFrameHeader.receiver_ID = 108;
    }
    else if (Send_Color == 1)
    {
        CommuData.dataFrameHeader.receiver_ID = 8;
    }
}

void Send_Radar(void)
{
    if (Send_Color == 0)
    {
        CommuData.dataFrameHeader.receiver_ID = 109;
    }
    else if (Send_Color == 1)
    {
        CommuData.dataFrameHeader.receiver_ID = 9;
    }
}

unsigned char SenrtyCommuBuff[40]; //(0x0250---)
void Judge_SendSentry(uint16_t cmd)
{
    static uint8_t datalength;
    Send_Color = is_red_or_blue();  //判断发送给兵种的颜色,107哨兵(蓝),7哨兵(红)；103'g'步兵蓝  3步兵红
    memset(SenrtyCommuBuff, 0, 40); //SenrtyCommuBuff

    Sentry_CommuData.txFrameHeader.SOF = 0xA5;
    Sentry_CommuData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
    Sentry_CommuData.txFrameHeader.Seq = 0x00;
    memcpy(SenrtyCommuBuff, &Sentry_CommuData.txFrameHeader, sizeof(xFrameHeader));
    Append_CRC8_Check_Sum(SenrtyCommuBuff, sizeof(xFrameHeader));

    Sentry_CommuData.CmdID = 0x0301;
    Sentry_CommuData.dataFrameHeader.sender_ID = Judge_Self_ID;
    Sentry_CommuData.dataFrameHeader.data_cmd_id = cmd; //裁判协议：在0x0200-0x02FF之间选择

    switch (cmd)
    {
    case 0x0250:
        Send_Sentry();
        Sentry_CommuData.interactData.data[0] = '1';
        break;
    case 0x0251:
        Send_Sentry();
        Sentry_CommuData.interactData.data[0] = '2';
        break;
    case 0x0252:
        Send_Sentry();
        Sentry_CommuData.interactData.data[0] = '3';
        break;
    case 0x0253:
        Send_Sentry();
        Sentry_CommuData.interactData.data[0] = '4';
        break;
    default:
        Sentry_CommuData.dataFrameHeader.receiver_ID = 88; //随便给个ID，不发送
        break;
    }

    memcpy(SenrtyCommuBuff + 5, (uint8_t *)&Sentry_CommuData.CmdID, (sizeof(Sentry_CommuData.CmdID) + sizeof(Sentry_CommuData.dataFrameHeader) + sizeof(Sentry_CommuData.interactData)));
    Append_CRC16_Check_Sum(SenrtyCommuBuff, sizeof(Sentry_CommuData));
    datalength = sizeof(Sentry_CommuData);
    HAL_UART_Transmit(&huart3, SenrtyCommuBuff, datalength, 0xFF);
}

unsigned char DartCommuBuff[40]; //(0x0220---)
void Judge_SendDart(uint16_t cmd)
{
    static uint8_t datalength;
    Send_Color = is_red_or_blue(); //判断发送给兵种的颜色,107哨兵(蓝),7哨兵(红)；103'g'步兵蓝  3步兵红
    memset(DartCommuBuff, 0, 40);  //DartCommuBuff

    Dart_CommuData.txFrameHeader.SOF = 0xA5;
    Dart_CommuData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
    Dart_CommuData.txFrameHeader.Seq = 0x00;
    memcpy(DartCommuBuff, &Dart_CommuData.txFrameHeader, sizeof(xFrameHeader));
    Append_CRC8_Check_Sum(DartCommuBuff, sizeof(xFrameHeader));

    Dart_CommuData.CmdID = 0x0301;
    Dart_CommuData.dataFrameHeader.sender_ID = Judge_Self_ID;
    Dart_CommuData.dataFrameHeader.data_cmd_id = cmd; //裁判协议：在0x0200-0x02FF之间选择

    switch (cmd)
    {
    case 0x0220:
        Send_Dart();
        Dart_CommuData.interactData.data[0] = '1';
        break;
    case 0x0221:
        Send_Dart();
        Dart_CommuData.interactData.data[0] = '2';
        break;
    case 0x0222:
        Send_Dart();
        Dart_CommuData.interactData.data[0] = '3';
        break;
    case 0x0223:
        Send_Dart();
        Dart_CommuData.interactData.data[0] = '4';
        break;
    default:
        Dart_CommuData.dataFrameHeader.receiver_ID = 88; //随便给个ID，不发送
        break;
    }

    memcpy(DartCommuBuff + 5, (uint8_t *)&Dart_CommuData.CmdID, (sizeof(Dart_CommuData.CmdID) + sizeof(Dart_CommuData.dataFrameHeader) + sizeof(Dart_CommuData.interactData)));
    Append_CRC16_Check_Sum(DartCommuBuff, sizeof(Dart_CommuData));
    datalength = sizeof(Dart_CommuData);
    HAL_UART_Transmit(&huart3, DartCommuBuff, datalength, 0xFF);
}

#endif

void Check_Judge(void)
{
    if (RM_Judge.InfoUpdateFrame < 1)
    {
        RM_Judge.OffLineFlag = 1;
    }
    else
    {
        RM_Judge.OffLineFlag = 0;
    }
    RM_Judge.InfoUpdateFrame = 0;
}
