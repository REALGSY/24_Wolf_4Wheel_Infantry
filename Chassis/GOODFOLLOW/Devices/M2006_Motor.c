/**
 * @file M2006_Motor.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "M2006_Motor.h"

M2006s_t M2006_Reload_1; 
M2006s_t M2006_Reload_2; 
M2006s_t M2006_Reload_3; //����������
M2006s_t *M2006_Array[] = {&M2006_Reload_1,&M2006_Reload_2,&M2006_Reload_3};
#define M2006_Amount 1 //��Ӧ���档
void M2006_getInfo(Can_Export_Data_t RxMessage);
void Check_M2006(void);
M2006_FUN_t M2006_FUN = M2006_FunGroundInit;
#undef M2006_FunGroundInit
/**
  * @brief  ����M2006�������ֵ��id��Ϊ7��M2006��6623���÷��ͺ���
  * @param  iqx (x:5) ��Ӧid�ŵ���ĵ���ֵ����Χ-10000~0~10000
  * @retval None
  */
// void M2006_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
// {

// 	uint8_t data[8];

// 	data[0] = iq1 >> 8;
// 	data[1] = iq1;
// 	data[2] = iq2 >> 8;
// 	data[3] = iq2;
// 	data[4] = iq3 >> 8;
// 	data[5] = iq3;
// 	data[6] = iq4 >> 8;
// 	data[7] = iq4;
// 	CAN_SendData(&hcan2, CAN_ID_STD, M2006_SENDID, data);
// }

/**
  * @brief  ��CAN�����л�ȡM2006�����Ϣ
  * @param[in]  RxMessage 	CAN���Ľ��սṹ��
  * @retval None
  */

void M2006_getInfo(Can_Export_Data_t RxMessage)
{
    //����idȷ��
    // if ((RxMessage.CAN_RxHeader.StdId < M2006_READID_START) || (RxMessage.CAN_RxHeader.StdId > M2006_READID_END))
    // 	return;
    int32_t StdId;
    StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - M2006_READID_START;
    // if (IndexOutofBounds(StdId, M2006_Amount))
    // {
    // 	Device_setAlertType(Alert_Times_SoftWare);
    // 	return;
    // }
    M2006_Array[StdId]->lastAngle = M2006_Array[StdId]->realAngle;
    //������ݣ����ݸ�ʽ���C610���˵����P9
    M2006_Array[StdId]->realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    M2006_Array[StdId]->realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    M2006_Array[StdId]->realTorque = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);

    if (M2006_Array[StdId]->realAngle - M2006_Array[StdId]->lastAngle < -6000)
    {
        M2006_Array[StdId]->turnCount++;
    }

    if (M2006_Array[StdId]->lastAngle - M2006_Array[StdId]->realAngle < -6000)
    {
        M2006_Array[StdId]->turnCount--;
    }
    M2006_Array[StdId]->totalAngle = M2006_Array[StdId]->realAngle + (8192 * M2006_Array[StdId]->turnCount);
    M2006_Array[StdId]->lastAngle = M2006_Array[StdId]->realAngle;
    M2006_Array[StdId]->InfoUpdateFrame++;
    M2006_Array[StdId]->InfoUpdateFlag = 1;
}

void Check_M2006(void)
{
    for (uint8_t i = 0; i < M2006_Amount; i++)
    {
        if (M2006_Array[i]->InfoUpdateFrame < 1)
        {
            M2006_Array[i]->OffLineFlag = 1;
        }
        else
        {
            M2006_Array[i]->OffLineFlag = 0;
        }
        M2006_Array[i]->InfoUpdateFrame = 0;
    }
}
