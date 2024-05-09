/**
 * @file BSP_CAN.c
 * @author Gsy 
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "BSP_CAN.h"
#include "can.h"
#include "Handle.h"
#include "typedef.h"
#include "stm32f4xx_hal_can.h"
/*******************************�û����ݶ���************************************/
CAN_HandleTypeDef CAN1_Handler; //CAN1���
CAN_RxHeaderTypeDef CAN_Rx_Heade;
uint8_t CAN_RxMessage[8];
Can_Data_t Can_Data[2] = Can_DataGroundInit;
/*******************************************************************************/
/**
  * @Data   2022-06-30
  * @brief  CANɸѡ����ʼ��
  * @param  CAN_FilterTypeDef *CAN_Filter, CAN_HandleTypeDef *hcanx
  * @retval void
  */

  void CAN_1_Filter_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;
    
  sFilterConfig.FilterBank = 0;                       //CAN��������ţ���Χ0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN������ģʽ������ģʽ���б�ģʽ
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN�������߶ȣ�16λ��32λ
  sFilterConfig.FilterIdHigh = 0x0000;			//32λ�£��洢Ҫ����ID�ĸ�16λ
  sFilterConfig.FilterIdLow = 0x0000;					//32λ�£��洢Ҫ����ID�ĵ�16λ
  sFilterConfig.FilterMaskIdHigh = 0x0000;			//����ģʽ�£��洢��������
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;				//����ͨ����������ƥ��󣬴洢���ĸ�FIFO
  sFilterConfig.FilterActivation = ENABLE;    		//���������
  //sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	HAL_CAN_Start(&hcan1) ;
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

 void CAN_2_Filter_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;

	sFilterConfig.SlaveStartFilterBank = 14;      //�¼ӵ�
  sFilterConfig.FilterBank = 14;                       //CAN��������ţ���Χ0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN������ģʽ������ģʽ���б�ģʽ
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN�������߶ȣ�16λ��32λ
  sFilterConfig.FilterIdHigh = 0x0000;			//32λ�£��洢Ҫ����ID�ĸ�16λ
  sFilterConfig.FilterIdLow = 0x0000;					//32λ�£��洢Ҫ����ID�ĵ�16λ
  sFilterConfig.FilterMaskIdHigh = 0x0000;			//����ģʽ�£��洢��������
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;				//����ͨ����������ƥ��󣬴洢���ĸ�FIFO
  sFilterConfig.FilterActivation = ENABLE;    		//���������
	
	
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	HAL_CAN_Start(&hcan2) ;
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	
}

/*******************************************************************************/
/**
  * @Data   2022-06-30
  * @brief  ����CAN��Ϣ���������
  * @param  CAN_FilterTypeDef *CAN_Filter, CAN_HandleTypeDef *hcanx
  * @retval void
  */

void CAN_RxMessage_Export_Date(CAN_HandleTypeDef *hcanx, osMessageQId CANx_Handle, uint8_t Can_type)
{
    Can_Export_Data_t Can_Export_Data[2];
    uint8_t Canx_type = Can_type - 1;
    HAL_CAN_GetRxMessage(hcanx, CAN_RX_FIFO0,
                         &Can_Data[Canx_type].CAN_RxTypedef.CANx_RxHeader,
                         Can_Data[Canx_type].CAN_RxTypedef.CAN_RxMessage);

    Can_Export_Data[Canx_type].CAN_RxHeader = Can_Data[Canx_type].CAN_RxTypedef.CANx_RxHeader;
    memcpy(&Can_Export_Data[Canx_type].CANx_Export_RxMessage,
           Can_Data[Canx_type].CAN_RxTypedef.CAN_RxMessage,
           sizeof(uint8_t[8]));

    xQueueSendToBackFromISR(CANx_Handle, &Can_Export_Data[Canx_type], 0); //�ѽ������ݷ������ն���
}

