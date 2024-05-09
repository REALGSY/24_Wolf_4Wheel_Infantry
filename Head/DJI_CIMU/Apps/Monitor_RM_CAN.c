/**
 * @file Monitor_RM_CAN.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Monitor_RM_CAN.h"
#include "BSP_CAN.h"
#include "can.h"
#include "Handle.h"


/*******************************用户数据定义************************************/
static CAN_TxHeaderTypeDef        TxMessage;    //CAN发送的消息的消息头
/*******************************************************************************/


/**
  * @Data   2022-06-30
  * @brief  发送CAN消息(CAN1 : 1 - 4)
  * @param  CAN_FilterTypeDef *CAN_Filter, CAN_HandleTypeDef *hcanx
  * @retval void
  */
  void Motor_0x200_SendData (int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
{
		
	uint8_t Data[8] = {0};
		
  TxMessage.IDE = CAN_ID_STD;     //设置ID类型
	TxMessage.RTR = CAN_RTR_DATA;   //设置传送数据帧
	TxMessage.DLC = 0x08;              //设置数据长度
	TxMessage.StdId = 0x200;        //设置ID号
	
	Data[0] = motor1 >>8;
	Data[1] = motor1;
	Data[2] = motor2 >>8;
	Data[3] = motor2;
	Data[4] = motor3 >>8;
	Data[5] = motor3;
	Data[6] = motor4 >>8;
	Data[7] = motor4;
		
    
    HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Data,0);
   
}

  void Motor_0x200_Can2_SendData (int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
{
		
	uint8_t Data[8] = {0};
		
  TxMessage.IDE = CAN_ID_STD;     //设置ID类型
	TxMessage.RTR = CAN_RTR_DATA;   //设置传送数据帧
	TxMessage.DLC = 0x08;              //设置数据长度
	TxMessage.StdId = 0x200;        //设置ID号
	
	Data[0] = motor1 >>8;
	Data[1] = motor1;
	Data[2] = motor2 >>8;
	Data[3] = motor2;
	Data[4] = motor3 >>8;
	Data[5] = motor3;
	Data[6] = motor4 >>8;
	Data[7] = motor4;
		
    
    HAL_CAN_AddTxMessage(&hcan2,&TxMessage,Data,0);
   
}

/*******************************************************************************/
/**
  * @Data   2022-06-30
  * @brief  发送CAN消息(CAN1 : 5 - 8)
  * @param  CAN_FilterTypeDef *CAN_Filter, CAN_HandleTypeDef *hcanx
  * @retval void
  */

void Motor_0x1FF_SendData(int16_t motor5,int16_t motor6,int16_t motor7,int16_t motor8)
{
	uint8_t DATA[8] = {0};
		
  TxMessage.IDE = CAN_ID_STD;     //设置ID类型
	TxMessage.RTR = CAN_RTR_DATA;   //设置传送数据帧
	TxMessage.DLC = 0x08;              //设置数据长度
	TxMessage.StdId = 0x1FF;        //设置ID号
	
	DATA[0] = motor5 >>8;
	DATA[1] = motor5;
	DATA[2] = motor6 >>8;
	DATA[3] = motor6;
	DATA[4] = motor7 >>8;
	DATA[5] = motor7;
	DATA[6] = motor8 >>8;
	DATA[7] = motor8;
		
    HAL_CAN_AddTxMessage(&hcan1,&TxMessage,DATA,0);
}

/*******************************************************************************/
/**
  * @Data   2022-06-30
  * @brief  发送CAN消息(CAN1 : 5 - 8)
  * @param  CAN_FilterTypeDef *CAN_Filter, CAN_HandleTypeDef *hcanx
  * @retval void
  */

void Motor_0x2FF_SendData(int16_t motor5,int16_t motor6,int16_t motor7,int16_t motor8)
{
	uint8_t DATA[8] = {0};
		
  TxMessage.IDE = CAN_ID_STD;     //设置ID类型
	TxMessage.RTR = CAN_RTR_DATA;   //设置传送数据帧
	TxMessage.DLC = 0x08;              //设置数据长度
	TxMessage.StdId = 0x2FF;        //设置ID号
	
	DATA[0] = motor5 >>8;
	DATA[1] = motor5;
	DATA[2] = motor6 >>8;
	DATA[3] = motor6;
	DATA[4] = motor7 >>8;
	DATA[5] = motor7;
	DATA[6] = motor8 >>8;
	DATA[7] = motor8;
		
    HAL_CAN_AddTxMessage(&hcan1,&TxMessage,DATA,0);
}


/*******************************************************************************/
/**
  * @Data   2022-06-30
  * @brief  发送CAN消息(CAN1 : 5 - 8)
  * @param  CAN_FilterTypeDef *CAN_Filter, CAN_HandleTypeDef *hcanx
  * @retval void
  */

void Motor_0x1FF_Can2_SendData(int16_t motor5,int16_t motor6,int16_t motor7,int16_t motor8)
{
	uint8_t DATA[8] = {0};
		
  TxMessage.IDE = CAN_ID_STD;     //设置ID类型
	TxMessage.RTR = CAN_RTR_DATA;   //设置传送数据帧
	TxMessage.DLC = 0x08;              //设置数据长度
	TxMessage.StdId = 0x1FF;        //设置ID号
	
	DATA[0] = motor5 >>8;
	DATA[1] = motor5;
	DATA[2] = motor6 >>8;
	DATA[3] = motor6;
	DATA[4] = motor7 >>8;
	DATA[5] = motor7;
	DATA[6] = motor8 >>8;
	DATA[7] = motor8;
		
  HAL_CAN_AddTxMessage(&hcan2,&TxMessage,DATA,0);
}

void DR16_0x175_Can1_SendData(int16_t Vx,int16_t Vy,int16_t Vw,int16_t mode)
{
	uint8_t DATA[8] = {0};
		
  TxMessage.IDE = CAN_ID_STD;     //设置ID类型
	TxMessage.RTR = CAN_RTR_DATA;   //设置传送数据帧
	TxMessage.DLC = 0x08;              //设置数据长度
	TxMessage.StdId = 0x175;        //设置ID号
	
	DATA[0] = Vx >>8;
	DATA[1] = Vx;
	DATA[2] = Vy >>8;
	DATA[3] = Vy;
	DATA[4] = Vw >>8;
	DATA[5] = Vw;
	DATA[6] = mode >>8;
	DATA[7] = mode;
		
  HAL_CAN_AddTxMessage(&hcan1,&TxMessage,DATA,0);
}

void DR16_0x185_Can1_SendData(int16_t Vx,int16_t Vy,int16_t Vw,int16_t mode)
{
	uint8_t DATA[8] = {0};
		
  TxMessage.IDE = CAN_ID_STD;     //设置ID类型
	TxMessage.RTR = CAN_RTR_DATA;   //设置传送数据帧
	TxMessage.DLC = 0x08;              //设置数据长度
	TxMessage.StdId = 0x185;        //设置ID号
	
	DATA[0] = Vx >>8;
	DATA[1] = Vx;
	DATA[2] = Vy >>8;
	DATA[3] = Vy;
	DATA[4] = Vw >>8;
	DATA[5] = Vw;
	DATA[6] = mode >>8;
	DATA[7] = mode;
		
  HAL_CAN_AddTxMessage(&hcan1,&TxMessage,DATA,0);
}
