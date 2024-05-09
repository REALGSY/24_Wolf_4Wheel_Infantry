/**
 * @file DR16_Remote.c
 * @author Gsy
 * @brief 
 * @version 1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
 
#include "string.h"
#include "stdlib.h"
#include "DR16_Remote.h"
#include "usart.h"
#include "main.h"
#include "can.h"
#include "Robot_control.h"
#include "Dev_SupCap.h"
#include "RM_JudgeSystem.h"
/*******************************用户数据定义************************************/
uint8_t   dbus_buf[DBUS_BUFLEN];
rc_info_t rc = rc_Init;
ControlSwitch_t ControlSwitch;
DR16_Export_Data_t DR16_Export_Data = DR16_ExportDataGroundInit;
/*******************************************************************************/

void RemoteControl_Output(void)
{
	DR16_Export_Data.ControlSwitch->Left  = (RemotePole_e)rc.sw1;
	DR16_Export_Data.ControlSwitch->Right = (RemotePole_e)rc.sw2;
	DR16_Export_Data.Robot_TargetValue.Forward_Back_Value = rc.ch3;
	DR16_Export_Data.Robot_TargetValue.Left_Right_Value = rc.ch2;
	DR16_Export_Data.Robot_TargetValue.Omega_Value = rc.ch0;
	DR16_Export_Data.Robot_TargetValue.Pitch_Value = rc.ch1;
	DR16_Export_Data.Robot_TargetValue.Yaw_Value = rc.ch0;
	DR16_Export_Data.infoUpdateFrame = rc.infoUpdateFrame;
}

static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;

  tmp1 = huart->RxState;
	
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

	
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  return ((uint16_t)(dma_stream->NDTR));
}


void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
  rc->ch0 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc->ch0 -= 1024;
  rc->ch1 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch3 -= 1024;
  rc->roll = (buff[16] | (buff[17] << 8)) & 0x07FF;  //左上角滚轮
  rc->roll -= 1024;

  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;
  
	rc->infoUpdateFrame++;
	
  if ((abs(rc->ch0) > 660) || \
      (abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
	  (abs(rc->roll) > 660))
	  
  {
    memset(rc, 0, sizeof(rc_info_t));
  }		
}
void DR16_CAN2_REC(Can_Export_Data_t RxMessage)
{
	rc.infoUpdateFrame++;
	rc.ch2=(int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
	rc.ch3=(int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
	rc.ch0=(int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
	Robot.ChassisWorkMode  = (int16_t)(RxMessage.CANx_Export_RxMessage[6] << 8 | RxMessage.CANx_Export_RxMessage[7]);
//	if(ext_game_robot_state.data.mains_power_chassis_output == 0)
//	{
//		Robot.ChassisWorkMode = ChassisWorkMode_Disable;
//	}
}
void DR16_ROLL_REC(Can_Export_Data_t RxMessage)
{
	rc.roll=(int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
	Robot.Device_FrictMode = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
	SupCap_C_Flag = (uint16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
}
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	
	__HAL_UART_CLEAR_IDLEFLAG(huart);

	
	if (huart == &DBUS_HUART)
	{
		__HAL_DMA_DISABLE(huart->hdmarx);

		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{
			rc_callback_handler(&rc, dbus_buf);	
		}
		
		
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

void uart_receive_handler(UART_HandleTypeDef *huart)
{  
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}


void dbus_uart_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);

	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}


void Check_DR16(void)
{
	if(rc.infoUpdateFrame < 1)
	{
		rc.OffLineFlag = 1;
	}
	else
	{
		rc.OffLineFlag = 0;
	}
	rc.infoUpdateFrame = 0;
}
