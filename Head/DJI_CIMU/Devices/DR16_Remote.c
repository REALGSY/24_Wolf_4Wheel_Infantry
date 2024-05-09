/**
 * @file DR16_Remote.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "DR16_Remote.h"
void DR16_Handler(UART_HandleTypeDef *huart);
void DR16_USART_Receive_DMA(UART_HandleTypeDef *huartx);
bool GetKeyMouseAction(KeyList_e KeyMouse, KeyAction_e Action);
void RemoteControl_Output(void);
int DR16_DataCheck(void); 
void Check_DR16(void);

ControlSwitch_t ControlSwitch;

DR16_Export_Data_t DR16_Export_Data = DR16_ExportDataGroundInit;
#undef DR16_ExportDataGroundInit
DR16_t DR16 = DR16_GroundInit;
#undef DR16_GroundInit
DR16_Fun_t DR16_Fun = DR16_FunGroundInit;
#undef DR16_FunGroundInit

/**
  * @Data    2021/3/30
  * @brief   DR16���
  * @param   void
  * @retval  void
  */
void RemoteControl_Output(void)
{
		if(abs(DR16.rc.ch3) <10)
		{
			DR16.rc.ch3 = 0;
		}
    DR16_Export_Data.Robot_TargetValue.Forward_Back_Value = DR16.rc.ch3;
    DR16_Export_Data.Robot_TargetValue.Left_Right_Value = DR16.rc.ch2;
    DR16_Export_Data.Robot_TargetValue.Omega_Value = DR16.rc.ch4_DW *1.5f;
    DR16_Export_Data.Robot_TargetValue.Pitch_Value = DR16.rc.ch1;
		if(abs(DR16.rc.ch0) <15)
		{
			DR16.rc.ch0 = 0;
		}
    DR16_Export_Data.Robot_TargetValue.Yaw_Value = DR16.rc.ch0;
    DR16_Export_Data.Robot_TargetValue.Dial_Wheel = DR16.rc.ch4_DW;
    DR16_Export_Data.ControlSwitch->Left = (RemotePole_e)DR16.rc.s_left;
    DR16_Export_Data.ControlSwitch->Right = (RemotePole_e)DR16.rc.s_right;
    DR16_Export_Data.mouse.x = (float)DR16.mouse.x * 0.05f;
    DR16_Export_Data.mouse.y = (float)DR16.mouse.y * 0.6f;

    // RemoteMode_Update();
}

/**
  * @Data    2021/3/30
  * @brief   ���̱�־λ����
  * @param   void
  * @retval  void
  */
void KeyMouseFlag_Update(void)
{
    uint32_t KeyMouse = (uint32_t)DR16.keyBoard.key_code | DR16.mouse.keyLeft << 16 | DR16.mouse.keyRight << 17; // �Ѽ������ı�־λ�ϲ���

    for (int Index = 0; Index < KEYMOUSE_AMOUNT; Index++) //����ȫ����λ���������ǵ�״̬��
    {
        if (KeyMouse & (1 << Index)) //�жϵ�indexλ�Ƿ�Ϊ1��
        {
            DR16_Export_Data.KeyMouse.PressTime[Index]++;
            if (DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_Press) //���㰴�µ�ʱ�䣬��Ϊ����
            {
                DR16_Export_Data.KeyMouse.Press_Flag |= 1 << Index; //���øü��ı�־λΪ1
            }

            if (DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_LongPress) //�����ж�
            {

                DR16_Export_Data.KeyMouse.Long_Press_Flag |= 1 << Index;  //���ó�����־λ
            }
        }
        else
        {
            if ((DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_Press) && (DR16_Export_Data.KeyMouse.PressTime[Index] < TIME_KeyMouse_LongPress)) //ʱ�䴦������֮�䣬Ϊ������
            {
                DR16_Export_Data.KeyMouse.Click_Press_Flag |= 1 << Index; //���õ�����־λ
            }
            else
            {
                DR16_Export_Data.KeyMouse.Click_Press_Flag &= ~(1 << Index); //ȡ�����������ü��ı�־λ��Ϊ0
            }

            //�Ѿ��ɿ��������±�־λ�ÿ�
            DR16_Export_Data.KeyMouse.Press_Flag &= ~(1 << Index);
            DR16_Export_Data.KeyMouse.Long_Press_Flag &= ~(1 << Index);
            DR16_Export_Data.KeyMouse.PressTime[Index] = 0;
        }
    }
}

/**
 * @brief USART_DMA���տ������ض���
 * 
 * @param huart 
 * @param pData 
 * @param Size 
 * @return  
 */
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{

    /*��⵱ǰhuart״̬*/
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        /*����ĵ�ַ��������������Ļ�*/
        if ((pData == NULL) || (Size == 0))
        {
            return HAL_ERROR;
        }

         /*huart�����Ӧ��Rx�����ض���*/
        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        /*����huart1�ϵ�RX_DMA*/
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

        /*ֻ������ӦDMA�����Rx���ܣ�����ǿ���Tx�Ļ�����USART_CR3_DMAT��*/
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
  * @brief   DR16����
  * @param   void
  * @retval  void
  */
static void DR16_Process(uint8_t *pData)
{
    if (pData == NULL)
    {
        return;
    }
    DR16.rc.ch0 = (pData[0] | (pData[1] << 8)) & 0x07FF;
    DR16.rc.ch1 = ((pData[1] >> 3) | (pData[2] << 5)) & 0x07FF;
    DR16.rc.ch2 = ((pData[2] >> 6) | (pData[3] << 2) | (pData[4] << 10)) & 0x07FF;
    DR16.rc.ch3 = ((pData[4] >> 1) | (pData[5] << 7)) & 0x07FF;
    DR16.rc.s_left = ((pData[5] >> 4) & 0x000C) >> 2;
    DR16.rc.s_right = ((pData[5] >> 4) & 0x0003);
    DR16.mouse.x = (pData[6]) | (pData[7] << 8);
    DR16.mouse.y = (pData[8]) | (pData[9] << 8);
    DR16.mouse.z = (pData[10]) | (pData[11] << 8);
    DR16.mouse.keyLeft = pData[12];
    DR16.mouse.keyRight = pData[13];
    DR16.keyBoard.key_code = pData[14] | (pData[15] << 8);

    //your control code ��.
    DR16.rc.ch4_DW = (pData[16] | (pData[17] << 8)) & 0x07FF;
    // DR16.infoUpdateFrame++;
    DR16_Export_Data.infoUpdateFrame++;

    DR16.rc.ch0 -= 1024;
    DR16.rc.ch1 -= 1024;
    DR16.rc.ch2 -= 1024;
    DR16.rc.ch3 -= 1024;
    DR16.rc.ch4_DW -= 1024;

    /* prevent remote control zero deviation */
    if (DR16.rc.ch0 <= 5 && DR16.rc.ch0 >= -5)
        DR16.rc.ch0 = 0;
    if (DR16.rc.ch1 <= 5 && DR16.rc.ch1 >= -5)
        DR16.rc.ch1 = 0;
    if (DR16.rc.ch2 <= 5 && DR16.rc.ch2 >= -5)
        DR16.rc.ch2 = 0;
    if (DR16.rc.ch3 <= 5 && DR16.rc.ch3 >= -5)
        DR16.rc.ch3 = 0;
    if (DR16.rc.ch4_DW <= 5 && DR16.rc.ch4_DW >= -5)
        DR16.rc.ch4_DW = 0;

    DR16_Export_Data.ControlSwitch->Left = (RemotePole_e)DR16.rc.s_left;
    DR16_Export_Data.ControlSwitch->Right = (RemotePole_e)DR16.rc.s_right;
    DR16_Export_Data.mouse.x = (float)DR16.mouse.x * 0.05f;
    DR16_Export_Data.mouse.y = (float)DR16.mouse.y * 0.6f;
		
    KeyMouseFlag_Update();
}

/**
	* @brief  ���ң�����ݣ�ң�����������ݲ�û�г������ԼӶ�һ��0�жϣ���ֹ��������mcu�����Ǳ���ʱ��Ҫȥ������жϣ�
  * @param	void
  * @retval int   0:����û�г���      1�����ݳ�����
  */
int DR16_DataCheck(void)
{
    if ((DR16.rc.s_left != RemotePole_UP && DR16.rc.s_left != RemotePole_MID && DR16.rc.s_left != RemotePole_DOWM && DR16.rc.s_left != 0)        
        || (DR16.rc.s_right != RemotePole_UP && DR16.rc.s_right != RemotePole_MID && DR16.rc.s_right != RemotePole_DOWM && DR16.rc.s_right != 0) 
        || (DR16.rc.ch0 > 660 || DR16.rc.ch0 < -660)                                                                                          
        || (DR16.rc.ch1 > 660 || DR16.rc.ch1 < -660)                                                                                        
        || (DR16.rc.ch2 > 660 || DR16.rc.ch2 < -660)                                                                                        
        || (DR16.rc.ch3 > 660 || DR16.rc.ch3 < -660)                                                                                           
        || (DR16.rc.ch4_DW > 660 || DR16.rc.ch4_DW < -660))                                                                                     
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*һ���Ƕ�ȡSR�Ĵ�����һ���Ƕ�ȡ��Ӧ��CR���ƼĴ���*/
/*���������CR����SR���������Ҫ��ȡ��Ӧ�ı�־λ�Ļ����ȿ��Դ�CR��ȡҲ���Դ�SR��ȡ*/
/*__HAL_UART_GET_FLAG�ǻ�ȡSR�Ĵ�������������Ҳ���Ƕ�ȡ��CR����������֮��Ķ�Ӧ״̬*/
/*__HAL_UART_GET_IT_SOURCE��ֱ�Ӷ�ȡ���ƼĴ��������CRx��־λ�����*/
/*�����DMA_GET_COUNTER�ǻ�ȡ��û����ȥ���ַ���������֮ǰ�Ĳ�ͬ*/
/*��������������ĶԱȣ�����ϸ�Ķ�*/
/**
  * @Data    2021/3/26
  * @brief   DR16������
  * @param   UART_HandleTypeDef *huart
  * @retval  void
  */
void DR16_Handler(UART_HandleTypeDef *huart)
{
    __HAL_DMA_DISABLE(huart->hdmarx);

    //if(DR16BufferNumber - DMA_GET_COUNTER(huart->hdmarx->Instance) == DR16BufferTruthNumber)
    if (__HAL_DMA_GET_COUNTER(huart->hdmarx) == DR16BufferLastNumber)
    {
        DR16_Process(DR16.DR16Buffer);
    }
    __HAL_DMA_SET_COUNTER(huart->hdmarx, DR16BufferNumber);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/**
  * @Data    2021/3/26
  * @brief   DR16������ 
  * @param   UART_HandleTypeDef *huart
  * @retval  void
  */
void DR16_USART_Receive_DMA(UART_HandleTypeDef *huartx)
{
    /*��ձ�־λȻ��ʹ��USART���ж�*/
    __HAL_UART_CLEAR_IDLEFLAG(huartx);
    __HAL_UART_ENABLE(huartx);
    __HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);
    // assert(DR16BufferNumber == 22);
    USART_Receive_DMA_NO_IT(huartx, DR16.DR16Buffer, DR16BufferNumber);
}

/**
	* @brief  ��ȡ������ĳ������ǰ�Ķ���
  * @param	��ֵ  ����
  * @retval ���ؼ�����״̬  0 û�иö��� 1 �иö���
  */
bool GetKeyMouseAction(KeyList_e KeyMouse, KeyAction_e Action)
{
    uint8_t action = 0;
    switch (Action)
    {
    case KeyAction_CLICK: //����

        action = ((DR16_Export_Data.KeyMouse.Click_Press_Flag >> KeyMouse) & 1);
        break;
    case KeyAction_PRESS: //����
        action = ((DR16_Export_Data.KeyMouse.Press_Flag >> KeyMouse) & 1);
        break;
    case KeyAction_LONG_PRESS: //����
        action = ((DR16_Export_Data.KeyMouse.Long_Press_Flag >> KeyMouse) & 1);
        break;
    default:
        action = 0;
        break;
    }
    return action;
}

/**
 * @brief DR16���
 * 
 */
void Check_DR16(void)
{
    //ң���� ---------------------------
    if (DR16_Export_Data.infoUpdateFrame < 1)
    {
        DR16_Export_Data.OffLineFlag = 1;
    }
    else
    {
        DR16_Export_Data.OffLineFlag = 0;
    }
    DR16_Export_Data.infoUpdateFrame = 0;
}

/**
 * @brief ����ƫ���Ƿ����
 * 
 * @return  
 */
bool Calibration_Shoot(void)
{
    if ((DR16.rc.ch0 > 400 && DR16.rc.ch1 < -400 && DR16.rc.ch2 < -400 && DR16.rc.ch3 < -400)
        || (DR16_Fun.GetKeyMouseAction(KEY_C,KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_V,KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_B,KeyAction_PRESS)))
    {
        return true;
    }
    else
    {
        return false;
    }
}
