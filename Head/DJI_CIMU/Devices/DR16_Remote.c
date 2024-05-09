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
  * @brief   DR16输出
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
  * @brief   键盘标志位更新
  * @param   void
  * @retval  void
  */
void KeyMouseFlag_Update(void)
{
    uint32_t KeyMouse = (uint32_t)DR16.keyBoard.key_code | DR16.mouse.keyLeft << 16 | DR16.mouse.keyRight << 17; // 把键盘鼠标的标志位合并。

    for (int Index = 0; Index < KEYMOUSE_AMOUNT; Index++) //遍历全部键位，更新他们的状态。
    {
        if (KeyMouse & (1 << Index)) //判断第index位是否为1。
        {
            DR16_Export_Data.KeyMouse.PressTime[Index]++;
            if (DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_Press) //满足按下的时间，视为按下
            {
                DR16_Export_Data.KeyMouse.Press_Flag |= 1 << Index; //设置该键的标志位为1
            }

            if (DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_LongPress) //长按判断
            {

                DR16_Export_Data.KeyMouse.Long_Press_Flag |= 1 << Index;  //设置长按标志位
            }
        }
        else
        {
            if ((DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_Press) && (DR16_Export_Data.KeyMouse.PressTime[Index] < TIME_KeyMouse_LongPress)) //时间处于两者之间，为单击。
            {
                DR16_Export_Data.KeyMouse.Click_Press_Flag |= 1 << Index; //设置单击标志位
            }
            else
            {
                DR16_Export_Data.KeyMouse.Click_Press_Flag &= ~(1 << Index); //取反操作，将该键的标志位设为0
            }

            //已经松开，将按下标志位置空
            DR16_Export_Data.KeyMouse.Press_Flag &= ~(1 << Index);
            DR16_Export_Data.KeyMouse.Long_Press_Flag &= ~(1 << Index);
            DR16_Export_Data.KeyMouse.PressTime[Index] = 0;
        }
    }
}

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
  * @brief   DR16解码
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

    //your control code ….
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
	* @brief  检测遥控数据（遥控器掉线数据并没有出错，所以加多一个0判断，防止掉线重启mcu，但是比赛时需要去掉这个判断）
  * @param	void
  * @retval int   0:数据没有出错      1：数据出错了
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

/*一个是读取SR寄存器，一个是读取对应的CR控制寄存器*/
/*正常情况下CR控制SR，我们入股要读取对应的标志位的话，既可以从CR读取也可以从SR读取*/
/*__HAL_UART_GET_FLAG是获取SR寄存器里面的情况，也就是读取被CR控制器控制之后的对应状态*/
/*__HAL_UART_GET_IT_SOURCE是直接读取控制寄存器里面的CRx标志位的情况*/
/*这里的DMA_GET_COUNTER是获取还没发出去的字符数量，和之前的不同*/
/*下面是两种情况的对比，请仔细阅读*/
/**
  * @Data    2021/3/26
  * @brief   DR16处理函数
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
  * @brief   DR16处理函数 
  * @param   UART_HandleTypeDef *huart
  * @retval  void
  */
void DR16_USART_Receive_DMA(UART_HandleTypeDef *huartx)
{
    /*清空标志位然后使能USART的中断*/
    __HAL_UART_CLEAR_IDLEFLAG(huartx);
    __HAL_UART_ENABLE(huartx);
    __HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);
    // assert(DR16BufferNumber == 22);
    USART_Receive_DMA_NO_IT(huartx, DR16.DR16Buffer, DR16BufferNumber);
}

/**
	* @brief  获取鼠标键盘某个键当前的动作
  * @param	键值  动作
  * @retval 返回键鼠动作状态  0 没有该动作 1 有该动作
  */
bool GetKeyMouseAction(KeyList_e KeyMouse, KeyAction_e Action)
{
    uint8_t action = 0;
    switch (Action)
    {
    case KeyAction_CLICK: //单击

        action = ((DR16_Export_Data.KeyMouse.Click_Press_Flag >> KeyMouse) & 1);
        break;
    case KeyAction_PRESS: //按下
        action = ((DR16_Export_Data.KeyMouse.Press_Flag >> KeyMouse) & 1);
        break;
    case KeyAction_LONG_PRESS: //长按
        action = ((DR16_Export_Data.KeyMouse.Long_Press_Flag >> KeyMouse) & 1);
        break;
    default:
        action = 0;
        break;
    }
    return action;
}

/**
 * @brief DR16检测
 * 
 */
void Check_DR16(void)
{
    //遥控器 ---------------------------
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
 * @brief 射速偏移是否擦除
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
