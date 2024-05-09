/**
 * @file Control_Vision.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Control_Vision.h"
#include "cmsis_os.h"
#include "M6020_Motor.h"
#include "CRC.h"
#include <stdio.h>
#include "DJI_IMU.h"
#include "RM_Message.h"
#include "Control_Vision.h"
#include "usbd_cdc_if.h"
uint8_t Vision_DataBuff[Vision_BuffSIZE];
VisionData_t VisionData;
VisionSend_IMU_t VisionSend_IMU;
UART_HandleTypeDef *vision_uart;
VisionSendMsg_u Send_Msg;
//WorldTime_RxTypedef Vision_WorldTime;
//WorldTime_RxTypedef VisionKF_TIME;

//extKalman_t Vision_depthKalman;

int robot_id = 1;
void Vision_Init(void);
void Update_VisionTarget(void);
void Vision_processing(void);
void Vision_ID_Init(void);
void Vision_Handler(UART_HandleTypeDef *huart);
void Vision_USART_Receive_DMA(UART_HandleTypeDef *huartx);
void Vision_SendBufFunction(float *angle, float *Gyro);
uint8_t GetVisionDiscMode(void);
uint8_t GetVisionHitMode(void);
void Check_Vision(void);
int16_t Vision_getOffset(int16_t length, int16_t Value);
void Vision_CP(float CPData);
VisionExportData_t VisionExportData = VisionExportDataGroundInit;
#undef VisionExportDataGroundInit
Control_Vision_FUN_t Control_Vision_FUN = Control_Vision_FUNGroundInit ;
#undef Control_Vision_FUNGroundInit


uint8_t Vision_SendBuf[9][16] = {'S', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
                                 'S', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
                                 'S', '0', '2', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
                                 'S', '0', '3', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
                                 'S', '0', '4', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
                                 'S', '0', '5', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
                                 'S', '0', '6', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
                                 'S', '0', '7', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
                                 'S', '0', '8', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E'};

static void Vision_I_T_Set(uint8_t ID, uint8_t Type)
{
    for (uint8_t n = 0; n < 9; n++)
    {
        Vision_SendBuf[n][1] = ID;
        Vision_SendBuf[n][3] = Type;
    }
}

//发给上位机相关 暂时堆大便
static const uint16_t wCRC_Table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e,
    0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64,
    0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e,
    0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e,
    0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44,
    0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e,
    0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e,
    0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324,
    0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e,
    0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e,
    0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704,
    0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e,
    0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1,
    0x0f78
};
  uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL) {
        return 0xFFFF;
    }
    while(dwLength--) {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

void Vision_ID_Init(void)
{
    switch (robot_id)
    {
    case 1:
        Vision_I_T_Set('1', '1');
        break;

    case 2:
        Vision_I_T_Set('1', '2');
        break;

    case 3:
        Vision_I_T_Set('1', '3');
        break;

    case 4:
        Vision_I_T_Set('1', '3');
        break;

    case 5:
        Vision_I_T_Set('1', '3');
        break;

    case 6:
        Vision_I_T_Set('1', '6');
        break;

    case 7:
        *Vision_SendBuf[1] = '1';
        Vision_I_T_Set('1', '7');
        break;

    case 101:
        Vision_I_T_Set('2', '1');
        break;

    case 102:
        Vision_I_T_Set('2', '2');
        break;

    case 103:
        Vision_I_T_Set('2', '3');
        break;

    case 104:
        Vision_I_T_Set('2', '3');
        break;

    case 105:
        Vision_I_T_Set('2', '3');
        break;

    case 106:
        Vision_I_T_Set('2', '6');
        break;

    case 107:
        Vision_I_T_Set('2', '7');
        break;
    }
}


void Vision_Init(void)
{

		VisionData.OffLineFlag = 0;
    VisionData.Gravity_Offset.x = 0;
    VisionData.Gravity_Offset.y = 0;
    //VisionData.Gravity_Offset.y = 60;

    VisionData.LpfAttFactor.y = 0.1;
    VisionData.LpfAttFactor.x = 0.9;

    VisionData.SpeedGain.y = 0.0; //1.2
    VisionData.SpeedGain.x = 0.0; //-2.2

    VisionExportData.FinalOffset_Last.x = 0;
    VisionExportData.FinalOffset_Last.y = 0;

}


void Vision_processing(void)
{

    //对当前数据进行判断，是否可以进行射击
    if (VisionData.RawData.auto_aim)
    {
        VisionExportData.AbleToFire = true;
    }

//    if (VisionData.RawData.mode == 0 || VisionData.RawData.depth == 0  || VisionData.OffLineFlag == 1) //视觉模块失能或找不到目标
//    {
    if (VisionData.OffLineFlag == 1) //视觉模块失能或找不到目标
    {
        VisionExportData.FinalOffset_Last.x = 0;
        VisionExportData.FinalOffset_Last.y = 0;
        VisionExportData.FinalOffset.x = 0;
        VisionExportData.FinalOffset.y = 0;
        VisionExportData.FinalOffset_depth = 0;

        VisionExportData.FinalOffset_depth = 1000.0f;

        VisionExportData.AbleToFire = false;
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
        /*	VisionData.Final_Offset.x = 0;
			VisionData.Final_Offset.y = 0;

			VisionData.ErrorChange_Rate.x = 0;
			VisionData.ErrorChange_Rate.y = 0;*/
        //注意：这里不能清零VisionData.Target_Offset 否则会造成ec值错误！

        return;
    }

    //对当前误差进行运算，进行自瞄运动。
    // --- 数据是否更新
//    if (VisionData.InfoUpdateFlag != 0)
//    {
//        VisionExportData.FinalOffset.x = VisionData.Final_Offset.x * M6020_mAngleRatio;
//        if (Robot.Attack_ShootTarget == ShootTarget_BIG_WHEEL)
//        {
//            VisionExportData.FinalOffset.y = VisionData.Final_Offset.y * M6020_mAngleRatio;
//        }
//        else
//        {
//            VisionExportData.FinalOffset.y = (VisionData.Final_Offset.y + VisionData.Gravity_Offset.y) * M6020_mAngleRatio;
//        }

//        VisionKF_TIME.WorldTime = xTaskGetTickCount();

//        VisionData.InfoUpdateFlag = 0; // 清除标志位
//    }

//    VisionExportData.FinalOffset_depth = KalmanFilter(&Vision_depthKalman, VisionData.RawData.depth);

//    VisionFourth_Ring_Yaw.Speed_Gain = VisionData.SpeedGain.x * imu_Export.SpeedLPF[0];
//    VisionFourth_Ring_Pitch.Speed_Gain = VisionData.SpeedGain.y * M6020s_Pitch.realSpeed;
//    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);

    //if (VisionData.RawData.whether_Fire == 1)
    //{
    //	Buzzer_On(true);
    //}
    //else
    //{
    //	Buzzer_On(false);
    //}
}

/**
 * @brief 视觉接收
 * 
 * @param data 
 */
uint8_t CRCBuffer;
uint16_t nan_jiajia;
void Vision_DataReceive(uint8_t *data)
{
		//--- 不校验帧头帧尾以及crc位
  \
#if Vision_Version == Vision_Oldversion
    //	sscanf(data, "%c%1d%1d%04d%03d%04d%03d%c", &VisionData.RawData.Start_Tag, &VisionData.RawData.mode, &VisionData.RawData.mode_select, &VisionData.RawData.x, &VisionData.RawData.y, &VisionData.RawData.depth, &VisionData.RawData.crc, &VisionData.RawData.End_Tag);
    sscanf(data, "%c%1d%1d%1d%04d%1d%03d%04d%03d%c", &VisionData.RawData.Start_Tag, &VisionData.RawData.mode, &VisionData.RawData.whether_Fire, &VisionData.RawData._yaw,
           &VisionData.RawData.x, &VisionData.RawData._pitch, &VisionData.RawData.y, &VisionData.RawData.depth, &VisionData.RawData.crc, &VisionData.RawData.End_Tag);

//    if (/*CRCBuffer != VisionData.RawData.crc ||*/ VisionData.RawData.Start_Tag != 'S' || VisionData.RawData.End_Tag != 'E') //CRC校验失败。
//    {
//        return;
//    }

    VisionData.InfoUpdateFrame++;

    //Get_FPS(&Vision_WorldTime, &VisionData.FPS);

    if (VisionData.RawData.mode == 0 || VisionData.RawData.depth == 0 || VisionData.OffLineFlag == 1) //视觉模块失能或找不到目标
    {
        VisionExportData.FinalOffset_Last.x = 0;
        VisionExportData.FinalOffset_Last.y = 0;
        VisionExportData.FinalOffset.x = 0;
        VisionExportData.FinalOffset.y = 0;
        VisionData.Final_Offset.x = 0;
        VisionData.Final_Offset.y = 0;

        VisionData.ErrorChange_Rate.x = 0;
        VisionData.ErrorChange_Rate.y = 0;
        //注意：这里不能清零VisionData.Target_Offset 否则会造成ec值错误！
        VisionData.Target_Offset.x = VisionPage_Width / 2;
        VisionData.Target_Offset.y = VisionPage_Height / 2;
        VisionExportData.AbleToFire = false;
        return;
    }

    //坐标转成本次的偏差。
    VisionData.Target_Offset.x = Vision_getOffset(VisionPage_Width, VisionData.RawData.x);
    VisionData.Target_Offset.y = Vision_getOffset(VisionPage_Height, VisionData.RawData.y);

    if (VisionData.RawData.depth != 0) //防止从 丢失 变 找到 出现ec 错误！！！
    {
        //计算偏差变化率。
        VisionData.ErrorChange_Rate.x = VisionData.Target_Offset.x - VisionData.Target_LastOffset.x;
        VisionData.ErrorChange_Rate.y = VisionData.Target_Offset.y - VisionData.Target_LastOffset.y;

        //保存上一次的偏差。
        VisionData.Target_LastOffset.x = VisionData.Target_Offset.x;
        VisionData.Target_LastOffset.y = VisionData.Target_Offset.y;
    }
    else
    {
        VisionData.ErrorChange_Rate.x = 0;
        VisionData.ErrorChange_Rate.y = 0;
        VisionData.Target_LastOffset.x = VisionData.Target_Offset.x;
        VisionData.Target_LastOffset.y = VisionData.Target_Offset.y;
    }

    VisionData.Gravity_Offset.y =
        /*(pow(VisionData.RawData.depth, 3) *0.0004088 -9'
				
				+
			0.2795 * pow(VisionData.RawData.depth, 2) +
			74.2*VisionData.RawData.depth - 1316) / (VisionData.RawData.depth - 30.77)+*/
        150;

    //确定最终用于计算的偏差值
    VisionData.Final_Offset.x = VisionData.Target_Offset.x - VisionData.Gravity_Offset.x;
    VisionData.Final_Offset.y = VisionData.Target_Offset.y - VisionData.Gravity_Offset.y;

    /*
f(x) = (p1*x^3 + p2*x^2 + p3*x + p4) / (x + q1)
Coefficients:
	   p1 =   0.0004088
	   p2 =     -0.2795
	   p3 =        74.2
	   p4 = - 1316
	   q1 =      -30.77
	*/

#elif Vision_Version == Vision_Newversion
//视觉接受数据的标志位是反的 区域赛后再改
    for (int i = 0; i < 14; i++)
    {
        VisionData.RawData.VisionRawData[i] = data[i];
    }

     CRCBuffer = Checksum_CRC8(data+1, sizeof(VisionData.RawData) - 18);
		
		if (VisionData.RawData.header == 0xA5 ||VisionData.RawData.checksum!=Get_CRC16_Check_Sum(VisionData.RawData.VisionRawData,sizeof(VisionData.RawData.VisionRawData)-2,0xFFFF)) //CRC校验失败。
    {
			VisionData.InfoUpdateFrame++;
			VisionData.OffLineFlag = 1;
			
			VisionData.RawData.yaw_angle = isnan(VisionData.RawData.yaw_angle)==true ? 0 : VisionData.RawData.yaw_angle;
			VisionData.RawData.pit_angle = isnan(VisionData.RawData.pit_angle)==true ? 0 : VisionData.RawData.pit_angle;
			
			VisionData.RawData.x =  VisionData.RawData.yaw_angle;
      VisionData.RawData.y =  VisionData.RawData.pit_angle;
			if(VisionData.RawData.x > 400 || VisionData.RawData.x < -400 || VisionData.RawData.y > 400 ||VisionData.RawData.y < -400)
			{
				 VisionData.RawData.x = VisionData.Error_Reme.x ;
				 VisionData.RawData.y = VisionData.Error_Reme.y ;
			}
    }
		else
		{
			VisionData.OffLineFlag = 0;
			VisionData.RawData.auto_aim = 0;
			VisionData.RawData.x = VisionData.RawData.y = 0 ;
		}

			//--- Nan也能过CRC校验
	if (isnan(VisionData.RawData.x)==true || isnan(VisionData.RawData.y)==true)
	{
		nan_jiajia++;
	}
	


    
    VisionData.InfoUpdateFlag = 1;

    //Get_FPS(&Vision_WorldTime, &VisionData.FPS);

//    if (VisionData.RawData.mode == 0 || VisionData.RawData.depth == 0 || VisionData.OffLineFlag == 1) //视觉模块失能或找不到目标
//    {
//        VisionExportData.FinalOffset_Last.x = 0;
//        VisionExportData.FinalOffset_Last.y = 0;
//        VisionExportData.FinalOffset.x = 0;
//        VisionExportData.FinalOffset.y = 0;
//        VisionData.Final_Offset.x = 0;
//        VisionData.Final_Offset.y = 0;
//     //  VisionData.RawData._pitch = 0;
//    //   VisionData.RawData.depth = 1000.0f;
//        return;
//    }

    //VisionData.Gravity_Offset.y =
    //	/*(pow(VisionData.RawData.depth, 3) *0.0004088 -
    //		0.2795 * pow(VisionData.RawData.depth, 2) +
    //		74.2*VisionData.RawData.depth - 1316) / (VisionData.RawData.depth - 30.77)+*/;

    //确定最终用于计算的偏差值
	if((VisionData.RawData.x < 400 && VisionData.RawData.x > -400) && (VisionData.RawData.y < 400 && VisionData.RawData.y > -400))
  {
		VisionData.Error_Reme.x = VisionData.RawData.x ;
    VisionData.Error_Reme.y = VisionData.RawData.y ;
	}
    /*
f(x) = (p1*x^3 + p2*x^2 + p3*x + p4) / (x + q1)
Coefficients:
	   p1 =   0.0004088
	   p2 =     -0.2795
	   p3 =        74.2
	   p4 = - 1316
	   q1 =      -30.77
	*/
#endif
}

/*************************************
* Method:    Vision_getOffset
* Returns:   int16_t
* Parameter: int16_t length
* Parameter: int16_t Value
* 说明：计算value 与 坐标系中点的偏差值。
************************************/
int16_t Vision_getOffset(int16_t length, int16_t Value)
{
    return Value - length / 2;
}

/*一个是读取SR寄存器，一个是读取对应的CR控制寄存器*/
/*正常情况下CR控制SR，我们入股要读取对应的标志位的话，既可以从CR读取也可以从SR读取*/
/*__HAL_UART_GET_FLAG是获取SR寄存器里面的情况，也就是读取被CR控制器控制之后的对应状态*/
/*__HAL_UART_GET_IT_SOURCE是直接读取控制寄存器里面的CRx标志位的情况*/
/*这里的DMA_GET_COUNTER是获取还没发出去的字符数量，和之前的不同*/
/*下面是两种情况的对比，请仔细阅读*/
/**
 * @Data    2019-03-23 20:07
 * @brief   视觉处理函数
 * @param   uint8_t *pData
 * @retval  void
 */
void Vision_Handler(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_DMA_DISABLE(huart->hdmarx);

    Vision_DataReceive(Vision_DataBuff);

    __HAL_DMA_SET_COUNTER(huart->hdmarx, Vision_BuffSIZE);
    __HAL_DMA_ENABLE(huart->hdmarx);
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
 * @brief 视觉使能DMA-USART
 * 
 * @param huartx 
 */
void Vision_USART_Receive_DMA(UART_HandleTypeDef *huartx)
{
    /*清空标志位然后使能USART的中断*/
    __HAL_UART_CLEAR_IDLEFLAG(huartx);
    __HAL_UART_ENABLE(huartx);
    __HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);
    // assert(Vision_BuffSIZE == (13 + 2));
    USART_Receive_DMA_NO_IT(huartx, Vision_DataBuff, Vision_BuffSIZE);
}

/**
 * @brief Get the Vision Disc Mode object
 * 
 * @return  
 */
uint8_t GetVisionDiscMode(void)
{
    return VisionData.RawData.enemy;
}


/**
 * @brief 视觉检测
 * 
 */
void Check_Vision(void)
{
    //视觉自瞄 ---------------------------
			if(VisionData.InfoUpdateFrame > VisionData.FPS )
		{
			VisionData.FPS = VisionData.InfoUpdateFrame;
		}
	
    if (VisionData.InfoUpdateFrame < 1)
    {
        VisionData.OffLineFlag = 0;
    }
    else
    {
        VisionData.OffLineFlag = 1;
    }
    VisionData.InfoUpdateFrame = 0;


}

int iii = 0;

int To_mode = 6;

void Vision_classdef_SendToPC(VisionSendMsg_u *pack2vision)
{
	//--- Test
	pack2vision->Pack.start_tag = 0x5A; 
	pack2vision->Pack.robot_color = 1; //机器人ID
	pack2vision->Pack.roll_angle =DJI_C_IMU.roll;   //发roll轴
	pack2vision->Pack.pit = DJI_C_IMU.pitch;//发pitch轴
	pack2vision->Pack.yaw_angle =  DJI_C_IMU.yaw;//发yaw轴
	pack2vision->Pack.checksum = Get_CRC16_Check_Sum(pack2vision->data,(uint32_t)14,0xFFFF); //crc
	
	CDC_Transmit_FS (pack2vision->data,sizeof(pack2vision->Pack));
	
	//HAL_UART_Transmit_DMA(&huart7, pack2vision->data, sizeof(pack2vision->Pack));

}
