#include "Control_Vision.h"
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
#include "DEV_CIMU.h"
#include "M6020_Motor.h"
#include "CRC.h"
#include <stdio.h>
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
    if (VisionData.RawData.whether_Fire)
    {
        VisionExportData.AbleToFire = true;
    }

    if (VisionData.RawData.mode == 0 || VisionData.RawData.depth == 0  || VisionData.OffLineFlag == 1) //视觉模块失能或找不到目标
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
void Vision_DataReceive(uint8_t *data)
{
		//--- 不校验帧头帧尾以及crc位
    uint8_t CRCBuffer = Checksum_CRC8(data+1, sizeof(VisionData.RawData) - 3);
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
        /*(pow(VisionData.RawData.depth, 3) *0.0004088 -
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

    for (int i = 0; i < 15; i++)
    {
        VisionData.RawData.VisionRawData[i] = data[i];
    }

 


    if (CRCBuffer == VisionData.RawData.crc||VisionData.RawData.Start_Tag == 'S' || VisionData.RawData.End_Tag == 'E') //CRC校验失败。
    {
			VisionData.OffLineFlag = 1;
			VisionData.InfoUpdateFrame++;
			VisionData.RawData.x = (VisionData.RawData.yaw_angle) * 22.7527f ;
      VisionData.RawData.y = (VisionData.RawData.pitch_angle) * 22.7527f ;
			if(VisionData.RawData.x > 360 ||VisionData.RawData.x < -360 || VisionData.RawData.y > 360 ||VisionData.RawData.y < -360)
			{
				 VisionData.RawData.x = VisionData.Error_Reme.x ;
				 VisionData.RawData.y = VisionData.Error_Reme.y ;
			}
      VisionData.RawData.depth = VisionData.RawData.pitch_angle;
    }
		else
		{
			VisionData.OffLineFlag = 0;
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
	if(VisionData.RawData.x > 360 ||VisionData.RawData.x < -360 || VisionData.RawData.y > 360 ||VisionData.RawData.y < -360)
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
    return VisionData.RawData.mode;
}


/**
 * @brief 视觉检测
 * 
 */
void Check_Vision(void)
{
    //视觉自瞄 ---------------------------
    if (VisionData.InfoUpdateFrame < 1)
    {
        VisionData.OffLineFlag = 1;
    }
    else
    {
        VisionData.OffLineFlag = 0;
    }
    VisionData.InfoUpdateFrame = 0;
}

int iii = 0;
void Vision_classdef_SendToPC(VisionSendMsg_u *pack2vision)
{
	//--- Test
	// static uint16_t crc_calc = 0;
	// crc_calc = Checksum_CRC8(pack2vision->data+1,CRC_CHECK_LEN);

	pack2vision->Pack.start_tag = 0x53; // 'S'
	// pack2vision->Pack.crc = 888;
	pack2vision->Pack.robot_id = 0; 
	pack2vision->Pack.mode = 6;
//	if(pack2vision->Pack.mode > 6 || pack2vision->Pack.mode < 0)
//	{
//		pack2vision->Pack.mode = 0;
//	}
	
//	pack2vision->Pack.yaw_angle = Cimu_Export.DJI_C_Euler_Receive.yaw;
//	pack2vision->Pack.pit_angle = Cimu_Export.DJI_C_Gyro_Receive.pitch;
	pack2vision->Pack.yaw_angle = DJI_C_IMU.yaw;
	pack2vision->Pack.pit_angle = DJI_C_IMU.pitch;
	// pack2vision->Pack.yaw_angle = IMU_YawLPF.f(Gimbal.Get_IMUAngle(Yaw));
	// pack2vision->Pack.pit_angle = IMU_PitLPF.f(Gimbal.Get_IMUAngle(Rol));
	pack2vision->Pack.bullet_velocity = 0;
	if(pack2vision->Pack.bullet_velocity < 15)
	{
		pack2vision->Pack.bullet_velocity = 15;
	}
	else if(pack2vision->Pack.bullet_velocity > 30)
	{
		pack2vision->Pack.bullet_velocity = 30;	
	}
	
	pack2vision->Pack.end_tag = 0x45; // 'E' 
	HAL_UART_Transmit(&huart7 , pack2vision->data, sizeof(pack2vision->Pack), 0xFF);
	
	//HAL_UART_Transmit_DMA(&huart7, pack2vision->data, sizeof(pack2vision->Pack));

}
