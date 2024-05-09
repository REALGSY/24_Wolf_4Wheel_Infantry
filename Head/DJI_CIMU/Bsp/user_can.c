#include "user_can.h"

//CAN1_Filter 初始化
void CAN2_Filter0_Init(void)
{
	//CAN_Filter 初始化结构体
	CAN_FilterTypeDef CANx_Filter;
	
	CANx_Filter.SlaveStartFilterBank = 14;				//Filter 索引号14以下的 分配到CAN1
	CANx_Filter.FilterBank = 0;				//选择Filter0
	
	CANx_Filter.FilterMode = CAN_FILTERMODE_IDMASK;				//Filter 标识符掩码模式
	CANx_Filter.FilterScale = CAN_FILTERSCALE_32BIT;				//Filter 位宽选择：32位
	
	CANx_Filter.FilterIdHigh = 0x0000;				//标识符ID高16位
	CANx_Filter.FilterIdLow = 0x0000;					
	CANx_Filter.FilterMaskIdHigh = 0x0000;					//掩码高16位
	CANx_Filter.FilterMaskIdLow = 0x0000;
	
	CANx_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;				//关联到FIFO0
	
	CANx_Filter.FilterActivation = ENABLE;				//CAN使能
	
	//CAN_Filter 初始化函数
	if(HAL_CAN_ConfigFilter(&hcan2,&CANx_Filter) != HAL_OK)
	{
		while(1)
		{
		}
	}
}

//CAN发送数据帧函数
//参数1：选择要发送的CAN
//参数2：要发送数据帧的 ID类型
//参数3：要发送的数据帧 ID
//参数4：要发送的数据 数组
CAN_Tx_Typedef CAN_Tx_Structure;
void CAN_SendData(CAN_HandleTypeDef* hcanx,uint8_t id_type,uint32_t id,uint8_t data[8])
{
	//若不要检测，直接运行的话，作为局部变量就行了
//	CAN_Tx_Typedef CAN_Tx_Structure;				//CAN的发送结构体
	
	uint32_t Len;				//接收数据帧长度
	uint8_t i;				//循环控制参数
	uint32_t TxMailbox;				//返回CAN发送FIFO
	
	CAN_Tx_Structure.CAN_TxMessage.RTR = CAN_RTR_DATA;				//帧类型：数据帧
	CAN_Tx_Structure.CAN_TxMessage.IDE = id_type;				//ID类型：标准 或 拓展
	
	if(CAN_Tx_Structure.CAN_TxMessage.IDE == CAN_ID_STD)				//若为标准ID
	{
		CAN_Tx_Structure.CAN_TxMessage.StdId = id;								
	}
	else if(CAN_Tx_Structure.CAN_TxMessage.IDE == CAN_ID_EXT)				//若为拓展ID
	{
		CAN_Tx_Structure.CAN_TxMessage.ExtId = id;
	}
	
	CAN_Tx_Structure.CAN_TxMessage.DLC = 0x08;				//确定要发送的帧长度：max 8（字节）
	
	CAN_Tx_Structure.CAN_TxMessage.TransmitGlobalTime = DISABLE;				//非时间触发
	
	Len = CAN_Tx_Structure.CAN_TxMessage.DLC;
	for(i = 0 ; i < Len ; i++)
	{
		CAN_Tx_Structure.CAN_TxMessageData[i] = data[i];				//将要发送的数据传递給 存储CAN发送数据 数组
	}
	
	//向第一个空闲的Tx邮箱添加一条消息，并且激活相依的传输请求
	HAL_CAN_AddTxMessage(hcanx,&CAN_Tx_Structure.CAN_TxMessage,CAN_Tx_Structure.CAN_TxMessageData,&TxMailbox);
	
}

//CAN2 Filter0初始使能、CAN1使能
void CAN2_Config(void)
{
	//CAN2_Filter0 初始化 使能
	CAN2_Filter0_Init();
	
	//CAN2使能
	if(HAL_CAN_Start(&hcan2)!= HAL_OK)
	{
		while(1)
		{
		}
	}

}

