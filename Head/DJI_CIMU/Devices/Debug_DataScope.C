/**
  ******************************************************************************
  * @file    Debug_DataScope.c
  * @author  IMTao
  * @version V2.0
  * @brief   MiniBalance上位机模块
  ******************************************************************************
  */
#include "Debug_DataScope.h"
unsigned char DataScope_OutPut_Buffer[42] = { 0 };	   //串口发送缓冲区
/*
用法：
		1. Debug_addData(500*sin(a), 1 ); 将要显示的各通道数据，依次添加，其中1表示第一条数据,依次类推。
			Debug_addData(500* tan(a), 2 );
			Debug_addData( 500*cos(a), 3 );

		2. Debug_show(3);  完成后，调用show函数将数据发送到上位机中，3表示一共发送3个通道的数据。

*/

//函数说明：将单精度浮点数据转成4字节数据并存入指定地址 
//附加说明：用户无需直接操作此函数 
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
//函数无返回 
void Float2Byte(float *target, unsigned char *buf, unsigned char beg)
{
	unsigned char *point;
	point = (unsigned char*)target;	  //得到float的地址
	buf[beg] = point[0];
	buf[beg + 1] = point[1];
	buf[beg + 2] = point[2];
	buf[beg + 3] = point[3];
}



//原函数名：DataScope_Get_Channel_Data
//函数说明：将待发送通道的单精度浮点数据写入发送缓冲区
//Data：通道数据
//Channel：选择通道（1-10）
//函数无返回 
void Debug_addData(float Data, unsigned char Channel)
{
	if ((Channel > 10) || (Channel == 0)) return;  //通道个数大于10或等于0，直接跳出，不执行函数
	else
	{
		switch (Channel)
		{
		case 1:  Float2Byte(&Data, DataScope_OutPut_Buffer, 1); break;
		case 2:  Float2Byte(&Data, DataScope_OutPut_Buffer, 5); break;
		case 3:  Float2Byte(&Data, DataScope_OutPut_Buffer, 9); break;
		case 4:  Float2Byte(&Data, DataScope_OutPut_Buffer, 13); break;
		case 5:  Float2Byte(&Data, DataScope_OutPut_Buffer, 17); break;
		case 6:  Float2Byte(&Data, DataScope_OutPut_Buffer, 21); break;
		case 7:  Float2Byte(&Data, DataScope_OutPut_Buffer, 25); break;
		case 8:  Float2Byte(&Data, DataScope_OutPut_Buffer, 29); break;
		case 9:  Float2Byte(&Data, DataScope_OutPut_Buffer, 33); break;
		case 10: Float2Byte(&Data, DataScope_OutPut_Buffer, 37); break;
		}
	}
}


//函数说明：生成 DataScopeV1.0 能正确识别的帧格式
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
//返回0表示帧格式生成失败 
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ((Channel_Number > 10) || (Channel_Number == 0)) { return 0; }  //通道个数大于10或等于0，直接跳出，不执行函数
	else
	{
		DataScope_OutPut_Buffer[0] = '$';  //帧头

		switch (Channel_Number)
		{
		case 1:   DataScope_OutPut_Buffer[5] = 5; return  6;
		case 2:   DataScope_OutPut_Buffer[9] = 9; return 10;
		case 3:   DataScope_OutPut_Buffer[13] = 13; return 14;
		case 4:   DataScope_OutPut_Buffer[17] = 17; return 18;
		case 5:   DataScope_OutPut_Buffer[21] = 21; return 22;
		case 6:   DataScope_OutPut_Buffer[25] = 25; return 26;
		case 7:   DataScope_OutPut_Buffer[29] = 29; return 30;
		case 8:   DataScope_OutPut_Buffer[33] = 33; return 34;
		case 9:   DataScope_OutPut_Buffer[37] = 37; return 38;
		case 10:  DataScope_OutPut_Buffer[41] = 41; return 42;
		}
	}
	return 0;
}

//函数说明：将已经添加到发送缓冲区的数据发出。
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
//返回0表示帧格式生成失败 
void Debug_show(int ChannelAmount) {
	int Send_Count = DataScope_Data_Generate(ChannelAmount);

	/*循环Send_Count次将DataScope_OutPut_Buffer[i]中的内容发送出去。
		即完成了一次当前帧的数据上传。*/
	for (int i = 0; i < Send_Count; i++)
	{
		while ((USART1->SR & 0X40) == 0);
		USART1->DR = DataScope_OutPut_Buffer[i];
		//USART_sendChar(USART6, DataScope_OutPut_Buffer[i]);
	}
}

/* 下面是一个可以debug 时切换输出的调试信息的功能，看的懂的可以修改后使用。
typedef enum {
	DataType_TempData,
	DataType_CloudData,
	DataType_ChassisData,
	DataType_VisionData,
	DataType_ShootData

}DataType_e;
DataType_e Debug_DataType = DataType_ShootData;//可以在debug的时候直接修改该值实现切换数据 
static uint8_t DelayTime = 10;
void Task_DebugShow(void) {
	while (true)
	{
		vTaskDelay(DelayTime);//过低时会造成发送丢包失败。
		if (Debug_DataType == DataType_TempData)
		{

		}
		else if (Debug_DataType == DataType_CloudData)
		{
			Debug_addData(M6020s_Pitch.pid_Angle.err, 1);
			Debug_addData(M6020s_Pitch.outCurrent, 2);
			Debug_addData(M6020s_Pitch.realSpeed, 3);

			Debug_addData(Cloud.YawAttitude_pid.err, 4);
			Debug_addData(M6020s_Yaw.outCurrent, 5);
			Debug_addData(IMU_Cloud.Gyro.z, 6);
			Debug_show(6);
		}
		else if (Debug_DataType == DataType_ChassisData)
		{
			Debug_addData(M3508s[0].pid_angle.err, 1);
			Debug_addData(IMU_Chassis.Gyro.z, 2);
			Debug_show(2);
		}
		else if (Debug_DataType == DataType_VisionData)
		{
			Debug_addData(VisionData.Final_Offset.x, 1);
			Debug_addData(VisionData.Final_Offset.y, 2);
			Debug_addData(VisionData.ErrorChange_Rate.x, 3);
			Debug_addData(VisionData.ErrorChange_Rate.y, 4);
			Debug_addData(VisionExportData.FinalOffset.x, 5);
			Debug_addData(VisionExportData.FinalOffset.y, 6);
			//Debug_addData(VisionData.Vision_SpeedPid_Yaw.pwm, 7);
			Debug_addData(VisionData.RawData.x, 7);
			Debug_show(7);
		}
		else if (Debug_DataType == DataType_ShootData)
		{
			Debug_addData(shootUnit1.Reloader_Motor->realSpeed, 1);
			Debug_addData(shootUnit1.Reloader_Motor->realTorque, 2);
			Debug_addData(shootUnit1.Reloader.PillObstruct_Time,3);
			Debug_show(3);
		}


	}


}
*/
