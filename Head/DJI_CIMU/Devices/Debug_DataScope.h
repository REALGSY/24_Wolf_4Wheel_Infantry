

#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H

#include "usart.h"
 
extern unsigned char DataScope_OutPut_Buffer[42];	   //待发送帧数据缓存区



void Debug_show(int ChannelAmount);
void Debug_addData(float Data,unsigned char Channel);    // 写通道数据至 待发送帧数据缓存区
void Task_DebugShow(void);
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // 发送帧数据生成函数 
 
 
#endif 



