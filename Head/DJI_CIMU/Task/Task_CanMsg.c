#include "Task_CanMsg.h"
#include "BSP_CAN.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "M2006_Motor.h"
#include "typedef.h"
#include "RM_Message.h"
int xjj;
int ymy;
PowerMeter_t Power_Meter;
int ainimemeda;
void PowerMeter_Update(Can_Export_Data_t RxMessage)
{
    memcpy(&Power_Meter.data, RxMessage.CANx_Export_RxMessage, 8);	
		ainimemeda = Power_Meter.Pack.Shunt_Current * Power_Meter.Pack.voltageVal /1000/1000;
}

void CAN1_REC(void const * argument)
{
	Can_Export_Data_t Can_Export_Data;
	uint32_t ID;
	for( ; ;)
	{
	 xQueueReceive(CAN1_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
	 ID = Can_Export_Data.CAN_RxHeader.StdId; 
		xjj = ID;
		

	 if(ID >= 0x205 && ID <= 0x20A)
	 {
		M6020_getInfo(Can_Export_Data);
	 }
	 else if(ID == 0x088)
	 {
		RM_Mess_getInfo(Can_Export_Data);
	 }
	 else if(ID == 0x301)
	 {
		PowerMeter_Update(Can_Export_Data);
	 }	

  }
	
}


void CAN2_REC(void const * argument)
{
  uint32_t ID;
  Can_Export_Data_t Can_Export_Data;
	for( ; ;)
	{
   xQueueReceive(CAN2_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
   ID = Can_Export_Data.CAN_RxHeader.StdId;
	 if(ID >= 0x201 && ID <= 0x203)
	 {
		M2006_getInfo(Can_Export_Data);
	 }
  }
}
