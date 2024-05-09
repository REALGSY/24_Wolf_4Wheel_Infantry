#include "Task_CanMsg.h"
#include "BSP_CAN.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "M2006_Motor.h"
#include "DEV_CIMU.h"
#include "typedef.h"
#include "DR16_Remote.h"
#include "DEV_Power_Meter.h" 
#include "DEV_SupCap.h"
void CAN1_REC(void const * argument)
{
	Can_Export_Data_t Can_Export_Data;
	uint32_t ID;
	for( ; ;)
	{
	 xQueueReceive(CAN1_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
	 ID = Can_Export_Data.CAN_RxHeader.StdId; 
	 if(ID == 0x175)
	 {
		DR16_CAN2_REC(Can_Export_Data);
	 }
	 else if(ID == 0x185)
	 {
		DR16_ROLL_REC(Can_Export_Data);
	 }
	 	else if(ID == 0x301)
	 {
		PowerMeter_Update(Can_Export_Data);
	 }
	 else if (ID == 0x600)
	 {
		SupCap_Update(Can_Export_Data);
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
	 if(ID >= 0x201 && ID <= 0x204)
   {
		M3508_getInfo(Can_Export_Data);
	 }
	 else if(ID >= 0x205 && ID <= 0x208)
   {
		M6020_getInfo(Can_Export_Data);
	 }

  }
}
