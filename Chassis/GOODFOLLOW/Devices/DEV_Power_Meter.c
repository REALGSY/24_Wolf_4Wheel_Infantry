#include "DEV_Power_Meter.h" 
#include "typedef.h"
#include "DEV_CIMU.h"
PowerMeter_Rec_t PowerMeter_Rec;
PowerMeter_t Power_Meter;
void PowerMeter_Update(Can_Export_Data_t RxMessage)
{
  memcpy(&PowerMeter_Rec.data, RxMessage.CANx_Export_RxMessage, 8);
	Power_Meter.Power = PowerMeter_Rec.Pack.Shunt_Current * PowerMeter_Rec.Pack.voltageVal /1000/1000;
	Power_Meter.infoUpdateFrame++;
}

void Check_Power_Meter(void)
{
	if(Power_Meter.infoUpdateFrame < 1)
	{
		Power_Meter.OffLineFlag = 1; 
	}
	else
	{
		Power_Meter.OffLineFlag = 0;
	}
	Power_Meter.infoUpdateFrame = 0;
}
