#include "DevicesCheck.h"
#include "DR16_Remote.h"
#include "M2006_Motor.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"

void Offline_Check (void);
Dev_Check_t  Dev_Check = Dev_Check_Init;


	void Offline_Check (void)
{
	rc.sw1 = 2;
	rc.sw2 = 2;
}
