#include "bsp_buzzer.h"

Buzzer_t Buzzer;

//初始化buzzer
void Buzzer_Init(void)
{
  //使能控制buzzer的PWM
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  //初始时为无声状态
  Buzzer.mode = Zero_times;
  //占空比
  Buzzer.buzzer_value = 0;
	Buzzer.status = OFF;
  Buzzer.continue_times = 0;
  Buzzer.times = 0;
}

void Buzzer_Processing(void)
{
  switch (Buzzer.mode)
  {
  case Zero_times:
    //占空比
    Buzzer.buzzer_value = 0;
	  Buzzer.continue_times = 0;
    Buzzer.status = OFF;
		Buzzer.times = 0;
    break;
  case One_times:
    //响一声持续1s
    Buzzer.continue_times++;
	  if (Buzzer.times == 1)
    {
      Buzzer.mode = Zero_times;
    }
	//这是一周周期性思想， T = 2 *  Buzzer_Continue_Times
    if ((Buzzer.continue_times / Buzzer_Continue_Times) % 2 == 0 && Buzzer.status == OFF)
    {
      Buzzer.buzzer_value = Buzzer_Val;
			Buzzer.status = ON;
    }
		if((Buzzer.continue_times / Buzzer_Continue_Times) % 2 == 1 && Buzzer.status == ON)
		{
			Buzzer.buzzer_value = 0;
			Buzzer.status = OFF;
			Buzzer.times += 1;
		}
    break;
	case heaps_times:
		Buzzer.buzzer_value = Low_Buzzer_Val;
	  Buzzer.continue_times = 0;
    Buzzer.status = OFF;			//保证他是从OFF开始 
		Buzzer.times = 0;
		break;
  default:
    Buzzer.continue_times++;
	  if (Buzzer.times == 3)
    {
      Buzzer.mode = Zero_times;
    }
    if ((Buzzer.continue_times / Buzzer_Continue_Times) % 2 == 0 && Buzzer.status == OFF)
    {
      Buzzer.buzzer_value = Buzzer_Val;
			Buzzer.status = ON;
    }
		if((Buzzer.continue_times / Buzzer_Continue_Times) % 2 == 1 && Buzzer.status == ON)
		{
			Buzzer.buzzer_value = 0;
			Buzzer.status = OFF;
			Buzzer.times += 1;
		}
    break;
  }
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, Buzzer.buzzer_value);
}
