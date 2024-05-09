#include "Power_control.h"
#include "arm_math.h"
#include "RM_JudgeSystem.h"
#include <math.h>
#include "DR16_Remote.h"
int16_t Power_Buffer;   /*<! ���幦�� */
int32_t SumCurrent_In;  /*<! ���������ܺ� */
int32_t SumCurrent_Out; /*<! �����ĵ�������ܺ� */

int32_t RUD_Current_In;  /*<! ���������ܺ� */
int32_t RUD_Current_Out; /*<! �����ĵ�������ܺ� */
int32_t DRV_Current_In;
int32_t DRV_Current_Out;

float DRV_CalcRatio;       /*<! ���ڼ������ƹ��ʵ�ϵ�� */
float RUD_CalcRatio;       /*<! ���ڼ������ƹ��ʵ�ϵ�� */

void Limit(int16_t *wheelCurrent, int8_t amount)
{
    float coe[4] = {0.0f};

    //--- ����ϵͳ���� ǿ������
//    if(DevicesMonitor.Get_State(REFEREE_MONITOR) == Off_line)
//    {

//    }

    //--- ���޹���
    if(ext_game_robot_state.data.chassis_power_limit == 65535)
    {
        SumCurrent_Out = SumCurrent_In;  //--- �޴���ʱΪԭ����ֵ
        return;
    }

    SumCurrent_In = SumCurrent_Out = 0;


    /*-----------------------------------------*/
    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        SumCurrent_In += abs(wheelCurrent[i]);
    }
		
    SumCurrent_Out = SumCurrent_In;  // �޴���ʱΪԭ����ֵ

    // ����ÿ������ĵ���ռ��
//		if(rc.ch3 != 660)
//		{
			for(uint8_t i = 0 ; i < 4 ; i++)
			{
        coe[i] = ((float)(wheelCurrent[i])) / ((float)(SumCurrent_In));
			}
//		}
//		else
//		{
//			coe[0] = ((float)(wheelCurrent[0])) / ((float)(SumCurrent_In)) * 0.3f;
//			coe[1] = ((float)(wheelCurrent[1])) / ((float)(SumCurrent_In)) * 0.3f;
//			coe[2] = ((float)(wheelCurrent[2])) / ((float)(SumCurrent_In)) * 1.7f;
//			coe[3] = ((float)(wheelCurrent[3])) / ((float)(SumCurrent_In)) * 1.7f;
//		}
    Limit_Calc();

    for(uint8_t i = 0 ; i < amount ; i++)
    {
        wheelCurrent[i] = ((SumCurrent_Out) * coe[i]);
    }
}

int16_t powerBuffErr;  // �õ��Ļ�������
float debug_powercoe = 80.0f;
void Limit_Calc (void)
{
    Power_Buffer = ext_power_heat_data.data.chassis_power_buffer;

    powerBuffErr = 60 - Power_Buffer;

    DRV_CalcRatio = 0;
    DRV_CalcRatio = (float)Power_Buffer / debug_powercoe;
    DRV_CalcRatio *= DRV_CalcRatio;  // ƽ���Ĺ�ϵ

    if(powerBuffErr > 0 /* && Infantry.Write_Msg[Cap_Ctrl] != true */)  // ���õ����幦������й������ƴ���
    {
        SumCurrent_Out = SumCurrent_In * DRV_CalcRatio;
    }
}
