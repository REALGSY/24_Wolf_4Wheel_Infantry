/**
  * @brief  底盘超级电容控制函数
  * @param	None
  * @retval None
  */
	
#include "RM_JudgeSystem.h "
#include "DEV_SupCap.h"
#include "DEV_Power_Meter.h" 
#include "Monitor_RM_CAN.h"
/**
  ******************************************************************************
  * @file     Supercapacitor.c
  * @author   Shake
  * @version  V2.0
  * @brief    超级电容模块
  ******************************************************************************
  */

#include "Dev_SupCap.h"
#include "DR16_Remote.h"
//#include "System_DataPool.h"
//#include "Power_Meter.h"
uint8_t SupCap_C_Flag = 0;
SupCap_t SupCap_Check;
SupCap_classdef Supcap_Rec;
void SupCap_Update(Can_Export_Data_t RxMessage)
{

		if(RxMessage.CAN_RxHeader.StdId != SCCM_RECEIVE_ID)
	{
		return;
	}
  memcpy(Supcap_Rec.RecvData.data, RxMessage.CANx_Export_RxMessage, 8);
	SupCap_Check.infoUpdateFrame ++;
	SupCap_Control();
}

void SupCap_Control(void)
{
    SupCap_Process();

    SupCap_Send_Msg(&hcan1);
}

uint8_t charge_flag; //--- 超电用完后充电标志位

void SupCap_Process(void)
{
    //--- 底盘失能或电容离线
    if (Robot.ChassisWorkMode ==  ChassisWorkMode_Disable || SupCap_Check.OffLineFlag == 1)
    {
        Supcap_Rec.State = SupCap_OFF;
        SupCap_Main_Switch(SupCap_OFF, Charging_OFF, Power_NotSupply);
        return;
    }
    else
    {
        Supcap_Rec.State = SupCap_ON;
        Supcap_Rec.SendData.Pack.charge_enable = SupCap_ON;
    }

    //

    if (ext_game_robot_state.data.chassis_power_limit < 45)
    {
        Supcap_Rec.ChassisPower_Limit = 45;
    }
    else if (ext_game_robot_state.data.chassis_power_limit > 120)
    {
        Supcap_Rec.ChassisPower_Limit = 120;
    }
    else
    {
        Supcap_Rec.ChassisPower_Limit = ext_game_robot_state.data.chassis_power_limit;
    }

    //---充能功率——剩余功率
    if (SupCap_Is_Output() == true && SupCap_Get_Cell() > 40)
    {
        //--- 边放边充，放电的时候满功率充电
        Supcap_Rec.Charging_Power = Supcap_Rec.ChassisPower_Limit;
    }
    else
    {
        if (Power_Meter.OffLineFlag == 1)
        {
            //--- 功率计掉线则使用裁判系统的数据
            Supcap_Rec.Charging_Power = Supcap_Rec.ChassisPower_Limit - ext_game_robot_state.data.chassis_power_limit;
        }
        else
        {
            Supcap_Rec.Charging_Power = Supcap_Rec.ChassisPower_Limit - Power_Meter.Power;
        }
    }

    //充能功率限幅
    if (Supcap_Rec.Charging_Power > Supcap_Rec.ChassisPower_Limit + 5) // 底盘输出功率
    {
        Supcap_Rec.Charging_Power = Supcap_Rec.ChassisPower_Limit;
    }
    else if (Supcap_Rec.Charging_Power < 0)
    {  
        Supcap_Rec.Charging_Power = 0.0f;
    }

    //缓存功率限制充能功率
    if (ext_power_heat_data.data.chassis_power_buffer < 60 && ext_power_heat_data.data.chassis_power_buffer > 55)
    {
        Supcap_Rec.Charging_Power -= 5;
    }
    else if (ext_power_heat_data.data.chassis_power_buffer <= 55)
    {
        Supcap_Rec.Charging_Power = 0.1f;
        // SendData.is_cap_output = false;
    }
    SupCap_ChargeControl(Supcap_Rec.Charging_Power);

    //--- 用完后等充到50再允许再次使用
    if (charge_flag == true && SupCap_Get_Cell() > 50)
    {
        charge_flag = false;
    }

//    if (Infantry.Write_Msg[Cap_Ctrl] == true && SupCap_Get_Cell(supcap) > 40 && SupCap_Is_CanbeUse(supcap) != false)
//    {
//        if (charge_flag == false)
//        {
//            SupCap_SupplySwitch(supcap, Power_Supply); //--- 放电
//        }
//    }
//    else
//    {
//        if (SupCap_Get_Cell(supcap) <= 40)
//        {
//            charge_flag = true;
//        }
//        SupCap_SupplySwitch(supcap, Power_NotSupply);
//    }
		if (Robot.ChassisWorkMode ==  ChassisWorkMode_CloudFollow || Robot.ChassisWorkMode ==  ChassisWorkMode_Square)
    {
			SupCap_SupplySwitch(Power_Supply);
//			if(rc.roll != 0)
//			{
//				SupCap_SupplySwitch(Power_NotSupply);
//			}
    }
        
		if(Supcap_Rec.RecvData.Pack.cap_cell <= 40)
		{
			SupCap_SupplySwitch(Power_NotSupply);
		}
		
		if(SupCap_C_Flag == 0)
		{
			SupCap_SupplySwitch(Power_NotSupply);
		}
    
}

void SupCap_Send_Msg(CAN_HandleTypeDef *hcan)
{
    uint8_t Data[8];
    memcpy(Data, Supcap_Rec.SendData.data, 8);
    Robomaster_SupCap_Can1_SendData(Data);
}

void SupCap_Main_Switch(bool Ctrl, bool Charge, bool Supply)
{
    Supcap_Rec.State = Ctrl;
    Supcap_Rec.SendData.Pack.charge_enable = Charge;
    Supcap_Rec.SendData.Pack.is_cap_output = Supply;
}

void SupCap_ChargeControl( float Charging_Power)
{
    Supcap_Rec.SendData.Pack.charge_power = Charging_Power;
}

void SupCap_ChargeSwitch(bool Switch)
{
    Supcap_Rec.SendData.Pack.charge_enable = Switch;
}

void SupCap_SupplySwitch(bool Switch)
{
    Supcap_Rec.SendData.Pack.is_cap_output = Switch;
}

uint8_t SupCap_Get_Cell(void)
{
    return Supcap_Rec.RecvData.Pack.cap_cell;
}

bool SupCap_Is_CanbeUse(void)
{
    return Supcap_Rec.RecvData.Pack.Is_enable;
}

bool SupCap_Is_Output(void)
{
    return Supcap_Rec.SendData.Pack.is_cap_output;
}

void Check_SupCap(void)
{
	if(SupCap_Check.infoUpdateFrame < 1)
	{
		SupCap_Check.OffLineFlag = 1;
	}
	else
	{
		SupCap_Check.OffLineFlag = 0;
	}
	SupCap_Check.infoUpdateFrame = 0;
}
