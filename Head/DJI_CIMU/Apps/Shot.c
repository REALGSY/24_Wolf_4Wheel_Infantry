#include "shot.h"
#include "M2006_Motor.h"
#include "Monitor_RM_CAN.h"
#include "PID.h"
#include "DR16_Remote.h"
#include "tim.h"
#include "Robot_control.h"
#include "PID.h"
#include "RM_Message.h"
#include "Chassis_control.h"
#include "Control_Vision.h"
#include "WS2812.h"
M2006s_t M2006s[3];
M2006s_t roll_inc;
M2006s_t roll_pos;
Shot_t shot;
uint8_t Fric_L = 0;
uint8_t Fric_R = 1;
float Shot_RealSpeed = 0;
//摩擦轮转速
int16_t FricSpeed_out = 200;
int16_t FricSpeed_15 = 4400;
int16_t FricSpeed_18 = 4900;
int16_t FricSpeed_22 = 5100;
int16_t FricSpeed_30 = 6950;
int16_t Start_Speed = 4400;
int16_t FricSpeed = 0;
int16_t hold_time = 45;
uint16_t Auto_Shot = 0;
uint16_t FIRE_FIRE = 0;
uint16_t Shoot_Down = 100; 
uint8_t Shoot_Down_Flag = 0;
//摩擦轮和拨盘pid赋值

cloud_incrementalpid_t Fric_L_IPID    = Fric_L_IPIDInit;
cloud_incrementalpid_t Fric_R_IPID    = Fric_R_IPIDInit;
cloud_positionpid_t    Fric_MotorOPID  = Fric_MotorOPIDInit;
cloud_positionpid_t Fric_MotorIPID  = Fric_MotorIPIDInit;

void Auto_Shoot(void);

/**
 * @brief 发射机构总控制
 * 
 * @param target 
 * @param value 
 * @return int 6
 */
int Shot_G = 0;
void roll_shot_go (void) //摩擦轮函数
{
	Speed_Reme();
	if(Robot.Device_FrictMode == FrictWorkMode_HighSpeed)
	{
		shot.Disability = 1;		
		Roll_Init();//摩擦轮初始化
		Roll_Preload();//预装载
		Auto_Shoot();
		Roll_Mode(); //roll模式	
		Shoot_MotorInObstruct();
		State_detection();
		shot_IRQHandler();
		Motor_0x200_Can2_SendData(M2006s[0].outCurrent,M2006s[1].outCurrent,roll_inc.outCurrent,0);
		if(roll_pos.totalAngle > roll_pos.targetAngle - 10000)
		{
			shot.maybe_stop = 0;
		}
	}
	else
	{
		Reload_Reset();
	}
}


/**
 * @brief 摩擦轮初始化
 * 
 * @param target 
 * @param value 
 * @return int 
 */
 
void Roll_Init (void)  //摩擦轮初始化 
{
  static int time = 0 ;
	if((M2006_Reload_1.OffLineFlag == 1 && M2006_Reload_2.OffLineFlag == 1 && M2006_Reload_3.OffLineFlag == 1 ))
	{
		time = 0; //应急时刻检查发射有没有寄
	}
	if(time == 0)
  {
	 roll_pos.targetAngle = M2006_Reload_3.realAngle;
   M2006_Reload_3.totalAngle = 0;
	 M2006_Reload_3.turnCount = 0;
	 M2006s[Fric_L].targetSpeed = 0;
   M2006s[Fric_R].targetSpeed = 0;
  }
  time++;
}

/**
 * @brief 发射控制
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void shot_IRQHandler(void)
{	
	FricSpeed_Change();  //速度获取
	M2006s[Fric_L].targetSpeed = - FricSpeed; //摩擦轮速度
	M2006s[Fric_R].targetSpeed =   FricSpeed; 
	M2006s[Fric_L].outCurrent = Fric_L_IPID.CLOUD_Incremental_PID(&Fric_L_IPID, M2006s[0].targetSpeed, M2006s[0].realSpeed); //摩擦轮计算pid
	M2006s[Fric_R].outCurrent = Fric_R_IPID.CLOUD_Incremental_PID(&Fric_R_IPID, M2006s[1].targetSpeed, M2006s[1].realSpeed);
	roll_pos.outCurrent = Fric_MotorOPID.CLOUD_Position_PID(&Fric_MotorOPID,roll_pos.targetAngle,roll_pos.totalAngle);  //拨轮外环（位置环）
	roll_inc.outCurrent = Fric_MotorIPID.CLOUD_Position_PID(&Fric_MotorIPID, roll_pos.outCurrent, M2006_Reload_3.realSpeed); //拨轮内环 （速度环）
}





/**
 * @brief 准备发射
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void if_or_no (void)
{
	shot.roll_frequency ++ ;//记录发射次数
	roll_pos.targetAngle += one_part_roll ;
	shot.shot_one = 1;
	shot.roll_get_time = 0 ;
	if(shot.Key_need > 0)
	shot.Key_need --;
}

/**
 * @brief 摩擦轮预装载
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void Roll_Preload (void)  
{
	if(DR16_Export_Data.Robot_TargetValue.Dial_Wheel  > 550 && DR16_Export_Data.Robot_TargetValue.Dial_Wheel  < 620 )
	{
		shot.roll_get_time = 38;
	}
}

/**
 * @brief 准备拨弹
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void Roll_Mode (void)   
{
	//限制射频
	if(RM_ME[0].Residual_Heat == -1)
	{
		hold_time = 25;
	}
	else
	{
		if(RM_ME[0].Residual_Heat < 11)
		{
			if(FIRE_FIRE == 1 && Robot.ControlSource == ControlSource_PC)
			{
			}	
		  else
			{
				shot.Key_need = 0;
				shot.roll_get_time = 0;
			}
		}
		else if(RM_ME[0].Residual_Heat< 35 && RM_ME[0].Residual_Heat >= 11)
		{
			hold_time = 120;
		}
		else if(RM_ME[0].Residual_Heat< 45 && RM_ME[0].Residual_Heat >= 35)
		{
			hold_time = 65;
		} 
		else
		{
			hold_time = 25;
		}
  }
	if(FIRE_FIRE == 1 && Robot.ControlSource == ControlSource_PC)
	{
		hold_time = 5;
	}
	
//	if(M2006_Reload_3.OffLineFlag == 1)
//	{
//		shot.Key_need = 0;
//	}	
	//使用遥控器的时候
	if(shot.maybe_stop == 0)
	{
		if(Robot.ControlSource != ControlSource_PC)
		{
	 		if(shot.Key_need != 0)
			{
				shot.roll_get_time ++;
				if(shot.roll_get_time > hold_time)
				{
					shot.roll_get_time = hold_time;
				}
			}
			if(Robot.ControlSource != ControlSource_PC && onlineyaw != 1)
			{
				if(DR16.rc.ch4_DW  >= 620 && M2006_Reload_3.OffLineFlag != 1)
				{
					shot.roll_get_time ++;
					if(shot.roll_get_time > hold_time)
					{
						shot.roll_get_time = hold_time;
					}
				}
			}
			if (shot.roll_get_time == hold_time)
			{
				if_or_no();
			}
		}
		else
		{
			if(shot.Key_need != 0)
			{
				shot.roll_get_time ++;
				if(shot.roll_get_time > hold_time)
				{
					shot.roll_get_time = hold_time;
				}
			}
	
			if (shot.roll_get_time == hold_time)
			{
				if_or_no();
			}
		}
	}
}

/**
 * @brief 保存发射数量
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void Shoot_MotorInObstruct (void)
{
	  shot.last_roll_frequency = shot.roll_frequency;  //将现在的发射数目存为上次的发射数
}

/**
 * @brief 发射状态检查
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void State_detection (void)      
{
	if(shot.shot_one == 1)
	{
		if(roll_pos.totalAngle < roll_pos.targetAngle - 10000 && M2006_Reload_3.realSpeed < 1300 && shot.shot_one == 1 ) 
		{
			shot.time_up ++ ; //计算卡弹时间
		}
		 
		shot.Locked_time ++ ; //时间
		
		if( shot.Locked_time >= 85 )
		{
			shot.Locked_time = 0;
			shot.time_up = 0;
			shot.shot_one = 0 ;
		}
		
		if( shot.time_up >= 84 )
	  {
			shot.maybe_stop = 1 ; //超过卡弹时间，判断为卡弹 
	  }
		
		if (shot.maybe_stop == 1)
		{
			roll_pos.targetAngle -= 4.0f * one_part_roll;
			shot.time_up = 0;
		}
	}
}


/**
 * @brief 摩擦轮失能先伺服到0，而后持续输出0
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void Reload_Reset(void)
{
	if (shot.Disability == 1)
 {
			M2006s[Fric_L].targetSpeed = 0; //摩擦轮速度
	    M2006s[Fric_R].targetSpeed = 0; //摩擦轮速度
			M2006s[Fric_L].outCurrent = Fric_L_IPID.CLOUD_Incremental_PID(&Fric_L_IPID, M2006s[Fric_L].targetSpeed, M2006s[Fric_L].realSpeed); //摩擦轮计算pid
	    M2006s[Fric_R].outCurrent = Fric_R_IPID.CLOUD_Incremental_PID(&Fric_R_IPID, M2006s[Fric_R].targetSpeed, M2006s[Fric_R].realSpeed);
			if( M2006_Reload_1.realSpeed == 0||M2006s[1].realSpeed == 0)
		 {
			shot.Disability = 0;
			for(int i = 0; i<2;i++)
			{
				M2006s[i].outCurrent = 0;
			}
		 }	
 }
		Motor_0x200_Can2_SendData(M2006s[Fric_L].outCurrent,M2006s[Fric_R].outCurrent,0,0);
		roll_pos.targetAngle = M2006_Reload_3.realAngle;
    M2006_Reload_3.turnCount = 0;
    M2006_Reload_3.totalAngle = 0;
}

void Speed_Reme(void)
{
	roll_pos.totalAngle = M2006_Reload_3.totalAngle; //获取拨盘电机角度
	M2006s[Fric_L].realSpeed = M2006_Reload_1.realSpeed;  //获取摩擦轮速度
	M2006s[Fric_R].realSpeed = M2006_Reload_2.realSpeed;  //获取摩擦轮速度
}

void FricSpeed_Change(void)
{
//	if(Robot.ChassisWorkMode == ChassisWorkMode_Lock && onlineyaw == 0)
//	{
//		FricSpeed = FricSpeed_out;
//	}
//	else
//	{
		if(Robot.ControlSource != ControlSource_PC )    
		{
			Shoot_Down_Flag = 0;
		}
		
		if(RM_ME[0].FricSpeed == 15)
		{
			FricSpeed = FricSpeed_15 - Shoot_Down * Shoot_Down_Flag;
			Start_Speed = FricSpeed;
		}
		else if(RM_ME[0].FricSpeed == 18)
		{
			FricSpeed = FricSpeed_18 - Shoot_Down * Shoot_Down_Flag;
			Start_Speed = FricSpeed;
		}
		else if(RM_ME[0].FricSpeed == 30)
		{
			FricSpeed = FricSpeed_30 - Shoot_Down * Shoot_Down_Flag;
			Start_Speed = FricSpeed;
		}
		else
		{
			FricSpeed = Start_Speed - Shoot_Down * Shoot_Down_Flag;
		}
//	}
};

/**
 * @brief 自动射击模式
 * 
 * @param target 
 * @param value 
 * @return int 
 */
uint8_t Vision_Shot_Time = 0;
uint8_t Vision_Shot_Hold = 0;
void Auto_Shoot(void)
{
	//以下为用摇杆的普通视觉模式不自动击打代码
	if(Robot.ControlSource != ControlSource_PC && onlineyaw == 1&& (To_mode == 6 || To_mode == 2))    
	{
			if(DR16.rc.ch4_DW  >= 620)
			{
				shot.Key_need ++;
			}
			else
			{
				shot.Key_need = 0;
			}

	}


//以下为用摇杆的小陀螺模式自动击打的代码

	if(Robot.ControlSource != ControlSource_PC && onlineyaw == 1 && To_mode == 5)    
	{
			if(DR16.rc.ch4_DW  >= 620)
			{
				if(VisionData.RawData.auto_aim == 1 && VisionData.RawData.enemy == 1)
				{
					if(Vision_Shot_Hold == 0)
					{
						shot.Key_need ++;
						Vision_Shot_Hold = 1;
					}
				}
				else if(VisionData.RawData.auto_aim == 0)
				{
					shot.Key_need = 0;
				}
			}
			else
			{
				shot.Key_need = 0;
			}

	}
	
	if(Robot.ControlSource != ControlSource_PC && onlineyaw == 0)
	{
		shot.Key_need = 0;
	}
	
	if(Robot.ControlSource == ControlSource_PC && To_mode == 5)
	{
		if(((VisionData.RawData.auto_aim == 1 && VisionData.RawData.enemy == 1) || DR16_Fun.GetKeyMouseAction(MOUSE_Left, KeyAction_CLICK) || DR16_Fun.GetKeyMouseAction(MOUSE_Left, KeyAction_PRESS)) && Auto_Shot == 1)
		{
				if(Vision_Shot_Hold == 0)
				{
					shot.Key_need ++;
					Vision_Shot_Hold = 1;
				}
		}
		else 
		{
			shot.Key_need = 0;
		}
	}
	
	if(Vision_Shot_Hold == 1)
	{
		Vision_Shot_Time ++;
		if(Vision_Shot_Time >= 85)
		{
			Vision_Shot_Hold = 0;
			Vision_Shot_Time = 0;
		}
	}
	
};

