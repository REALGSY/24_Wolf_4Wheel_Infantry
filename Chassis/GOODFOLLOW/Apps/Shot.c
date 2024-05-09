#include "shot.h"
#include "M2006_Motor.h"
#include "Monitor_RM_CAN.h"
#include "PID.h"
#include "DR16_Remote.h"
#include "tim.h"
#include "Robot_control.h"
#include "PID.h"
M2006s_t M2006s[3];
M2006s_t roll_inc;
M2006s_t roll_pos;
Shot_t shot;
uint8_t Fric_L = 0;
uint8_t Fric_R = 1;
//摩擦轮转速
int16_t FricSpeed_15 = 4120;
int16_t FricSpeed_18 = 4570;
int16_t FricSpeed_22 = 5400;
int16_t FricSpeed_30 = 6920;


//摩擦轮和拨盘pid赋值

cloud_incrementalpid_t Fric_L_IPID    = Fric_L_IPIDInit;
cloud_incrementalpid_t Fric_R_IPID    = Fric_R_IPIDInit;
cloud_positionpid_t    Fric_MotorOPID  = Fric_MotorOPIDInit;
cloud_incrementalpid_t Fric_MotorIPID  = Fric_MotorIPIDInit;



/**
 * @brief 发射机构总控制
 * 
 * @param target 
 * @param value 
 * @return int 6
 */
 
void roll_shot_go (void) //摩擦轮函数
{
	Speed_Reme();
	if(Robot.Device_FrictMode == FrictWorkMode_HighSpeed)
	{
		shot.Disability = 1;		
		Roll_Init();//摩擦轮初始化
		Roll_Preload();//预装载
		Roll_Mode(); //roll模式	
		Shoot_MotorInObstruct();
		State_detection();
		shot_IRQHandler();
		Motor_0x200_Can2_SendData(M2006s[0].outCurrent,M2006s[1].outCurrent,roll_inc.outCurrent,0);
	if(shot.maybe_stop == 1)
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
	if(time == 0)
  {
	 roll_pos.targetAngle = M2006_Reload_3.realAngle;
   M2006_Reload_3.totalAngle = 0;
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
	M2006s[Fric_L].targetSpeed = FricSpeed_30; //摩擦轮速度
	M2006s[Fric_R].targetSpeed = - FricSpeed_30; 
	M2006s[Fric_L].outCurrent = Fric_L_IPID.CLOUD_Incremental_PID(&Fric_L_IPID, M2006s[0].targetSpeed, M2006s[0].realSpeed); //摩擦轮计算pid
	M2006s[Fric_R].outCurrent = Fric_R_IPID.CLOUD_Incremental_PID(&Fric_R_IPID, M2006s[1].targetSpeed, M2006s[1].realSpeed);
	roll_pos.outCurrent = Fric_MotorOPID.CLOUD_Position_PID(&Fric_MotorOPID,roll_pos.targetAngle,roll_pos.totalAngle);  //拨轮外环（位置环）
	roll_inc.outCurrent = Fric_MotorIPID.CLOUD_Incremental_PID(&Fric_MotorIPID, roll_pos.outCurrent, M2006_Reload_3.realSpeed); //拨轮内环 （速度环）
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
	if(rc.roll > 550 && rc.roll < 650 )
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
	if(rc.roll == 660)
	{
		shot.roll_get_time ++;
		if(shot.roll_get_time > 45)
		{
			shot.roll_get_time = 45;
		}
	}
	
	if (shot.roll_get_time == 45)
	{
		if_or_no();
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
		
		if( shot.time_up >= 90 )
	  {
			shot.maybe_stop = 1 ; //超过卡弹时间，判断为卡弹 
	  }
		
		if (shot.maybe_stop == 1)
		{
			roll_pos.targetAngle -= 1*one_part_roll;
			shot.time_up = 0;
		}
	}
}


void Off_Shot(void)
{
	if(rc.roll <= -460)
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1850);
	}
	else
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 630);
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
