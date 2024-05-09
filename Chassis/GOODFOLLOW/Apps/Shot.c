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
//Ħ����ת��
int16_t FricSpeed_15 = 4120;
int16_t FricSpeed_18 = 4570;
int16_t FricSpeed_22 = 5400;
int16_t FricSpeed_30 = 6920;


//Ħ���ֺͲ���pid��ֵ

cloud_incrementalpid_t Fric_L_IPID    = Fric_L_IPIDInit;
cloud_incrementalpid_t Fric_R_IPID    = Fric_R_IPIDInit;
cloud_positionpid_t    Fric_MotorOPID  = Fric_MotorOPIDInit;
cloud_incrementalpid_t Fric_MotorIPID  = Fric_MotorIPIDInit;



/**
 * @brief ��������ܿ���
 * 
 * @param target 
 * @param value 
 * @return int 6
 */
 
void roll_shot_go (void) //Ħ���ֺ���
{
	Speed_Reme();
	if(Robot.Device_FrictMode == FrictWorkMode_HighSpeed)
	{
		shot.Disability = 1;		
		Roll_Init();//Ħ���ֳ�ʼ��
		Roll_Preload();//Ԥװ��
		Roll_Mode(); //rollģʽ	
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
 * @brief Ħ���ֳ�ʼ��
 * 
 * @param target 
 * @param value 
 * @return int 
 */
 
void Roll_Init (void)  //Ħ���ֳ�ʼ�� 
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
 * @brief �������
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void shot_IRQHandler(void)
{
	M2006s[Fric_L].targetSpeed = FricSpeed_30; //Ħ�����ٶ�
	M2006s[Fric_R].targetSpeed = - FricSpeed_30; 
	M2006s[Fric_L].outCurrent = Fric_L_IPID.CLOUD_Incremental_PID(&Fric_L_IPID, M2006s[0].targetSpeed, M2006s[0].realSpeed); //Ħ���ּ���pid
	M2006s[Fric_R].outCurrent = Fric_R_IPID.CLOUD_Incremental_PID(&Fric_R_IPID, M2006s[1].targetSpeed, M2006s[1].realSpeed);
	roll_pos.outCurrent = Fric_MotorOPID.CLOUD_Position_PID(&Fric_MotorOPID,roll_pos.targetAngle,roll_pos.totalAngle);  //�����⻷��λ�û���
	roll_inc.outCurrent = Fric_MotorIPID.CLOUD_Incremental_PID(&Fric_MotorIPID, roll_pos.outCurrent, M2006_Reload_3.realSpeed); //�����ڻ� ���ٶȻ���
}





/**
 * @brief ׼������
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void if_or_no (void)
{
	shot.roll_frequency ++ ;//��¼�������
	roll_pos.targetAngle += one_part_roll ;
	shot.shot_one = 1;
	shot.roll_get_time = 0 ;
}

/**
 * @brief Ħ����Ԥװ��
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
 * @brief ׼������
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
 * @brief ���淢������
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void Shoot_MotorInObstruct (void)
{
	  shot.last_roll_frequency = shot.roll_frequency;  //�����ڵķ�����Ŀ��Ϊ�ϴεķ�����
}

/**
 * @brief ����״̬���
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
			shot.time_up ++ ; //���㿨��ʱ��
		}
		 
		shot.Locked_time ++ ; //ʱ��
		
		if( shot.Locked_time >= 85 )
		{
			shot.Locked_time = 0;
			shot.time_up = 0;
			shot.shot_one = 0 ;
		}
		
		if( shot.time_up >= 90 )
	  {
			shot.maybe_stop = 1 ; //��������ʱ�䣬�ж�Ϊ���� 
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
 * @brief Ħ����ʧ�����ŷ���0������������0
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void Reload_Reset(void)
{
	if (shot.Disability == 1)
 {
			M2006s[Fric_L].targetSpeed = 0; //Ħ�����ٶ�
	    M2006s[Fric_R].targetSpeed = 0; //Ħ�����ٶ�
			M2006s[Fric_L].outCurrent = Fric_L_IPID.CLOUD_Incremental_PID(&Fric_L_IPID, M2006s[Fric_L].targetSpeed, M2006s[Fric_L].realSpeed); //Ħ���ּ���pid
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
	roll_pos.totalAngle = M2006_Reload_3.totalAngle; //��ȡ���̵���Ƕ�
	M2006s[Fric_L].realSpeed = M2006_Reload_1.realSpeed;  //��ȡĦ�����ٶ�
	M2006s[Fric_R].realSpeed = M2006_Reload_2.realSpeed;  //��ȡĦ�����ٶ�
}
