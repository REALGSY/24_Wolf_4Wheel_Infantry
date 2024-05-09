/**
 * @file Chassis_control.c
 * @author Gsy
 * @brief 
 * @version 1
 * @date 2022-07-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
 #include "Chassis_control.h"
 #include "IMU_Compensate.h"
 #include "M6020_Motor.h"
 #include "M3508_Motor.h"
 #include "Monitor_RM_CAN.h"
 #include "DEV_CIMU.h"
 #include "DJI_IMU.h"
 #include "Control_Vision.h"
 #include "DR16_Remote.h"
 #include "SpeedRamp.h"
 #include "Cloud_control.h"
 #include "arm_math.h"
 #include "DEV_Buzzer.h"
 #include <math.h>
 #include "kalman_filter.h"
 #include "Power_control.h"
 #include "RM_JudgeSystem.h"
 #include "USER_Filter.h"
 #include "DEV_SupCap.h"
/*******************************�û����ݶ���************************************/
int start_go;
int Ship_time = 0;
int ROLL_GO =0;
int Follow_P_Out;
int Follow_I_Out;
int Follow__Out;
int32_t turns_cnt = 0;
uint16_t init_cnt = 0;
uint8_t init_mode = true;
extKalman_t Kalman_CHASFollow_Speed;
void Variable_Spin(float *Omega);
void Limit_Omega(float *target,float Min,float Max);
/*******************************************************************************/
/**
 * @brief PID��ֵ
 * 
 * @param target 
 * @param value 
 * @return int 
 */
 
cloud_positionpid_t M3508s_FloowOPID = M3508s_FloowOPIDInit;
cloud_incrementalpid_t LFWheel_PID = LFWHEEL_PID_PARAM;
cloud_incrementalpid_t RFWheel_PID = RFWHEEL_PID_PARAM;
cloud_incrementalpid_t LBWheel_PID = LBWHEEL_PID_PARAM;
cloud_incrementalpid_t RBWheel_PID = RBWHEEL_PID_PARAM;
cloud_incrementalpid_t *Wheel_PID[] = {&LFWheel_PID, &RFWheel_PID, &LBWheel_PID, &RBWheel_PID};

cloud_positionpid_t RF_Ship_P_PID = RF_Ship_PID_P_PARAM;
cloud_positionpid_t LF_Ship_P_PID = LF_Ship_PID_P_PARAM;
cloud_positionpid_t LB_Ship_P_PID = LB_Ship_PID_P_PARAM;
cloud_positionpid_t RB_Ship_P_PID = RB_Ship_PID_P_PARAM;
cloud_positionpid_t *Ship_P_PID[] = {&RF_Ship_P_PID, &LF_Ship_P_PID, &LB_Ship_P_PID, &RB_Ship_P_PID};

cloud_positionpid_t RF_Ship_I_PID = RF_Ship_I_PID_PARAM;
cloud_positionpid_t LF_Ship_I_PID = LF_Ship_I_PID_PARAM;
cloud_positionpid_t LB_Ship_I_PID = LB_Ship_I_PID_PARAM;
cloud_positionpid_t RB_Ship_I_PID = RB_Ship_I_PID_PARAM;
cloud_positionpid_t *Ship_I_PID[] = {&RF_Ship_I_PID, &LF_Ship_I_PID, &LB_Ship_I_PID, &RB_Ship_I_PID};

cloud_positionpid_t Follow_P_PID  = Follow_P_PID_PARAM;
cloud_positionpid_t Follow_I_PID  = Follow_I_PID_PARAM;
/**
 * @brief  �����ķ���ٶȽ���
 * @param[in]  Vx		x���ٶ�
 *				Vy		y���ٶ�
 *				VOmega	��ת�ٶ�
 * @param[out]	Speed	�ٶ�
 * @retval None
 */

void MecanumCalculate(float X_Move,float Y_Move,float Yaw ,int16_t *Speed)
{
    float Target_velocity[4]={0}; 
    float MaxSpeed = 0.0f;
    float Param = 1.0f;
 
	  Target_velocity[0] = 	X_Move - Y_Move + Yaw;
		Target_velocity[1] =    X_Move + Y_Move + Yaw;
		Target_velocity[2] =   -X_Move + Y_Move + Yaw;
		Target_velocity[3]=    -X_Move - Y_Move + Yaw;	

    for(uint8_t i = 0 ; i<4 ; i ++)
    {
        if(abs(Target_velocity[i]) > MaxSpeed)
        {
            MaxSpeed = abs(Target_velocity[i]);
        }
    }

    if (MaxSpeed > WheelMaxSpeed)
    {
        Param = (float)WheelMaxSpeed / MaxSpeed;
    }

    Speed[0] = Target_velocity[0] * Param;
    Speed[1] = Target_velocity[1] * Param;
    Speed[2] = Target_velocity[2] * Param;
    Speed[3] = Target_velocity[3] * Param;

}

/**
 * @brief  �����˶�����(������)
 * @param[in]  Vx		x���ٶ�
 *				Vy		y���ٶ�
 *				VOmega	��ת�ٶ�
 * @param[out]	Speed	�ٶ�
 * @retval None
 */

int8_t drct=1;//���������������ת

void Wheel_calc(float X_Move,float Y_Move,float Yaw ,int16_t *Speed) 
{
		float Target_velocity[4]={0}; 
	  //3508Ŀ���ٶȼ���
    float MaxSpeed = 0.0f;
    float Param = 1.0f;
    float const theta = atan(1.0/1.0);
		
				if(Robot.ChassisWorkMode == ChassisWorkMode_Square)
				{
				  Target_velocity[0] = -sqrt(pow(X_Move + Yaw*0,2) + pow(Y_Move + Yaw*1,2)) ;
					Target_velocity[1] = sqrt(pow(X_Move - Yaw*1,2) + pow(Y_Move + Yaw*0,2)) ;
					Target_velocity[2] =  sqrt(pow(X_Move - Yaw*0,2) + pow(Y_Move - Yaw*1,2)) ;
					Target_velocity[3] = - sqrt(pow(X_Move + Yaw*1,2) + pow(Y_Move - Yaw*0,2)) ; 
				}
				else
				{
					Target_velocity[0] = -sqrt(pow(X_Move - Yaw*arm_sin_f32(theta),2) + pow(Y_Move + Yaw*arm_cos_f32(theta),2)) ;
					Target_velocity[1] =  sqrt(pow(X_Move - Yaw*arm_sin_f32(theta),2) + pow(Y_Move - Yaw*arm_cos_f32(theta),2)) ;
					Target_velocity[2] =  sqrt(pow(X_Move + Yaw*arm_sin_f32(theta),2) + pow(Y_Move - Yaw*arm_cos_f32(theta),2)) ; 
					Target_velocity[3] = - sqrt(pow(X_Move + Yaw*arm_sin_f32(theta),2) + pow(Y_Move + Yaw*arm_cos_f32(theta),2)) ;

				}

		
    for(uint8_t i = 0 ; i < 4 ; i ++)
    {
        if(abs(Target_velocity[i]) > MaxSpeed)
        {
            MaxSpeed = abs(Target_velocity[i]);
        }
    }

    if (MaxSpeed > WheelMaxSpeed)
    {
        Param = (float)WheelMaxSpeed / MaxSpeed;
    }

    Speed[0] = Target_velocity[0] * Param;
    Speed[1] = Target_velocity[1] * Param;
    Speed[2] = Target_velocity[2] * Param;
    Speed[3] = Target_velocity[3] * Param;
		
	RUDTotalAngle_Calc(0,0);
	RUDTotalAngle_Calc(1,0);
	RUDTotalAngle_Calc(2,0);
	RUDTotalAngle_Calc(3,0);
	
	RUDTargetAngle_Calc(0,0,0);
	RUDTargetAngle_Calc(1,0,0);
	RUDTargetAngle_Calc(2,0,0);
	RUDTargetAngle_Calc(3,0,0);
}

/**
 * @brief  �����˶�����(ת����)
 * @param[in]  Vx		x���ٶ�
 *				Vy		y���ٶ�
 *				VOmega	��ת�ٶ�
 * @param[out]	Speed	�ٶ�
 * @retval None
 */
float nmsl1 = 0;
float nmsl2 = 0;
float nmsl3 = 0;
float nmsl4 = 0;
uint8_t No_move_flag = false;
uint8_t spin_flag;
uint16_t Brake_cnt;//--- ɲ��
uint8_t Move_flag;
int gogogogogo = 0;
int shshshsh = 0;
void Ship_calc(float X_Move,float Y_Move,float Yaw ,float *Angle) 
{
    float const theta = atan(1.0/1.0);
		float Target_velocity[4]={0};
    float Radius = 1.0f;  // Բ�ľ�		
    static uint16_t No_move_cnt = 0;

    static float last_vx = 0, last_vy = 0, last_vw = 0;
    if(X_Move == 0 && Y_Move == 0)
    {
        No_move_flag = true;
        if(abs(Yaw) < 70) //---IMU��Ư�����������ٶ�
        {
            if(No_move_cnt < 500) //--- ��ֹ��1000ms�ڼ䲻������̸���
            {
                No_move_cnt++;
                Yaw = 0;
            }
            else
            {}
        }
    }
    else
    {
        spin_flag = false;
        No_move_flag = false;
        No_move_cnt = 0;
    }
    if(X_Move == 0 && Y_Move == 0 && Yaw == 0)
    {
        Move_flag = false;

        if(Brake_cnt < 500)
        {
            Brake_cnt++;

            //--- ����һ֡���Ǿ�ֹ��ʱ�����һ��Vw���ٶȣ�Ϊ�˲�����Ŀ��Ƕ���һ������Ķ���
            last_vw = (spin_flag == true ? 50 : 0);
						gogogogogo = 0;
            //--- ʹ����һ�ε�Ŀ���ٶ���������һ�ε�Ŀ��Ƕ�
				if(Robot.ChassisWorkMode == ChassisWorkMode_Square)
				{
					Target_velocity[0] = atan2(last_vx + last_vw*(0),last_vy + last_vw*1)*(180/PI);	
					Target_velocity[1] = atan2(last_vx - last_vw*(1),last_vy + last_vw*0)*(180/PI);
					Target_velocity[2] = atan2(last_vx - last_vw*(0),last_vy - last_vw*1)*(180/PI);
					Target_velocity[3] = atan2(last_vx + last_vw*(1),last_vy - last_vw*0)*(180/PI);		
				}
				else
				{
		  		Target_velocity[0] = atan2(last_vx - last_vw*(Radius*arm_sin_f32(theta)),last_vy + last_vw*Radius*arm_cos_f32(theta))*(180/PI);
					Target_velocity[1] = atan2(last_vx - last_vw*(Radius*arm_sin_f32(theta)),last_vy - last_vw*Radius*arm_cos_f32(theta))*(180/PI);
					Target_velocity[2] = atan2(last_vx + last_vw*(Radius*arm_sin_f32(theta)),last_vy - last_vw*Radius*arm_cos_f32(theta))*(180/PI);
					Target_velocity[3] = atan2(last_vx + last_vw*(Radius*arm_sin_f32(theta)),last_vy + last_vw*Radius*arm_cos_f32(theta))*(180/PI);
				}


        
        }
        else
        {
            spin_flag = false;
						gogogogogo = 1;
            //--- 45�ȹ���
		if(Robot.ChassisWorkMode == ChassisWorkMode_Square)
		{
			M6020s[0].Init_angle =  (-45) +130.91f;
			M6020s[1].Init_angle =  (+45) +190.99f;
			M6020s[2].Init_angle =  (-45) +131.62f;
			M6020s[3].Init_angle =  (+45) +107.18f;
		}
		else
		{
			M6020s[0].Init_angle =  (-45) +130.91f;
			M6020s[1].Init_angle =  (+45) +190.99f;
			M6020s[2].Init_angle =  (-45) +131.62f;
			M6020s[3].Init_angle =  (+45) +107.18f;
		}
				


            //--- Ŀ��Ƕȹ���
            for(uint8_t i = 0 ; i < 4 ; i++)
            {
                Target_velocity[i] = 0;
            }
        }

    }
    else
    {
        if(No_move_flag != true)
        {
            Brake_cnt = 0;
        }
        Move_flag = true;
		gogogogogo = 0;
        //--- ���45�ȹ���
		if(Robot.ChassisWorkMode == ChassisWorkMode_Square)
		{
			M6020s[0].Init_angle =  (-45) +130.91f;
			M6020s[1].Init_angle =  (-45) +190.99f;
			M6020s[2].Init_angle =  (-45) +131.62f;
			M6020s[3].Init_angle =  (-45) +107.18f;
		}
		else
		{
//			M6020s[0].Init_angle =  85.91f;
//			M6020s[1].Init_angle =  235.99f;
//			M6020s[2].Init_angle =  86.62f;
//			M6020s[3].Init_angle =  152.18f;
			M6020s[0].Init_angle =  130.91f;
			M6020s[1].Init_angle =  190.99f;
			M6020s[2].Init_angle =  131.62f;
			M6020s[3].Init_angle =  107.18f;
		}

        //--- ��Ŀ���ٶȵ�ʱ��Ž��ж��ֽ���ļ���
		if(Robot.ChassisWorkMode == ChassisWorkMode_Square)
		{
		Target_velocity[0] = atan2(X_Move + Yaw*(0),Y_Move + Yaw*1)*(180/PI);		
		Target_velocity[1] = atan2(X_Move - Yaw*(1),Y_Move + Yaw*0)*(180/PI);
		Target_velocity[2] = atan2(X_Move - Yaw*(0),Y_Move - Yaw*1)*(180/PI);
		Target_velocity[3] = atan2(X_Move + Yaw*(1),Y_Move - Yaw*0)*(180/PI);	
		}
		else
		{
		Target_velocity[0] = atan2(X_Move - Yaw*(Radius*arm_sin_f32(theta)),Y_Move + Yaw*Radius*arm_cos_f32(theta))*(180/PI);
		Target_velocity[1] = atan2(X_Move - Yaw*(Radius*arm_sin_f32(theta)),Y_Move - Yaw*Radius*arm_cos_f32(theta))*(180/PI);
		Target_velocity[2] = atan2(X_Move + Yaw*(Radius*arm_sin_f32(theta)),Y_Move - Yaw*Radius*arm_cos_f32(theta))*(180/PI);
		Target_velocity[3] = atan2(X_Move + Yaw*(Radius*arm_sin_f32(theta)),Y_Move + Yaw*Radius*arm_cos_f32(theta))*(180/PI);
		}




        if(abs(Yaw)>100)
        {
            spin_flag = true;
        }

        //--- ��Ŀ���ٶȵ�ʱ��ʹ����һ�νǶ�����������Ϊ����ģʽ��IMU��ֹ��˲��������΢��Vw�ٶ�
        last_vx = X_Move;
        last_vy = 0;
        last_vw = 0;

    }
		
    Angle[0] = Target_velocity[0];
    Angle[1] = Target_velocity[1];
    Angle[2] = Target_velocity[2];
    Angle[3] = Target_velocity[3];
}

/**
 * @brief  ͨ��6020��е�Ƕȵķ�ʽ��ȡ��̨Yaw��ת�ĽǶȣ�ƫ�Ƴ���ǰ���ĽǶ�-���ĵ㣩
 * @param[in]  None
 * @retval 360�ȵĽǶ�ֵ��
 */
float Cloud_getYawAngleWithCenter(void)
{
    return (rc.ch0)/22.7527f ;
}


/**
 * @brief  ȫ���ƶ���ʽ
 * @param[in]  Vx		x���ٶ�
 *				Vy		y���ٶ�
 *				VOmega	��ת�ٶ�
 * @param[out]	Speed	�ٶ�
 * @retval None
 */

uint8_t Yaw_Spin_Error = 22;  //С�����ƶ�ʱyaw�����
static void Omnidirectional_Formula(float *Vx, float *Vy)
{
    float RadRaw = 0.0f;
    float temp_Vx = 0.0f;
		
		if(ext_game_robot_state.data.chassis_power_limit == 80)
		{
			Yaw_Spin_Error = 22;
		}
		else
		{
			Yaw_Spin_Error = 17;
		}
		
		if(SupCap_C_Flag == 1 && Supcap_Rec.RecvData.Pack.cap_cell >= 40)
		{
			Yaw_Spin_Error = 12;
		}
		
    float angle =  Cloud_getYawAngleWithCenter() + Yaw_Spin_Error; //��е�Ƕ�ƫ��
    RadRaw = angle * DEG_TO_RAD;                           //����ƫ��
    temp_Vx = *Vx;
    *Vx = *Vx * arm_cos_f32(RadRaw) - *Vy * arm_sin_f32(RadRaw);
    *Vy = *Vy * arm_cos_f32(RadRaw) + temp_Vx * arm_sin_f32(RadRaw);
}



/**
 * @brief ���㴦��
 * 
 * @param target 
 * @param value 
 * @return int 
 */
 float ComputeMinOffset(float target, float value) //������Сƫ����̸���Ӧ�����ĸ�����ȥ��ɸ��涯����
{
    int err = target - value;

    if (err > 180.0f)
    {
        return (target - 360.0f);
    }
    else if (err < -180.0f)
    {
        return (target + 360.0f);;
    }
		else
	  {
		return target;
	  }  
}

/**
 * @brief ��ȡ�������������λ�ã�����Ϊ��׼
 * 
 * @param target 
 * @param value 
 * @return int 
 */
int a_start = 0;
float	start_imu; 
void read_start_imu (void)  //��ȡ�������������λ�ã�����Ϊ��׼
{
	if(IMU_Init_Condition == 1)
	{
	if (a_start == 0)
	{	
		for(int i = 0;i<10;++i)
		{
			M6020s_Yaw.targetAngle   = start_imu ;
			M6020s_Pitch.targetAngle =  pitch_start;
			start_imu =  DJI_C_IMU.total_yaw ;
			pitch_start = Cloud_Pitch_Center;
			a_start = 1 ;
			M6020s_Yaw.rc_targetAngle   = 0;
		  M6020s_Pitch.rc_targetAngle = 0;
		}
	}
 }
}
/**
 * @brief ҡ����������
 * 
 * @param target 
 * @param value 
 * @return int 
 */

void rc_dead(float *rc_dead)
{
	if(*rc_dead >= 0 && *rc_dead <= 20)
	{
		*rc_dead = 0;
	}
	else if(*rc_dead <= 0 && *rc_dead >= -20)
	{
		*rc_dead = 0;
	}
}



/**
 * @brief �����ܿ���
 * 
 * @param target 
 * @param value 
 * @return int 
 */
float angle[4];
int16_t speed[4];
float i_VOmega;
int SpinDirection_Flag = -1; /* С���ݷ��� */
uint8_t Break_stop = 0;
uint16_t Break_time = 0;
float Low_VOmega;
float Ramp_Vy, Ramp_Vx, Ramp_Vw;
float ACCCCC_VAL = 17.0f, DECCCCC_VAL = 20.0f;/*10.0f*/
float kaerman;
float meika;
int16_t drv_tempcurrent[4];
int16_t ainimemeda;
float  last_roll;
int ReF_Flag = 0;
int ReF_Cnt = 0;
void Ship_ChassisWorkMode_Follow(float Vx, float Vy, float VOmega)
{

    if(Robot.ChassisWorkMode == ChassisWorkMode_CloudFollow || Robot.ChassisWorkMode == ChassisWorkMode_Lock || Robot.ChassisWorkMode == ChassisWorkMode_Square)
    {
			
			if(Robot.ChassisWorkMode == ChassisWorkMode_Lock)
    {
       Vx = Vy = VOmega = 0;
    }

		
		Set_MaxSpeed();
		if(Robot.ChassisWorkMode == ChassisWorkMode_CloudFollow || Robot.ChassisWorkMode == ChassisWorkMode_Square)
		{
			if(Vx == 0 && Vy == 0)
			{
                if(abs(VOmega) < 30) 
                {
                    VOmega = 0;
                }
			}

			if(Vy == -660)
			{ 
				Vx = Vy = VOmega = 0;
			}

			Vx = Vx * 15;
			Vy = Vy * 15;
		}

//					if(Vx == 0 && Vy == 0 && fabs(VOmega) < 70)
//		  {
//			   Vx = Vy = VOmega =0;
//		  }
			
		Ramp_Vx = Vx;
		Ramp_Vy = Vy;

		
		//Ramp_Vw = KalmanFilter(&Kalman_CHASFollow_Speed, VOmega);
				
		Filter_IIRLPF(&VOmega, &Ramp_Vw, 0.2); //����������ʱ���̸�����̨
		
		if(abs(Ramp_Vw) > 1000)
		{
			Ramp_Vx = Ramp_Vx * 0.7f;
			Ramp_Vy = Ramp_Vy * 0.7f;
		}			
		
		
	  meika = Ramp_Vw;//��Ҳ�Ǹ���λ������
		
	    if(rc.roll>100&&Robot.Device_FrictMode != FrictWorkMode_HighSpeed && Robot.ChassisWorkMode != ChassisWorkMode_Lock)
	    {
		    Vx = Vx ;
		    Vy = Vy ;  
	      Omnidirectional_Formula(&Vx,&Vy);
        i_VOmega *= SpinDirection_Flag;
			if(abs(Vx) < 20 && abs(Vy) < 20)
			{
				i_VOmega = rc.roll * 8.0f * SpinDirection_Flag;
				//Variable_Spin(&i_VOmega);
			}
			else
			{
				i_VOmega = rc.roll * 6.0f * SpinDirection_Flag;
			}
				ainimemeda = i_VOmega;
	      ROLL_GO = 1;
			  ReF_Flag = 1;
	    }
	    else
	    {
		    i_VOmega = 0;
 		    ROLL_GO = 0;
	    }
		
			
			last_roll = rc.roll;
			
		if(Robot.ChassisWorkMode == ChassisWorkMode_Lock)
		{
			M6020s_Yaw.realSpeed = 0;
		}
		
	    if(ROLL_GO == 1)
	    {
	        Ship_calc(Vx,Vy,i_VOmega *2 ,angle);
	        Wheel_calc(Vx,Vy,i_VOmega *2,speed);
	    }
	    else
	    {
					if( ReF_Flag == 1)
					{
						 ReF_Cnt++;
						if (abs(rc.ch0) <= 50.0f)
						{
							ReF_Flag = 0;
						}
						if (ReF_Cnt > 2000 && ReF_Flag != false) // --- ��ʱδת����ֱ��ת��
						{
							ReF_Flag = 0;
							ReF_Cnt = 0;
						}
						Ramp_Vx = Vx;
						Ramp_Vy = Vy;
					  Omnidirectional_Formula(&Ramp_Vx,&Ramp_Vy);
					}
					Follow_P_Out = Follow_P_PID.CLOUD_Position_PID(&Follow_P_PID,Ramp_Vw,0); 
					Follow_I_Out = Follow_I_PID.CLOUD_Position_PID(&Follow_I_PID,Follow_P_Out,M6020s_Yaw.realSpeed);

	        Ship_calc(Ramp_Vx,Ramp_Vy,Follow_I_Out,angle);
	        Wheel_calc(Ramp_Vx,Ramp_Vy,Follow_I_Out,speed);
	    }		

	    for(int i = 0; i<4; i++)
	    {

				M6020s[i].Ship_error = ComputeMinOffset(angle[i], M6020s[i].RUD_totalAngle);
				M6020s[i].p_outCurrent = Ship_P_PID[i]->CLOUD_Position_PID(Ship_P_PID[i],M6020s[i].Ship_error , M6020s[i].RUD_totalAngle);

				M6020s[i].i_outCurrent = Ship_I_PID[i]->CLOUD_Position_PID(Ship_I_PID[i],M6020s[i].p_outCurrent,M6020s[i].realSpeed);
				M3508s[i].i_outCurrent = Wheel_PID[i]->CLOUD_Incremental_PID(Wheel_PID[i],  speed [i]  , M3508s[i].realSpeed); 
	    }


		//��ʱ��ע�͹������ƴ�����	
      for(uint8_t i = 0 ; i < 4 ; i++)
      {
         drv_tempcurrent[i] = M3508s[i].i_outCurrent;
      }
      Limit(drv_tempcurrent,4);
	    for(uint8_t i = 0 ; i < 4 ; i++)
      {
          M3508s[i].i_outCurrent = drv_tempcurrent[i];
      } 

	    Motor_0x200_SendData(M3508s[0].i_outCurrent,M3508s[1].i_outCurrent,M3508s[2].i_outCurrent,M3508s[3].i_outCurrent);
	    Motor_0x1FF_SendData(M6020s[0].i_outCurrent,M6020s[1].i_outCurrent,M6020s[2].i_outCurrent,M6020s[3].i_outCurrent);

	}
	else
	{
	    Motor_0x200_SendData(0,0,0,0);
	    Motor_0x1FF_SendData(0,0,0,0);
	}
	
}


/**
 * @brief  ���̿��ƴ���-������̨
 * @param[in]  Vx		x���ٶ�
 *				Vy		y���ٶ�
 *				Omega	ƫ���
*				mode	ģʽ - ��Status_ControlOFF�⣬������������
 * @retval None
 */
void Chassis_processing(float Vx, float Vy, float VOmega ,float VPitch)
{
//	rc_dead(&Vx);
//	rc_dead(&Vy);
//	rc_dead(&VOmega);
//	rc_dead(&VPitch);
	Ship_ChassisWorkMode_Follow(Vx,Vy,VOmega);
	
}
/**
 * @brief  ���̿��ƴ���-��ͳ����
 * @param[in]  Vx		x���ٶ�
 *				Vy		y���ٶ�
 *				Omega	ƫ���
*				mode	ģʽ - ��ͳ
 * @retval None
 */
void Cloud_processing(float Vx, float Vy, float VOmega ,float VPitch)
{
//	rc_dead(&Vx);
//	rc_dead(&Vy);
//	rc_dead(&VOmega);
//	rc_dead(&VPitch);
	YawWorkMode_Follow(Vx,Vy,VOmega,VPitch);
	
}

void Chassis_Init(void)
{
	M6020s[0].Init_angle = -83.51f;
	M6020s[1].Init_angle = (+45)-88.9;
	M6020s[2].Init_angle = -80.46;
	M6020s[3].Init_angle = (+45)+151.69;
  start_go = 1;
}

void Robot_Init(void)
{
	Robot.ChassisWorkMode = ChassisWorkMode_Disable;
	Robot.Device_FrictMode=FrictWorkMode_Disable;
	KalmanCreate(&Kalman_CHASFollow_Speed, 1, 100);
}

void RUDTotalAngle_Calc(int i ,uint8_t opposite)
{   
    float Cur_encoder = M6020s[i].realAngle/8192*360;
    static float Pre_encoder[4] = {0};
    if(Cur_encoder - Pre_encoder[i] > 180)
    {
        M6020s[i].RUD_turnCount--;
    }
    else if(Cur_encoder - Pre_encoder[i] < -180)
    {
        M6020s[i].RUD_turnCount++;
    }
    Pre_encoder[i] = Cur_encoder;
//		
//		if(gogogogogo == 1)
//		{
//			M6020s[i].RUD_turnCount = 0;
//		}
	  
    M6020s[i].RUD_totalAngle = Cur_encoder  + M6020s[i].RUD_turnCount*360;
}

float fl_error;


void RUDTargetAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite)
{
    float Cur_Target = angle[motor_num];

    /*--------------------------------*/
    //--- ��Ŀ��Ƕ��뵱ǰ�ǶȻ��㵽ͬһ��������
    turns_cnt = (int32_t)(M6020s[motor_num].RUD_totalAngle/180.0f) - (int32_t)(M6020s[motor_num].RUD_totalAngle/360.0f);
//		if(gogogogogo == 1)
//		{
//			turns_cnt = 0;
//			Cur_Target = 0;
//		}
//	
    angle[motor_num] = 360.0f*turns_cnt + Cur_Target + M6020s[motor_num].Init_angle;

    //---- ��ǰĿ��ǶȺ͵�ǰ�Ƕȵ����
    fl_error = angle[motor_num] - M6020s[motor_num].RUD_totalAngle;

    //--- ����޷�
    // Constrain(&error, -180.0f, 180.0f); //--- �������޷����������е�����
    AngleLimit(&fl_error);

    //--- ����Ƕ�������90�����ٶȷ��򲢽�Ŀ��Ƕȵ���180��
    if(fabs(fl_error) > 90.0f && (Move_flag == true|| Brake_cnt < 500))
    {
        //--- Ŀ��ֵ���Ӱ������
        angle[motor_num] += 180.0f;
        //--- �����ַ�ת
        speed[motor_num] = -speed[motor_num];
        //--- ȷ��Ŀ��ǶȺ͵�ǰ�Ƕȴ���ͬһ������
        if( angle[motor_num] > turns_cnt*360.0f + 180.0f)
        {
            angle[motor_num] -= 360.0f;
        }
        else if(angle[motor_num] < turns_cnt*360.0f - 180.0f)
        {
            angle[motor_num] += 360.0f;
        }
    }
    /*--------------------------------*/

}

void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes < 100)
	{
		if(*angle > 180.0f)
		{
			*angle -= 360.0f;
			AngleLimit(angle);
		}
		else if(*angle < -180.0f)
		{
			*angle += 360.0f;
			AngleLimit(angle);
		}
	}
	recursiveTimes--;
}

/**
 * @brief      �����ٶ�б��
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Drv_Slow(float *rec , float target , float slow_Inc, float Accval, float DecVal)
{

    if(abs(*rec) - abs(target) < 0)//����ʱ
    {
        if(abs(*rec) > 10)
        {
            slow_Inc = slow_Inc * Accval;//�ٶ���������ʱ������5��
        }
    }
    
    if(abs(*rec) - abs(target) > 0)
    {
        slow_Inc = slow_Inc * DecVal;//����ʱ�Ŵ�15��
    }
    if(abs(*rec - target) < slow_Inc)
    {
        *rec = target;
    }
    else 
    {
        if((*rec) > target) (*rec) -= slow_Inc;
        if((*rec) < target) (*rec) += slow_Inc;
    }
}

void Set_RudMaxOut(uint16_t maxout)
{
    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        Ship_I_PID[i]->MaxOutput = maxout;
    }
}

/**
 * @brief      ��������ٶ�����(��ʱ�������������������ٶ� ���������ֵ����ܲ���ϵͳ���ʻ��ʼƹ������� Ŀǰ������ֻ��������ѹ ��ֱ�����������ֵĹ��� ���Բ�������)
 * @param[in]  None
 * @retval     None
 */
void Set_MaxSpeed(void)
{
#if USE_RM_Referee
//    if (SupCap.SendData.is_cap_output == OFF || DevicesMonitor.Get_State(SUPCAP_MONITOR) == Off_line)
//	{
		if (ext_game_robot_state.data.chassis_power_limit < 60 /* && Get_PowerLimit() != 0 */)
		{
//			VxVy_Limit = 4700.0f/* Chassis_SpeedNormal */;
//			Vw_Limit =  4700.0f ;

            Set_RudMaxOut(12000);
		}
		else if(ext_game_robot_state.data.chassis_power_limit == 60)
		{
//			VxVy_Limit = CHASSIS_SPEED_L;
//			Vw_Limit = 5500.0f;

            Set_RudMaxOut(13000);
		}
		else if (ext_game_robot_state.data.chassis_power_limit > 60 && ext_game_robot_state.data.chassis_power_limit <= 80)
		{
//			VxVy_Limit = CHASSIS_SPEED_M;
//			Vw_Limit = CHASSIS_SPEED_M;

            Set_RudMaxOut(18000);
		}
		else if (ext_game_robot_state.data.chassis_power_limit > 80 /* && Get_PowerLimit() <= 120 */)
		{
            if(ext_game_robot_state.data.chassis_power_limit>=200) //--- ����׼���׶ε�200W���ʰ�80W����
            {
//              VxVy_Limit = 7500.0f;
//			    Vw_Limit = 7500.0f;
            }
            else
            {
//                VxVy_Limit = CHASSIS_SPEED_H;
//                Vw_Limit = CHASSIS_SPEED_H;
            }

            Set_RudMaxOut(29999);
		}

//		if(/* Infantry.Write_Msg[4] == ON || */ Get_Power() == 65535)  // --- û�е����һ������޹���
//		{
//			VxVy_Limit = CHASSIS_SPEED_H;
//			Vw_Limit = CHASSIS_SPEED_H;
//		}

//	}
    else
	{
		//--- �����ݵ�ʱ��Ҫ�Ӹ����µ��ر���
//        VxVy_Limit = 8000;
//        Vw_Limit = 8000;
// #if ROBOT_ID == INFANTRY_2022_SWERVE_2
        Set_RudMaxOut(13999);
// #endif
	}
#else
    // if(Low speed mode == true)
	// {
	// 	VxVy_Limit = 3000.0f;
	// 	Vw_Limit = 3000.0f;
	// }
	// else
	// {
		VxVy_Limit = CHASSIS_SPEED_H;
		Vw_Limit = CHASSIS_SPEED_H;
	// }
#endif

//    Constrain(&Target_Vx, (int16_t)(-VxVy_Limit), (int16_t)VxVy_Limit);
//    Constrain(&Target_Vy, (int16_t)(-VxVy_Limit), (int16_t)VxVy_Limit);
//    Constrain(&Target_Vw, (int16_t)(-Vw_Limit), (int16_t)Vw_Limit);

}

/**
 * @brief      ����С���ݴ���
 * @param[in]  None
 * @retval     None
 */
void Variable_Spin(float *Omega)
{
	static float Variable_Angle;
	static float Variable_Radin;
	Variable_Angle += 0.1f;
	
	if(Variable_Angle >= 120.0f)
	{
		Variable_Angle = 60.0f;
	}
	Variable_Radin = Variable_Angle * PI / 180;
	float sin_result = arm_sin_f32(Variable_Radin); // ʹ��arm sin f32()��������sinֵconstrainlsin result, 0.7f. 1.0f) :
	Limit_Omega(&sin_result, 0.7f, 1.0f);
	*Omega *= sin_result; // ������õ���ֵ��ŵ�ָ���ָ��λ��
}

void Limit_Omega(float *target,float Min,float Max)
{
	if(*target <= Min)
	{
		*target = Min;
	}
	else if(*target >= Max)
	{
		*target = Max;
	}
	else
	{
		*target = *target;
	}
}

