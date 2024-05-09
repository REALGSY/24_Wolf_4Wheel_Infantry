#include "PID.h"

int Test_I=0;

void abs_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

void IncrementalPID_paraReset(incrementalpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit){
	pid_t->Target = 0;
	pid_t->Measured = 0;
	pid_t->err = 0;
	pid_t->err_last = 0;
	pid_t->err_beforeLast = 0;
	pid_t->Kp = kp;
	pid_t->Ki = ki;
	pid_t->Kd = kd;
	pid_t->MaxOutput = MaxOutput;
	pid_t->IntegralLimit = IntegralLimit;
	pid_t->pwm = 0; 			
}

void IncrementalPID_setPara(incrementalpid_t *pid_t, float kp, float ki, float kd){
	pid_t->err = 0;
	pid_t->err_last = 0;
	pid_t->err_beforeLast = 0;
	pid_t->Kp = kp;
	pid_t->Ki = ki;
	pid_t->Kd = kd;	
	pid_t->pwm = 0;	
}


float Incremental_PID(incrementalpid_t *pid_t, float target, float measured) {
	float p_out, i_out, d_out;
	pid_t->Target = target;
	pid_t->Measured = measured;
	pid_t->err = pid_t->Target - pid_t->Measured;
	
//	if(abs(pid_t->err)<0.1f)
//		pid_t->err = 0.0f;
		//return 0;
	
	p_out = pid_t->Kp*(pid_t->err - pid_t->err_last);
	i_out = pid_t->Ki*pid_t->err;
	d_out = pid_t->Kd*(pid_t->err - 2.0f*pid_t->err_last + pid_t->err_beforeLast);
	
	//积分限幅
	abs_limit(&i_out, pid_t->IntegralLimit);
	
	pid_t->pwm += (p_out + i_out + d_out);
	
	//输出限幅
	abs_limit(&pid_t->pwm, pid_t->MaxOutput);
	
	pid_t->err_beforeLast = pid_t->err_last;
	pid_t->err_last = pid_t->err;

	return pid_t->pwm;
}

void PositionPID_paraReset(positionpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit){
	pid_t->Target = 0;
	pid_t->Measured = 0;
	pid_t->MaxOutput = MaxOutput;
	pid_t->IntegralLimit = IntegralLimit;
	pid_t->err = 0;
	pid_t->err_last = 0;
	pid_t->integral_err = 0;
	pid_t->Kp = kp;
	pid_t->Ki = ki;
	pid_t->Kd = kd;
	pid_t->pwm = 0; 			
}

void PositionPID_setPara(positionpid_t *pid_t, float kp, float ki, float kd){
	pid_t->err = 0;
	pid_t->err_last = 0;
	pid_t->integral_err = 0;
	pid_t->Kp = kp;
	pid_t->Ki = ki;
	pid_t->Kd = kd;		
	pid_t->pwm = 0;	
}


float Position_PID(positionpid_t *pid_t, float target, float measured) {
	float p_out, i_out, d_out;
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
	pid_t->err = pid_t->Target - pid_t->Measured;
	
//	if(abs(pid_t->err)<0.2f)
//		pid_t->err = 0.0f;
		//return 0;
	
	pid_t->integral_err += pid_t->err;
	
	p_out = pid_t->Kp*pid_t->err;
	i_out = pid_t->Ki*pid_t->integral_err;
	d_out = pid_t->Kd*(pid_t->err - pid_t->err_last);
	Test_I=i_out;
	//积分限幅
	abs_limit(&i_out, pid_t->IntegralLimit);
	
	pid_t->pwm = (p_out + i_out + d_out);
	
	//输出限幅
	abs_limit(&pid_t->pwm, pid_t->MaxOutput);
	
	pid_t->err_last = pid_t->err;
	
    
	
	return pid_t->pwm;
}

float CLOUD_Position_PID(cloud_positionpid_t *pid_t, float target, float measured)
{

    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

float CLOUD_Incremental_PID(cloud_incrementalpid_t *pid_t, float target, float measured)
{
	float p_out, i_out, d_out;
	pid_t->Target = target;
	pid_t->Measured = measured;
	pid_t->err = pid_t->Target - pid_t->Measured;
	
//	if(abs(pid_t->err)<0.1f)
//		pid_t->err = 0.0f;
		//return 0;
	
	p_out = pid_t->Kp*(pid_t->err - pid_t->err_last);
	i_out = pid_t->Ki*pid_t->err;
	d_out = pid_t->Kd*(pid_t->err - 2.0f*pid_t->err_last + pid_t->err_beforeLast);
	
	//积分限幅
	abs_limit(&i_out, pid_t->IntegralLimit);
	
	pid_t->pwm += (p_out + i_out + d_out);
	
	//输出限幅
	abs_limit(&pid_t->pwm, pid_t->MaxOutput);
	
	pid_t->err_beforeLast = pid_t->err_last;
	pid_t->err_last = pid_t->err;

	return pid_t->pwm;
	
	
}
