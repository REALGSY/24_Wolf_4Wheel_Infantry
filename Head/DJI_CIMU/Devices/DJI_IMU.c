#include "DJI_IMU.h"
#include "INS_task.h"
#include "arm_math.h"
//由于can的数据帧最大的字节数为8
//欧拉角的一个轴的数据就是一个float类型变量，是4个字节
//那也就最多只能发两个轴
Euler_Send_u Euler_Send;
Gyro_Send_u Gyro_Send;
IMU_Rec_Data_t DJI_C_IMU;
//接收变量
IMU_CAL_t IMU_CAL;

#if send_way == 0
/*第一种发送方式：联合体*/
//欧拉角
void Euler_Send_Fun(Euler_Send_u Euler_Send)
{
	
	DJI_C_IMU.yaw = INS_angle[0] * (180/PI) + 180.0f;
	DJI_C_IMU.roll = INS_angle[1] * (180/PI) + 180.0f;
	DJI_C_IMU.pitch = INS_angle[2] * (180/PI) + 180.0f;
	DJI_C_IMU.Gyro_z = INS_gyro[2] * (180/PI);
	DJI_C_IMU.Gyro_y = INS_gyro[0] * (180/PI);

  if (DJI_C_IMU.yaw - DJI_C_IMU.last_yaw < -300.0f)
    {
        DJI_C_IMU.turnCounts++;
    }
    if (DJI_C_IMU.last_yaw - DJI_C_IMU.yaw < -300.0f)
    {
        DJI_C_IMU.turnCounts--;
    }
    DJI_C_IMU.total_yaw = DJI_C_IMU.yaw + DJI_C_IMU.turnCounts * 360.0f;

    DJI_C_IMU.last_yaw = DJI_C_IMU.yaw;
		
    //pitch轴的过零处理
    if (DJI_C_IMU.pitch - DJI_C_IMU.last_pitch < -300.0f)
    {
        DJI_C_IMU.pitch_turnCounts++;
    }
    if (DJI_C_IMU.last_pitch - DJI_C_IMU.pitch < -300.0f)
    {
        DJI_C_IMU.pitch_turnCounts--;
    }
    DJI_C_IMU.total_pitch = DJI_C_IMU.pitch + DJI_C_IMU.pitch_turnCounts * 360.0f;

    DJI_C_IMU.last_pitch = DJI_C_IMU.pitch;
}
//角速度
void Gyro_Send_Fun(Gyro_Send_u Gyro_Send)
{
	//8个1字节的缓存局部变量
	uint8_t data[8];
	
	//Yaw轴angle
	data[0] = Gyro_Send.Gyro_zy[0];
	data[1] = Gyro_Send.Gyro_zy[1];
	data[2] = Gyro_Send.Gyro_zy[2];
	data[3] = Gyro_Send.Gyro_zy[3];
	
	//Pitch轴angle
	data[4] = Gyro_Send.Gyro_zy[4];
	data[5] = Gyro_Send.Gyro_zy[5];
	data[6] = Gyro_Send.Gyro_zy[6];
	data[7] = Gyro_Send.Gyro_zy[7];
	
	//以CAN通讯发送
	//CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Gyro_SENDID,data);
}

//接收解包
void IMU_Cal_Status_Reivece(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	if(CAN_Rx_Structure.CAN_RxMessage.StdId != IMU_CAL_REIID)
	{
		return;
	}
	//获取陀螺仪当前的校准状态
	IMU_CAL.real_Status = CAN_Rx_Structure.CAN_RxMessageData[0];
}
#endif

#if send_way == 1
/*第二种发送形式：指针*/
//欧拉角
void Euler_Send_Fun(float Yaw,float Pitch)
{
	//所有指针都占4个字节
	unsigned char* p[2];
	uint8_t data[8];
	
	p[0] = (unsigned char*)&Yaw;
	p[1] = (unsigned char*)&Pitch;
	
	//Yaw轴angle
	data[0] = *p[0];
	data[1] = *(p[0] + 1);
	data[2] = *(p[0] + 2);
	data[3] = *(p[0] + 3);
	
	//Pitch轴angle
	data[4] = *p[1];
	data[5] = *(p[1] + 1);
	data[6] = *(p[1] + 2);
	data[7] = *(p[1] + 3);
	
	//以CAN通讯发送
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Angle_SENDID,data);
}
//角速度
void Gyro_Send_Fun(float Gyro_z,float Gyro_y)
{
	//所有指针都占4个字节
	unsigned char* p[2];
	uint8_t data[8];
	
	p[0] = (unsigned char*)&Gyro_z;
	p[1] = (unsigned char*)&Gyro_y;
	
	//Yaw轴angle
	data[0] = *p[0];
	data[1] = *(p[0] + 1);
	data[2] = *(p[0] + 2);
	data[3] = *(p[0] + 3);
	
	//Pitch轴angle
	data[4] = *p[1];
	data[5] = *(p[1] + 1);
	data[6] = *(p[1] + 2);
	data[7] = *(p[1] + 3);
	
	//以CAN通讯发送
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Gyro_SENDID,data);
}

#endif
