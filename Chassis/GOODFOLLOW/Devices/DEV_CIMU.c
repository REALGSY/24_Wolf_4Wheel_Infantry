#include "DEV_CIMU.h"

//若有多个陀螺仪 需要不同的ID（防ID冲突），就把其变为数组

//接收数据联合体
DJI_C_Euler_u DJI_C_Euler_Receive /* [] */;
DJI_C_Gyro_u DJI_C_Gyro_Receive; /* [] */
//数据处理结构体
DJI_C_IMU_t DJI_C_IMU /*DJI_C_IMU[]*/;
//帧率结构体
////WorldTime_RxTypedef gyro_FPS;
//uint32_t Gyro_FPS;
//WorldTime_RxTypedef euler_FPS;
//uint32_t Euler_FPS;

void Updata_Hand_Euler_Gyro_Data(void);
void DJI_C_Euler_getInfo(Can_Export_Data_t CAN_Rx_Structure);
void DJI_C_Gyro_getInfo(Can_Export_Data_t CAN_Rx_Structure);
void Check_DJI_C_IMU(void);
void CIMU_Reset(DJI_C_IMU_t *DJI_C_IMU);

DJI_C_IMUFUN_t DJI_C_IMUFUN = DJI_C_IMUFUNGroundInit;
#undef DJI_C_IMUFUNGroundInit

//处理更新数据
void Updata_Hand_Euler_Gyro_Data(void)
{
    //角度
    DJI_C_IMU.yaw = ((float)DJI_C_Euler_Receive.yaw * Angle_turn_Radian + 180.0f);    //将弧度转为度
    DJI_C_IMU.pitch = (float)DJI_C_Euler_Receive.pitch * Angle_turn_Radian + 180.0f; //(-90° ~ 90°)
    //角速度
    DJI_C_IMU.Gyro_z = DJI_C_Gyro_Receive.Gyro_z * Angle_turn_Radian;
    DJI_C_IMU.Gyro_y = DJI_C_Gyro_Receive.Gyro_y * Angle_turn_Radian;

    //yaw轴的过零处理
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
		
		
    //yaw轴的过零处理
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

//解包DJI_C_IMU的Euler
void DJI_C_Euler_getInfo(Can_Export_Data_t CAN_Rx_Structure)
{
    if (CAN_Rx_Structure.CAN_RxHeader.StdId != DJI_C_Angle)
    {
        return;
    }
    for (int i = 0; i < 8; i++)
    {
        DJI_C_Euler_Receive.BraodData[i] = CAN_Rx_Structure.CANx_Export_RxMessage[i];
    }

    DJI_C_Euler_Receive.InfoUpdateFrame++;
    //获取其帧率
    //Get_FPS(&euler_FPS, &Euler_FPS);
}

//解包DJI_C_IMU的Gyro
void DJI_C_Gyro_getInfo(Can_Export_Data_t CAN_Rx_Structure)
{
    if (CAN_Rx_Structure.CAN_RxHeader.StdId != DJI_C_Gyro)
    {
        return;
    }
    for (int i = 0; i < 8; i++)
    {
        DJI_C_Gyro_Receive.BraodData[i] = CAN_Rx_Structure.CANx_Export_RxMessage[i];
    }
    //获取其帧率
   // Get_FPS(&gyro_FPS, &Gyro_FPS);
}

void CIMU_Reset(DJI_C_IMU_t *DJI_C_IMU)
{
    DJI_C_IMU->last_yaw = DJI_C_IMU->Gyro_z;
    DJI_C_IMU->total_yaw = DJI_C_IMU->Gyro_z;
    DJI_C_IMU->turnCounts = 0;
}

void Check_DJI_C_IMU(void)
{
    //C板陀螺仪 ---------------------------
    if (DJI_C_Euler_Receive.InfoUpdateFrame < 1)
    {
        DJI_C_IMU.OffLineFlag = 1;
    }
    else
    {
        DJI_C_IMU.OffLineFlag = 0;
    }
    DJI_C_Euler_Receive.InfoUpdateFrame = 0;
}
