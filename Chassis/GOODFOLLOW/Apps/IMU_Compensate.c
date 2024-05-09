/**
 * @file IMU_Compensate.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 陀螺仪温漂补偿
 * @version 0.1
 * @date 2021-04-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "IMU_Compensate.h"
#include "BSP_ADC.h"
#include "DJI_IMU.h"
#include "mpu6500reg.h"
#include "AHRS.h"
#include "PID.h"

int8_t Calibrate_Temperate = 0;
cloud_positionpid_t imuTempPid = imuTempPidInit;
#undef imuTempPidInit

void Preserve_temp(float Real_temp);
void IMU_GetData_Compensate(void);
IMU_CompensateFUN_t IMU_CompensateFUN = IMU_CompensateFUNInit;
#undef IMU_CompensateFUNInit

static mpu6500_real_data_t mpu6500_real_data; //转换成国际单位的MPU6500数据
static ist8310_real_data_t ist8310_real_data; //转换成国际单位的IST8310数据
mpu6500_Exportdata_t mpu6500_Exportdata;      //对外输出陀螺仪值
static float gyro_cali_offset[3] = {0.0f, 0.0f, 0.0f};
static float Gyro_Offset[3] = {0.0f, 0.0f, 0.0f}; //陀螺仪零漂
static float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static float INS_accel[3] = {0.0f, 0.0f, 0.0f};
static float INS_mag[3] = {0.0f, 0.0f, 0.0f};

static float INS_Angle[3] = {0.0f, 0.0f, 0.0f};      //欧拉角 单位 rad
static float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //四元数

static const float TimingTime = 0.002f; //任务运行的时间 单位 s				//-------------------需要改成自己的任务时间
uint8_t first_temperate = 0;

static float get_control_temperate(void)
{
    return mpu6500_real_data.temp;
}

/**
 * @brief 温度补偿
 * 
 * @param temp 
 * @return  
 */
static void IMU_temp_Control(float temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        imuTempPid.CLOUD_Position_PID(&imuTempPid, temp, get_control_temperate());
        if (imuTempPid.pwm < 0.0f)
        {
            imuTempPid.pwm = 0.0f;
        }
        tempPWM = (uint16_t)imuTempPid.pwm;
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        if (temp > get_control_temperate())
        {
            temp_constant_time++;
            if (temp_constant_time > 20)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                first_temperate = 1;
                imuTempPid.i_out = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
        else
        {
            first_temperate = 1;
        }
				
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, MPU6500_TEMP_PWM_MAX - 1);
    }
}

static void gyro_offset(float gyro_offset[3], float gyro[3], uint8_t imu_status, uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

    if (imu_status & (1 << MPU_MOT_BIT))
    {
        (*offset_time_count) = 0;
        return;
    }

    if (imu_status & (1 << MPU_DATA_READY_BIT))
    {
        gyro_offset[0] = gyro_offset[0] - GYRO_OFFSET_KP * gyro[0];
        gyro_offset[1] = gyro_offset[1] - GYRO_OFFSET_KP * gyro[1];
        gyro_offset[2] = gyro_offset[2] - GYRO_OFFSET_KP * gyro[2];
        (*offset_time_count)++;
    }
}

/**
 * @brief ADC温度获取
 * 
 * @param Real_temp 
 * @return  
 */
void Preserve_temp(float Real_temp)
{
    Calibrate_Temperate = Real_temp;
    if (Calibrate_Temperate > (int8_t)GYRO_CONST_MAX_TEMP)
    {
        Calibrate_Temperate = (int8_t)GYRO_CONST_MAX_TEMP;
    }
}

/**
 * @brief 更新陀螺仪数据
 * 
 * @return  
 */
static void Update_IMU_Data(void)
{
    DJI_IMUFUN.mpu_get_data();

    //将获取到的值传给单独的结构体
    DJI_IMUFUN.IMU_data_read_over(&mpu6500_real_data, &ist8310_real_data);

    //减去零漂以及旋转坐标系
    DJI_IMUFUN.IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &mpu6500_real_data, &ist8310_real_data);

    //加速度计低通滤波
    static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
    static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
    static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
    static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

    //判断是否第一次进入，如果第一次则初始化四元数，之后更新四元数计算角度单位rad
    static uint8_t updata_count = 0;

    if (mpu6500_real_data.status & 1 << MPU_DATA_READY_BIT)
    {

        if (updata_count == 0)
        {
            //初始化四元数
            AHRS_init(INS_quat, INS_accel, INS_mag);
            get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

            accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
            accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
            accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
            updata_count++;
        }
        else
        {
            //加速度计低通滤波
            accel_fliter_1[0] = accel_fliter_2[0];
            accel_fliter_2[0] = accel_fliter_3[0];

            accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

            accel_fliter_1[1] = accel_fliter_2[1];
            accel_fliter_2[1] = accel_fliter_3[1];

            accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

            accel_fliter_1[2] = accel_fliter_2[2];
            accel_fliter_2[2] = accel_fliter_3[2];

            accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

            //更新四元数
            AHRS_update(INS_quat, TimingTime, INS_gyro, accel_fliter_3, INS_mag);
            get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

            //陀螺仪开机校准
            {
                static uint16_t start_gyro_cali_time = 0;
                if (start_gyro_cali_time == 0)
                {
                    Gyro_Offset[0] = gyro_cali_offset[0];
                    Gyro_Offset[1] = gyro_cali_offset[1];
                    Gyro_Offset[2] = gyro_cali_offset[2];
                    start_gyro_cali_time++;
                }
                else if (start_gyro_cali_time < GYRO_OFFSET_START_TIME)
                {
                    // IMUWarnBuzzerOn();
                    if (1)
                    {
                        // 当进入gyro_offset函数，如果无运动start_gyro_cali_time++，如果有运动 start_gyro_cali_time = 0
                        gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, &start_gyro_cali_time);
                    }
                }
                else if (start_gyro_cali_time == GYRO_OFFSET_START_TIME)
                {
                    start_gyro_cali_time++;
                }
            } //陀螺仪开机校准   code end

            //将弧度值转换为360角度值
            //姿态角 将rad 变成 度，除这里的姿态角的单位为度，其他地方的姿态角，单位均为弧度
            mpu6500_Exportdata.temp = mpu6500_real_data.temp;

            for (int i = 0; i < 3; i++)
            {
                mpu6500_Exportdata.angle_degree[i] = INS_Angle[i] * 57.3f + 180.0f;
                if (mpu6500_Exportdata.angle_degree[i] - mpu6500_Exportdata.last_angle_degree[i] < -300)
                { //经过跳变边沿。
                    mpu6500_Exportdata.turnCount[i]++;
                }
                if (mpu6500_Exportdata.last_angle_degree[i] - mpu6500_Exportdata.angle_degree[i] < -300)
                {
                    mpu6500_Exportdata.turnCount[i]--;
                }
                mpu6500_Exportdata.total[i] = mpu6500_Exportdata.angle_degree[i] + (360 * mpu6500_Exportdata.turnCount[i]);

                mpu6500_Exportdata.last_angle_degree[i] = mpu6500_Exportdata.angle_degree[i];

                mpu6500_Exportdata.Gyro[i] = mpu6500_real_data.original_gyro[i];
            }

        } //update count if   code end
    }     //mpu6500 status  if end
}

/**
 * @brief 温补加上获取数据
 * 
 * @return  
 */
void IMU_GetData_Compensate(void)
{
    IMU_temp_Control(Calibrate_Temperate);
    Update_IMU_Data();
}
