#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H

#include "struct_typedef.h"
#include "main.h"
#include "bsp_buzzer.h"

#define key_down 0
#define key_up 1

//get stm32 chip temperature, to calc imu control temperature.获取stm32片内温度，计算imu的控制温度
#define cali_get_mcu_temperature()  get_temprate()    

#define cali_flash_read(address, buf, len)  flash_read((address), (buf), (len))                     //flash read function, flash 读取函数
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len))     //flash write function,flash 写入函数
#define cali_flash_erase(address, page_num) flash_erase_address((address), (page_num))              //flash erase function,flash擦除函数

// calc the zero drift function of gyro, 计算陀螺仪零漂
#define gyro_cali_fun(cali_scale, cali_offset, time_count)  INS_cali_gyro((cali_scale), (cali_offset), (time_count))
//set the zero drift to the INS task, 设置在INS task内的陀螺仪零漂
#define gyro_set_cali(cali_scale, cali_offset)              INS_set_cali_gyro((cali_scale), (cali_offset))

#define FLASH_USER_ADDR         ADDR_FLASH_SECTOR_9 //write flash page 9,保存的flash页地址

#define GYRO_CONST_MAX_TEMP     45.0f               //max control temperature of gyro,最大陀螺仪控制温度

#define CALI_FUNC_CMD_ON        1                   //need calibrate,设置校准
#define CALI_FUNC_CMD_INIT      0                   //has been calibrated, set value to init.已经校准过，设置校准值

#define CALIBRATE_CONTROL_TIME  1                   //osDelay time,  means 1ms.1ms 系统延时

#define CALI_SENSOR_HEAD_LEGHT  1

#define SELF_ID                 0                   //ID 
#define FIRMWARE_VERSION        12345               //handware version.
#define CALIED_FLAG             0x55                // means it has been calibrated

#define GYRO_CALIBRATE_TIME         20000   //gyro calibrate time,陀螺仪校准时间

//cali device name
typedef enum
{
    CALI_GYRO = 0,
    //add more...
    CALI_LIST_LENGHT,
} cali_id_e;

typedef __packed struct
{
    uint8_t name[3];                                    //device name
    uint8_t cali_done;                                  //0x55 means has been calibrated
    uint8_t flash_len : 7;                              //buf lenght				//冒号：数字 是表示我定义了个 8个位的 flash_len 变量 但是我只用到 它的 7个位
    uint8_t cali_cmd : 1;                               //1 means to run cali hook function,
    uint32_t *flash_buf;                                //link to device calibration data
    bool_t (*cali_hook)(uint32_t *point, bool_t cmd);   //cali function
	
} cali_sensor_t;
extern cali_sensor_t cali_sensor[CALI_LIST_LENGHT];

//gyro, accel, mag device
typedef struct
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;

/**
  * @brief          use remote control to begin a calibrate,such as gyro, gimbal, chassis
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
extern void cali_param_init(void);

/**
  * @brief          get imu control temperature, unit ℃
  * @param[in]      none
  * @retval         imu control temperature
  */
/**
  * @brief          获取imu控制温度, 单位℃
  * @param[in]      none
  * @retval         imu控制温度
  */
extern int8_t get_control_temperature(void);
/**
  * @brief          calibrate task, created by main function
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          校准任务，由main函数创建
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void calibrate_task(void const *pvParameters);

#endif
