#include "calibrate_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "bsp_flash.h"
#include "INS_task.h"

#include "DJI_IMU.h"

//include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
//要写入 flash 的大小：
// 1个head_cail_t + 1个 gimbal_cali_t + 3个imu_cali_t 结构体数据 + 5 个 传感器各种的名字cail_sensor[i].name[3]+ cail_done(已校准标志位)，即5个4个字节
//#define FLASH_WRITE_BUF_LENGHT  (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3  + CALI_LIST_LENGHT * 4)
#define FLASH_WRITE_BUF_LENGHT  (sizeof(imu_cali_t) + CALI_LIST_LENGHT * 4)

/**
  * @brief          read cali data from flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          从flash读取校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void);

/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          往flash写入校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_write(void);

/**
  * @brief          gyro cali function
  * @param[in][out] cali:the point to gyro data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);   //gyro device cali function

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif

static imu_cali_t      gyro_cali;       //gyro cali data

static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];

cali_sensor_t cali_sensor[CALI_LIST_LENGHT]; 

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"GYR"};

//将cail_sensor[i] 中 的校准数组 关联为 对应的传感器的数据
//cali data address
//static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
//        (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
//        (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
//        (uint32_t *)&mag_cali};
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
				(uint32_t *)&gyro_cali};
//sizeof 算出的是 传感器数据结构体的大小 但是为什么要 / 4？ 
//首先目标cail_sensor[i]结构体 中的 flash_len 就是 通过 这个值来确定的 
//而flash的读写都是以32位（即4个字节）
//static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
//    {
//        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
//        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};
static uint8_t cali_sensor_size[CALI_LIST_LENGHT] = {
				sizeof(imu_cali_t) / 4};
//将函数名（即函数指针常量）赋值给 void* 未定义的指针类型数组中
//用 void* 类型，可以让所赋给 数组的函数的可以是不同返回值
//void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};
void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_gyro_hook};

//static uint32_t calibrate_systemTick;

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
//若之前未进行校准，那么就需要执行此校准任务
void calibrate_task(void const *pvParameters)
{
    static uint8_t i = 0;
		static uint8_t key_last_level;		//key上次的电平
	
		uint8_t key_level;				//key的电平信号	
		key_last_level = key_up;	//初始化为未按下
		//初始化key电平
		key_level = HAL_GPIO_ReadPin(key_GPIO_Port,key_Pin);
	
    //获取遥控器结构体的所有数据
    //calibrate_RC = get_remote_ctrl_point_cali();

    while (1)
    {
				//遥控器的不同操作，对应开启不同的传感器的校准
				// 即 让 cali_sensor[i].cali_cmd = 1;
        //RC_cmd_to_calibrate();
				key_level = HAL_GPIO_ReadPin(key_GPIO_Port,key_Pin);
				if((key_level == key_down && key_last_level == key_up) || (IMU_CAL.real_Status == begin_calibration && IMU_CAL.last_Status == stop_calibration))
				{
					cali_sensor[0].cali_cmd = 1;
				}
				//保存上一时刻key的电平
				key_last_level = key_level;
				//保存上一时刻
				IMU_CAL.last_Status = IMU_CAL.real_Status;
				
        for (i = 0; i < CALI_LIST_LENGHT; i++)
        {		
						//若开启校准
            if (cali_sensor[i].cali_cmd)
            {		
								//并且校准的处理钩子函数不为空
                if (cali_sensor[i].cali_hook != NULL)
                {			
										//校准过程持续响
										Buzzer.mode = heaps_times;
										//则通过实时的gyro数据进行计算得到gyro_offset实际用于计算的Angle的偏移值，
										//并将此漂移值赋给cail_sensor[i].buf中校准数据数组，可以用于写入flash中
                    if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
                    {		
												//确定传感器的名字 和 校准状态将其 置为 已经校准
                        //done
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];
                        //set 0x55
                        cali_sensor[i].cali_done = CALIED_FLAG;
												//并将开启校准的标准位清0 
                        cali_sensor[i].cali_cmd = 0;
												//并将校准完的数据写入flash中供于下次开启可用
                        //write
                        cali_data_write();
												//buzzer
                        //响三声
												Buzzer.mode = Three_times;
                    }
                }
            }
        }

        //蜂鸣器的执行
        Buzzer_Processing();

        osDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

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
void cali_param_init(void)
{
    uint8_t i = 0;
		
		//初始化时 先将cail_sensor结构体 即传感器校准结构体 的一些确定的值初始化
		//将 其中 传感器数据结构体的的大小 以及 数据结构体的指针 以及 各传感器的校准的函数挂钩  
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
    }
		//读取flash中的数据，即之前校准后保存在flash中的数据
    cali_data_read();
		
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {		
			//判断已经校准过了 
        if (cali_sensor[i].cali_done == CALIED_FLAG)
        {
					//判断为校准处理钩子函数不为空
            if (cali_sensor[i].cali_hook != NULL)
            {
                //if has been calibrated, set to init
								//则执行校准钩子函数的已校准过的部分代码，即将之前校准过的在flash中的漂移值 赋给 用于实际角度处理的 漂移值
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
                //buzzer
                //响一声
                Buzzer.mode = One_times;
            }
        }
    }
		//若之前未校准过则不做任何处理
}
/**
  * @brief          read cali data from flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          从flash读取校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
    uint8_t i = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {

        //read the data in flash, 
				//由于他是以32位（即4个字节）进行读取的，所以cali_sensor[i].flash_len的初始化那里通过sizeof()算出来的长度（字节数）需要 /4
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
        //而内存的存储以字节为单位，所以它是读取cali_sensor[i].flash_len长度的32位（即4个字节）的数据，那么实际上读取的字节数：cali_sensor[i].flash_len * 4 所以地址也应该增加这么多
        offset += cali_sensor[i].flash_len * 4;

        //read the name and cali flag,
				//CALI_SENSOR_HEAD_LEGHT 只读取1个32位（即4个字节）的数据
        cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);
        
        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];
        
        offset += CALI_SENSOR_HEAD_LEGHT * 4;

        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
        {
            cali_sensor[i].cali_cmd = 1;
        }
    }
}


/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          往flash写入校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_write(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;


    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        //copy the data of device calibration data
			//将更新完存储在cail_sensor[i].flash_buf中的漂移量赋到写入flash的缓冲区数组中
				memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);
        offset += cali_sensor[i].flash_len * 4;

        //copy the name and "CALI_FLAG" of device
				//将各传感器的名字 和 传感器是否已校准标志位 也赋值到 缓冲区，其中name有个细节，他不是name[i]
				//由于name[3]只占3个字节 而cail_done也需要占1个字节，而后面复制的长度是4个字节
        memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
    }
		
    //erase the page
		//擦除flash页面 1页 1个扇区
    cali_flash_erase(FLASH_USER_ADDR,1);
    //write data
		//将要写入flash的缓冲区数据 按照 缓冲区的长度 写入flash中
		//之所以 +3 是为了防止  FLASH_WRITE_BUF_LENGHT是个奇数 导致/4 得到了长度并不够，+3 就可用让寄数长度变长，可以完全的写入flaah而不会丢失  
    cali_flash_write(FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);
}

/**
  * @brief          gyro cali function
  * @param[in][out] cali:the point to gyro data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
		//已经校准过了
    if (cmd == CALI_FUNC_CMD_INIT)
    {
				//则将原来算出来存储在cail_sensor[i].flash_buf / cail_sensor_buf[i] 即是flash中的漂移值，直接赋给gyro_offset[i]实际用于陀螺仪数据计算的漂移值
				gyro_set_cali(local_cali_t->scale, local_cali_t->offset);
        
        return 0;
    }
		//还未校准过的
    else if (cmd == CALI_FUNC_CMD_ON)
    {
				//校准时间初始化
        static uint16_t count_time = 0;
				//陀螺仪漂移值的计算
        gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
				//再判断一下校准的时间是否大于规定时间，是则开启控制
        if (count_time > GYRO_CALIBRATE_TIME)
        {
            count_time = 0;
            //cali_buzzer_off();
            //gyro_cali_enable_control();
            return 1;
        }
				//不是就不给开启控制
        else
        {
            //gyro_cali_disable_control(); //disable the remote control to make robot no move
            //imu_start_buzzer();
            
            return 0;
        }
    }

    return 0;
}
