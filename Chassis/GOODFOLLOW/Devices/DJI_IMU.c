/**
 * @file DJI_IMU.c
 * @author Miraggio (w1159904119@gmail)
 * @brief A板板载陀螺仪配置
 * @version 0.1
 * @date 2021-04-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "DJI_IMU.h"
#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "mpu6500reg.h"
#include "mpu6500_reg.h"
#include "ist8310_reg.h"
#include "spi.h"

void DJI_IMU_Init(void);
void mpu_get_data(void);
void IMU_Cali_Slove(float gyro[3], float accel[3], float mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310);
void IMU_data_read_over(mpu6500_real_data_t *mpu6500_real_data, ist8310_real_data_t *ist8310_real_data);
DJI_IMUFUN_t DJI_IMUFUN = DJI_IMUFUNGroundInit;
#undef DJI_IMUFUNGroundInit

#define IMU_BOARD_INSTALL_SPIN_MATRIX \
    {0.0f, 1.0f, 0.0f},               \
        {-1.0f, 0.0f, 0.0f},          \
    {                                 \
        0.0f, 0.0f, 1.0f              \
    }
#define MPU_HSPI hspi5
#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)
#define WOM_THR_Set 0x0F
uint8_t IMU_Init_Condition = 0 ; //陀螺仪数据准备
		
static float Gyro_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX};  //陀螺仪校准线性度
static float Accel_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //加速度校准线性度
static float Accel_Offset[3] = {0.0f, 0.0f, 0.0f};                       //加速度零漂
static float Mag_Scale_Factor[3][3] = {{1.0f, 0.0f, 0.0f},
                                       {0.0f, 1.0f, 0.0f},
                                       {0.0f, 0.0f, 1.0f}}; //磁力计校准线性度
static float Mag_Offset[3] = {0.0f, 0.0f, 0.0f};            //磁力计零漂
static uint8_t tx, rx;
static uint8_t tx_buff[14] = {0xff};
//uint8_t               mpu_buff[14];                          /* buffer to save imu raw data */
uint8_t mpu_buff[15]; /* buffer to save imu raw data */
uint8_t ist_buff[6];  /* buffer to save IST8310 raw data */
mpu_data_t mpu_data;

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval 
  * @usage  call in ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() function
  */
static uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
    MPU_NSS_LOW;
    tx = regAddr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  read a byte of data from specified register
  * @param  reg: the address of register to be read
  * @retval 
  * @usage  call in ist_reg_read_by_mpu(),         
  *                 mpu_device_init() function
  */
static uint8_t mpu_read_byte(uint8_t const reg)
{
    MPU_NSS_LOW;
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  write a byte of data to specified register
  * @param  reg:  the address of register to be written
  *         data: data to be written
  * @retval 
  * @usage  call in ist_reg_write_by_mpu(),         
  *                 ist_reg_read_by_mpu(), 
  *                 mpu_master_i2c_auto_read_config(), 
  *                 ist8310_init(), 
  *                 mpu_set_gyro_fsr(),             
  *                 mpu_set_accel_fsr(), 
  *                 mpu_device_init() function
  */
static uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
	* @brief  write IST8310 register through MPU6500's I2C master
  * @param  addr: the address to be written of IST8310's register
  *         data: data to be written
	* @retval   
  * @usage  call in ist8310_init() function
	*/
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* turn off slave 1 at first */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    MPU_DELAY(2);
    /* turn on slave 1 with one byte transmitting */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    MPU_DELAY(10);
}

/**
	* @brief  write IST8310 register through MPU6500's I2C Master
	* @param  addr: the address to be read of IST8310's register
	* @retval 
  * @usage  call in ist8310_init() function
	*/
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
    return retval;
}

/**
	* @brief  get the offset data of MPU6500
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
static void mpu_offset_call(void)
{
    int i;
    for (i = 0; i < 1000; i++)
    {
        mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

        mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
        mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
        mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];

        mpu_data.gx_offset += mpu_buff[8] << 8 | mpu_buff[9];
        mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
        mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

        MPU_DELAY(5);
    }
    mpu_data.ax_offset = mpu_data.ax_offset / 1000;
    mpu_data.ay_offset = mpu_data.ay_offset / 1000;
    mpu_data.az_offset = mpu_data.az_offset / 1000;
    mpu_data.gx_offset = mpu_data.gx_offset / 1000;
    mpu_data.gy_offset = mpu_data.gy_offset / 1000;
    mpu_data.gz_offset = mpu_data.gz_offset / 1000;
}

/**
	* @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
static uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
    return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}

/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
static uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
    return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3);
}

/**
	* @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
* @param    device_address: slave device address, Address[6:0]
	* @retval   void
	* @note     
	*/
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
	   * configure the device address of the IST8310 
     * use slave1, auto transmit single measure mode 
	   */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    /* use slave0,auto read data */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    /* every eight mpu6500 internal samples one i2c master read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
    /* enable slave 0 and 1 access delay */
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
    /* enable slave 1 auto transmit */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    MPU_DELAY(6);
    /* enable slave 0 with data_num bytes reading */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}

/**
	* @brief  Initializes the IST8310 device
	* @param  
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
static uint8_t ist8310_init()
{
    /* enable iic master mode */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    MPU_DELAY(10);
    /* enable iic 400khz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d);
    MPU_DELAY(10);

    /* turn on slave 1 for ist write and slave 4 to ist read */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    MPU_DELAY(10);

    /* IST8310_R_CONFB 0x01 = device rst */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

    /* soft reset */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);

    /* config as ready mode to access register */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    MPU_DELAY(10);

    /* normal state, no int */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    MPU_DELAY(10);

    /* config low noise mode, x,y,z axis 16 time 1 avg */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    MPU_DELAY(10);

    /* Set/Reset pulse duration setup,normal mode */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    MPU_DELAY(10);

    /* turn off slave1 & slave 4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);

    /* configure and turn on slave 0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    MPU_DELAY(100);
    return 0;
}

/**
	* @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
uint8_t id;
static uint8_t mpu_device_init(void)
{
    MPU_DELAY(100);

    id = mpu_read_byte(MPU6500_WHO_AM_I);
    uint8_t i = 0;

    uint8_t MPU6500_Init_Data[14][2] = {
        {MPU6500_PWR_MGMT_1, (((~MPU_DEVICE_RESET) & (~MPU_SLEEP) & (~MPU_CYCLE) & (~MPU_GYRO_STANDBY) & (~MPU_TEMP_DISABLE)) & (MPU_CLKSEL_INTERNAL))}, /* Reset Device */
        //{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */
        {MPU6500_PWR_MGMT_2, (((~MPU_DISABLE_XA) & (~MPU_DISABLE_YA) & (~MPU_DISABLE_ZA) & (~MPU_DISABLE_XG) & (~MPU_DISABLE_YG) & (~MPU_DISABLE_ZG)) & (MPU_LP_WAKE_1_25_HZ))}, /* Enable Acc & Gyro */
        {MPU_SMPLRT_DIV, MPU_SMPLRT_DIV_1},
        {MPU6500_CONFIG, ((~MPU_FIFO_MODE_OFF_REPLACE_OLD_DATA) & (MPU_EXT_SYNC_DISABLE | MPU_DLPF_CFG_2_SET))},                                                                        /* LPF 41Hz */
        {MPU6500_GYRO_CONFIG, (((~MPU_XG_SELF_TEST_SET) & (~MPU_YG_SELF_TEST_SET) & (~MPU_ZG_SELF_TEST_SET)) & ((0x2 << MPU_GYRO_FS_SEL_SHIFTS)))},                                     /* +-2000dps */
        {MPU6500_ACCEL_CONFIG, (((~MPU_XA_SELF_TEST_SET) & (~MPU_YA_SELF_TEST_SET) & (~MPU_ZA_SELF_TEST_SET)) & ((0x0 << MPU_ACCEL_FS_SEL_SHIFTS)))},                                   /* +-8G */
        {MPU6500_ACCEL_CONFIG_2, (MPU_ACCEL_FCHOICE_B_0_SET | MPU_A_DLPL_CFG_0_SET)},                                                                                                   /* enable LowPassFilter  Set Acc LPF */
        {MPU6500_USER_CTRL, (((~MPU_DMP_EN) & (~MPU_FIFO_MODE_EN) & (~MPU_DMP_RST) & (~MPU_FIFO_RST) & (~MPU_I2C_MST_RST) & (~MPU_SIG_COND_RST)) & (MPU_I2C_MST_EN | MPU_I2C_IF_DIS))}, /* Enable AUX */
        {MPU_INT_ENABLE, (((~MPU_FIFO_OVERFLOW_EN) & (~MPU_FSYNC_INT_EN)) & (MPU_RAW_RDY_EN))},
        {MPU_WOM_THR, WOM_THR_Set},
        {MPU_I2C_MST_CTRL, (((~MPU_MULT_MST_EN) & (~MPU_SLV_3_FIFO_EN) & (~MPU_I2C_MST_P_NSR)) & (MPU_WAIT_FOR_ES_EN | MPU_I2C_MST_CLK_400_KHZ))},
        {MPU_INTBP_CFG, ((~MPU_INTBP_ACTL) & (~MPU_INTBP_OPEN) & (~MPU_LATCH_INT_EN) & (~MPU_INT_ANYRD_2CLEAR) & (~MPU_ACTL_FSYNC) & (~MPU_FSYNC_INT_MODE_EN)) & (MPU_BYPASS_EN)},
        {MPU_I2C_MST_DELAY_CTRL, MPU_I2C_SLV0_DLY_EN},
        {MPU_MOT_DETECT_CTRL, MPU_ACCEL_INTEL_EN | MPU_ACCEL_INTEL_MODE_COMPARE},
    };
    for (i = 0; i < 14; i++)
    {
        mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
        MPU_DELAY(1);
    }

    mpu_set_gyro_fsr(3);
    mpu_set_accel_fsr(2);

    ist8310_init();
    mpu_offset_call();
		
		//初始化完成标志位
		IMU_Init_Condition = 1;
    return 0;
}

void DJI_IMU_Init(void)
{
    //初始化mpu6500
    mpu_device_init();
}

void IMU_Cali_Slove(float gyro[3], float accel[3], float mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = mpu6500->gyro[0] * Gyro_Scale_Factor[i][0] + mpu6500->gyro[1] * Gyro_Scale_Factor[i][1] + mpu6500->gyro[2] * Gyro_Scale_Factor[i][2];
        accel[i] = mpu6500->accel[0] * Accel_Scale_Factor[i][0] + mpu6500->accel[1] * Accel_Scale_Factor[i][1] + mpu6500->accel[2] * Accel_Scale_Factor[i][2] + Accel_Offset[i];
        mag[i] = ist8310->mag[0] * Mag_Scale_Factor[i][0] + ist8310->mag[1] * Mag_Scale_Factor[i][1] + ist8310->mag[2] * Mag_Scale_Factor[i][2] + Mag_Offset[i];
    }
}

/**
	* @brief  get the data of IST8310
  * @param  buff: the buffer to save the data of IST8310
	* @retval 
  * @usage  call in mpu_get_data() function
	*/
static void ist8310_get_data(uint8_t *buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6);
}

/**
	* @brief  get the data of imu
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_get_data(void)
{
    // mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    //    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    //    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    //    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
    //    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

    //    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    //    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    //    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

    mpu_read_bytes(MPU6500_INT_STATUS, mpu_buff, 15);

    mpu_data.ax = mpu_buff[1] << 8 | mpu_buff[2];
    mpu_data.ay = mpu_buff[3] << 8 | mpu_buff[4];
    mpu_data.az = mpu_buff[5] << 8 | mpu_buff[6];
    mpu_data.temp = mpu_buff[7] << 8 | mpu_buff[8];

    mpu_data.gx = ((mpu_buff[9] << 8 | mpu_buff[10]) - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[11] << 8 | mpu_buff[12]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[13] << 8 | mpu_buff[14]) - mpu_data.gz_offset);

    ist8310_get_data(ist_buff);
    memcpy(&mpu_data.mx, ist_buff, 6);

}

void IMU_data_read_over(mpu6500_real_data_t *mpu6500_real_data, ist8310_real_data_t *ist8310_real_data)
{

    if ((mpu_buff[0]) & MPU_INT_WOM_INT)
    {
        mpu6500_real_data->status |= (uint8_t)(1 << MPU_MOT_BIT);
    }
    if (mpu6500_real_data->status & (1 << MPU_MOT_BIT))
    {
        static uint8_t motion_time = 0;
        motion_time++;
        if (motion_time > 10)
        {
            motion_time = 0;
            mpu6500_real_data->status &= ~(1 << MPU_MOT_BIT);
        }
    }

    if ((mpu_buff[0]) & MPU_RAW_RDY_INT)
    {

        mpu6500_real_data->status |= (1 << MPU_DATA_READY_BIT);

        mpu6500_real_data->accel[0] = mpu_data.ax * 0.00059814453125f;
        mpu6500_real_data->accel[1] = mpu_data.ay * 0.00059814453125f;
        mpu6500_real_data->accel[2] = mpu_data.az * 0.00059814453125f;

        mpu6500_real_data->temp = 21 + mpu_data.temp / 333.87f;

        mpu6500_real_data->gyro[0] = mpu_data.gx * 0.0005326322180158476492076f;
        mpu6500_real_data->gyro[1] = mpu_data.gy * 0.0005326322180158476492076f;
        mpu6500_real_data->gyro[2] = mpu_data.gz * 0.0005326322180158476492076f;

        mpu6500_real_data->original_gyro[0] = mpu_data.gx;
        mpu6500_real_data->original_gyro[1] = mpu_data.gy;
        mpu6500_real_data->original_gyro[2] = mpu_data.gz;
    }

    ist8310_real_data->mag[0] = mpu_data.mx * 0.3f;
    ist8310_real_data->mag[1] = mpu_data.my * 0.3f;
    ist8310_real_data->mag[2] = mpu_data.mz * 0.3f;
}
