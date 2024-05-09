#include "WS2812.h"
#include "tim.h"
#include "shot.h"
#include "Robot_control.h"
#include "DR16_Remote.h"
#include "Cloud_control.h"
#include "Status_Update.h"
#include "Chassis_control.h"
#include "RM_Message.h"
/*

     超电                    自旋                          摩擦轮                          自瞄
’X开启超电              按1次' shift，               开启摩擦轮时亮绿灯               开启自瞄后亮绿灯
剩余 > 40% 绿灯         自旋开启时亮绿灯                                              若识别到了亮蓝灯
剩余 < 40% 红灯                                                                       若识别不到仍是绿

*/


#define ONE_PULSE        (149)                           //1 码
#define ZERO_PULSE       (74)                           //0 码
#define RESET_PULSE      (80)                           //80 ，复位信号
#define LED_NUMS         (11)                            //led 数量
#define LED_DATA_LEN     (24)                           //led 长度，单个需要24bits
#define WS2812_DATA_LEN  (LED_NUMS*LED_DATA_LEN)        //ws2812灯条需要的数组长度

uint16_t static RGB_buffur[RESET_PULSE + WS2812_DATA_LEN] = { 0 };
uint8_t WS2812_STATUS[4] = {2, 3, 4, 5};
uint16_t Maga_Open = 0;
void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num)
{
    //
    uint16_t* p = (RGB_buffur + RESET_PULSE) + (num * LED_DATA_LEN);
    
    for (uint16_t i = 0;i < 8;i++)
    {
        //
        p[i]      = (G << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 8]  = (R << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 16] = (B << i) & (0x80)?ONE_PULSE:ZERO_PULSE;

    }
}
/*ws2812 初始化*/
void ws2812_init(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0x00, 0x00, 0x00, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data)); 
}

//亮哪盏

void ws2812_stauts(uint8_t status,uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	
	if(status == LED_ON)
	{
		ws2812_set_RGB(0x00, 0xF, 0x00, led_nums); //green
	}
	else if(status == LED_OFF)
	{
		ws2812_set_RGB(0xF, 0x00, 0x00, led_nums); //0
	}
	
	HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}

//亮哪盏

void Supcap_stauts(uint8_t color,uint8_t led_nums )
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	
	if(color == 0)
	{
		ws2812_set_RGB(0xF, 0x00, 0x00, led_nums); //红色
	}
	else if(color == 1)
	{
		ws2812_set_RGB(0x00, 0x00, 0xF, led_nums); //深蓝
	}
	else if(color == 2)
	{
		ws2812_set_RGB(0x00, 0xF, 0xF, led_nums); //浅蓝
	}
	else if(color == 3)
	{
		ws2812_set_RGB(0x00, 0xF, 0x00, led_nums); //绿色
	}
	
	HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}

/*全亮蓝灯*/
void ws2812_blue(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0x00, 0x00, 0x22, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}

//亮一盏蓝灯
void ws2812_blue_single(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;

	ws2812_set_RGB(0x00, 0x00, 0x22, led_nums);

	HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}

/*全亮红灯*/
void ws2812_red(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0x22, 0x00, 0x00, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}

//亮一盏红灯
void ws2812_red_single(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;

	ws2812_set_RGB(0x22, 0x00, 0x00, led_nums);

	HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}


/*全亮绿灯*/
void ws2812_green(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0x00, 0x22, 0x00, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}

void ws2812_example(void)
{		
	  ws2812_set_RGB(0x00, 0x00, 0x22, 0);
    ws2812_set_RGB(0x00, 0x00, 0x22, 1);
    ws2812_set_RGB(0x00, 0x00, 0x22, 2);
    ws2812_set_RGB(0x00, 0x00, 0x22, 3);
    ws2812_set_RGB(0x00, 0x00, 0x22, 6);
    ws2812_set_RGB(0x00, 0x00, 0x22, 7);
		ws2812_set_RGB(0x00, 0x00, 0x22, 8);
    ws2812_set_RGB(0x00, 0x00, 0x22, 10);
    HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(344)); //344 = 80 + 24 * LED_NUMS(11)

    HAL_Delay(500);
}


void RGB_Control(void)
{
	if(Robot.Device_FrictMode == FrictWorkMode_HighSpeed)
	{
		if (Shoot_Down_Flag == 0)
		{
			ws2812_stauts(LED_ON,1);
		}
		else if(Shoot_Down_Flag == 1)
		{
			Supcap_stauts(2,1);
		}
		else
		{
			Supcap_stauts(1,1);
		}
	}
	else
	{
		ws2812_stauts(LED_OFF,1);
	}
	
	
	if(DR16_Export_Data.Robot_TargetValue.Omega_Value > 0)
	{
		ws2812_stauts(LED_ON,2);
	}
	else
	{
		ws2812_stauts(LED_OFF,2);
	}
	
	
	if(Open_Left == 1)
	{
		if(Maga_Open < 200)
		{
			ws2812_stauts(LED_ON,3);
			Maga_Open ++;
		}
		else if(Maga_Open >= 200 && Maga_Open <= 400)
		{
			ws2812_stauts(LED_OFF,3);
			Maga_Open ++;
			if(Maga_Open == 400)
			{
				Maga_Open = 0;
			}
		}
	}
	else
	{
		ws2812_stauts(LED_OFF,3);
	}
	
	if(Robot.ChassisWorkMode == ChassisWorkMode_Square)
	{
		ws2812_stauts(LED_ON,4);
	}
	else
	{
		ws2812_stauts(LED_OFF,4);
	}
	
	
	if(Sup_Cap == 1)
	{
		ws2812_stauts(LED_ON,5);
	}
	else
	{
		ws2812_stauts(LED_OFF,5);
	}
	
	switch(RM_ME[0].SupCap_Mix)
	{
		case 0 :
		Supcap_stauts(0,6);
		break;
		
		case 1:
		Supcap_stauts(1,6);
		break;
		
		case 2:
		Supcap_stauts(2,6);
		break;
		
		case 3:
		Supcap_stauts(3,6);
		break;
	
	}
	
}
