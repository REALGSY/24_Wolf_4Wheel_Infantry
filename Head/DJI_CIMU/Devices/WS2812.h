#ifndef  WS2812_H
#define WS2812_H

#include "stdint.h"
#include "tim.h"

//就看4盏灯状态，后面要加再加
#define Capp_led    0
#define Spin_led    1
#define Fric_led    2
#define Vers_led    3

#define LED_ON         1
#define LED_OFF        0

// WS2812灯的数量
#define WS2812_NUM 8
// DMA传输的数据长度
#define DMA_BUFFER_SIZE (WS2812_NUM * 24)

// 颜色掩码
#define COLOR_MASK 0x7

extern uint8_t ws2812_data[];
extern uint8_t WS2812_STATUS[4];

void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num);

void ws2812_example(void);
void ws2812_init(uint8_t led_nums);
void ws2812_stauts(uint8_t status,uint8_t led_nums);
extern void ws2812_blue(uint8_t led_nums);
void ws2812_blue_single(uint8_t led_nums);
extern void ws2812_red(uint8_t led_nums);
void ws2812_red_single(uint8_t led_nums);
extern void ws2812_green(uint8_t led_nums);
void RGB_Control (void);


#endif
