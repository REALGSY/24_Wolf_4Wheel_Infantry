/**
 * @file BSP_ADC.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-04-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __BSPADC_H__
#define __BSPADC_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <adc.h>
#include <stm32f4xx_hal_adc.h>

#define ADC_ChannelConInit           \
    {                                \
        ADC_CHANNEL_18,              \
            1,                       \
            ADC_SAMPLETIME_15CYCLES, \
            0,                       \
    }

#define ADC_FUNGroundInit            \
    {                                \
        &get_temprate,              \
    }

typedef struct
{
    float (*get_temprate)(void);

} ADC_FUN_t;

extern ADC_FUN_t ADC_FUN;

#endif /*__BSPADC_H__*/
