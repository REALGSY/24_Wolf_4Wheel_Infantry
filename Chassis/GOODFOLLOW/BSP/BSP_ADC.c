/**
 * @file BSP_ADC.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-04-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "BSP_ADC.h"
ADC_ChannelConfTypeDef ADC_ChannelCon = ADC_ChannelConInit;
#undef ADC_ChannelConInit

float get_temprate(void);
ADC_FUN_t ADC_FUN = ADC_FUNGroundInit;

static void temperature_ADC_Reset(void)
{
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);
}

static uint16_t get_ADC()
{

    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_STRT | ADC_FLAG_OVR | ADC_FLAG_EOC);
    HAL_ADC_ConfigChannel(&hadc1, &ADC_ChannelCon);

    // __HAL_ADC_IS_SOFTWARE_START_REGULAR(&hadc1);
    HAL_ADC_Start(&hadc1);
    // HAL_ADC_PollForConversion(&hadc1, 10);
    while (!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC))
    {
        ;
    }

    return HAL_ADC_GetValue(&hadc1);
}

float get_temprate(void)
{
    uint16_t adcx = 0;
    float temperate;
    temperature_ADC_Reset();
    adcx = get_ADC();
    temperate = (float)adcx * (3.3f / 4096.0f);
    temperate = (temperate - 0.76f) / 0.0025f + 25.0f;
    return temperate;
}
