/**
 * @file USER_Filter.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __FILTER_H
#define __FILTER_H


#include "stm32f4xx.h"
#include "typedef.h"


void Filter_IIRLPF(float *in,float *out, float LpfAttFactor);




#endif /* __FILTER_H */



