/**
 * @file USER_Filter.c
 * @author Someone
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "USER_Filter.h"


/**
  * @brief  IIR��ͨ�˲������������������ͬһ������
  * @param[in]  *in ��������
  *				 LpfAttFactor ��ͨ�˲�˥������ Attenuation should be between 0 to 1.
  * @param[out]	*out �������
  * @retval None
  */
void Filter_IIRLPF(float *in,float *out, float LpfAttFactor)
{
	*out = *out + LpfAttFactor*(*in - *out); 
}
