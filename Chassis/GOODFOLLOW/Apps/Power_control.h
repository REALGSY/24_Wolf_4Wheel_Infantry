#include "typedef.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H



void Limit_Calc (void);
void Limit(int16_t *wheelCurrent, int8_t amount);
#endif /* __M2006_MOTOR_H */
