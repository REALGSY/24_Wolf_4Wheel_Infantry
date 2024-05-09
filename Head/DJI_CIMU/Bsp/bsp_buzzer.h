#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "main.h"
#include "tim.h"

#define Buzzer_Continue_Times 300
#define Buzzer_Val 135
#define Low_Buzzer_Val 5
typedef enum
{
    Zero_times = 0,
    One_times = 1,
		heaps_times = 2,
		Three_times = 3
} buzzer_mode_u;
typedef enum
{
	OFF = 0,
	ON = 1
}buzzer_status_e;
typedef struct
{
    buzzer_mode_u mode;
    uint16_t buzzer_value;
    uint16_t continue_times;
    uint16_t times;
		buzzer_status_e  status;
} Buzzer_t;
extern Buzzer_t Buzzer;

void Buzzer_Processing(void);
void Buzzer_Init(void);

#endif
