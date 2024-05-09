#include <stdint.h>
#include "tim.h"

#define L1	262
#define L1U	277
#define L2	294
#define L2U	311
#define L3	330
#define L4	349
#define L4U	370
#define L5	392
#define L5U	415
#define L6	440
#define L6U	466
#define L7	494

#define M1	523
#define M1U	554
#define M2	587
#define M2U	622
#define M3	659
#define M4	698
#define M4U	740
#define M5	784
#define M5U	831
#define M6	880
#define M6U	932
#define M7	988

#define H1	1046
#define H1U	1109
#define H2	1175
#define H2U	1245
#define H3	1318
#define H4	1397
#define H4U	1480
#define H5	1568
#define H5U	1661
#define H6	1760
#define H6U	1865
#define H7	1976


/* 蜂鸣器响铃类型 */
typedef enum
{
	Ring_Stop = 0,
	Ring_ContLong = -1,
	Ring_1time = 1,
	Ring_2times = 2,
	Ring_3times = 3,
	Ring_4times = 4,
	Ring_5times = 5,
	Ring_ContShort = 125
}RingType_e;


typedef struct
{
	uint16_t note; // 音名
	uint16_t time; // 时值
}MusicNote_t;

void Music_Play(MusicNote_t music[], uint16_t len);
extern MusicNote_t TheWindRises[227];
extern MusicNote_t SuperMario[17];

