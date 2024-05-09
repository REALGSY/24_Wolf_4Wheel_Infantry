/**
 * @file CRC.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __DRIVER_CRC_H 
#define __DRIVER_CRC_H 

#include "stm32f4xx.h"

/*CRC8ะฃั้*/
unsigned char Checksum_CRC8(uint8_t *buf,uint16_t len);

#endif	// __DRIVER_CRC_H
/*-----------------------------------file of end------------------------------*/


