/**
 * @file DevicesCheck.h
 * @author Gsy
 * @brief 
 * @version 1
 * @date 2022-10-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef  __DEVICESCHECK_H
#define __DEVICESCHECK_H

#define Dev_Check_Init  \
{                       \
	&Offline_Check,			\
}

typedef struct
{
	void (*Offline_Check) (void);
}Dev_Check_t;

extern Dev_Check_t  Dev_Check;
#endif

