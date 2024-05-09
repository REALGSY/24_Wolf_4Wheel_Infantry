/**
 * @file Status_Update.h
 * @author Gsy
 * @brief  
 * @version 1
 * @date 2022-07-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
 
#ifndef _DJI_IMU_H
#define _DJI_IMU_H

void RemoteControl_Update (void);
void RemoteMode_Update(void);
void RemoteControl_PC_Update(void);  //���̿���
extern float Forward_Back_Value_Direction ; //�ı�ǰ������
extern float Left_Right_Value_Direction ;   //�ı����ҷ���
extern int Turn_Round_Flag;
extern int Half_Ship_Flag ;  //��ȫģʽ
#define ForwardBackGroundInit \
    {                         \
        0,                    \
            0,                \
            -659,            \
            660,             \
    }

#define LeftRightGroundInit \
    {                       \
        0,                  \
            0,              \
            -659,          \
            660,           \
    }

#define RotateGroundInit \
    {                    \
        0,               \
            0,           \
            1,           \
            13,          \
    }

		
#endif
