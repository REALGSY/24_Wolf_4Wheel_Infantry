///**
// * @file Chassis_control.c
// * @author Gsy
// * @brief 
// * @version 1
// * @date 2024-04-14
// * 
// * @copyright Copyright (c) 2021
// * 
// */
// #include "bsp_usb.h"
// #include <stdio.h>
// #include "usbd_cdc_if.h"
// static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
//{
//  /* USER CODE BEGIN 6 */
//  
//  /* 定义外部变量 */
//  extern uint16_t Rx_Date_Num,RX_goal_num;
//  extern uint8_t UserRxBuffer[APP_RX_DATA_SIZE];
//  extern uint8_t Rx_status;
//  extern uint8_t* p;

//  /* 保存接收到的数据 */
//  Rx_date_save(Buf,UserRxBuffer,*Len);
//  /* 如果接收到的数据量小于或等于缓冲区大小，增加接收数据的数量 */
//  if(Rx_Date_Num<=APP_RX_DATA_SIZE)
//      Rx_Date_Num+=*Len;
//  /* 如果接收到的数据量大于缓冲区大小，将接收数据的数量设置为缓冲区大小 */
//  else
//      Rx_Date_Num=APP_RX_DATA_SIZE;

//  /* 如果接收状态为0 */
//  if(Rx_status==0)
//  {
//    /* 如果接收到的数据量大于或等于目标数据量 */
//    if(Rx_Date_Num>=RX_goal_num)
//    {
//      /* 将用户接收缓冲区的数据复制到p指向的位置 */
//      Rx_buffer_copy(p,UserRxBuffer,RX_goal_num);
//      /* 减少接收数据的数量 */
//        Rx_Date_Num-=RX_goal_num;
//      /* 将接收状态设置为1 */
//      Rx_status=1;
//    }
//  }
//  /* 设置USB设备的接收缓冲区 */
//  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
//  /* 接收USB数据包 */
//  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
//  /* 返回操作结果 */
//  return (USBD_OK);
//  /* USER CODE END 6 */
//}

///**
//  * @brief  这个函数用于接收USB虚拟串口的数据
//  * @param  Rx_Buffer: 接收缓冲区
//  * @param  num: 需要接收的数据数量
//  * @param  overtime: 超时时间
//  * @retval 如果接收成功，返回1，如果超时，返回0
//  */
// 
//uint8_t usb_vbc_Receive(uint8_t* Rx_Buffer,uint16_t num,uint32_t overtime)
//{
//    uint32_t time=0;
//    overtime=overtime/2;
//    if(Rx_Date_Num>=num)
//    {
//        Rx_buffer_copy(Rx_Buffer,UserRxBuffer,num);
//        Rx_Date_Num-=num;
//        return 1;
//    }
//    else
//    {
//        while(1)
//        {
//            if(Rx_Date_Num>=num)
//            {
//                Rx_buffer_copy(Rx_Buffer,UserRxBuffer,num);
//                Rx_Date_Num-=num;
//                return 1;
//            }
//            else
//                time++;
//            if(time>overtime)
//                return 0;
//            HAL_Delay(1);
//        }
//    }
//}

///**
//  * @brief  开启接收数据，不堵塞，完成接收任务后，全局变量Rx_status置一，否则为0
//  * @param  Rx_Buffer: 接收缓冲区
//  * @param  num: 需要接收的数据数量
//  * @retval 无
//  */
//void usb_vbc_Receive_It(uint8_t* Rx_Buffer,uint16_t num)
//{
//    p=Rx_Buffer;
//    RX_goal_num=num;
//    Rx_status=0;
//}

//static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
//{
//  /* USER CODE BEGIN 6 */
//  
//  /* 定义外部变量 */
//  extern uint16_t Rx_Date_Num,RX_goal_num;
//  extern uint8_t UserRxBuffer[APP_RX_DATA_SIZE];
//  extern uint8_t Rx_status;
//  extern uint8_t* p;

//  /* 保存接收到的数据 */
//  Rx_date_save(Buf,UserRxBuffer,*Len);
//  /* 如果接收到的数据量小于或等于缓冲区大小，增加接收数据的数量 */
//  if(Rx_Date_Num<=APP_RX_DATA_SIZE)
//      Rx_Date_Num+=*Len;
//  /* 如果接收到的数据量大于缓冲区大小，将接收数据的数量设置为缓冲区大小 */
//  else
//      Rx_Date_Num=APP_RX_DATA_SIZE;

//  /* 如果接收状态为0 */
//  if(Rx_status==0)
//  {
//    /* 如果接收到的数据量大于或等于目标数据量 */
//    if(Rx_Date_Num>=RX_goal_num)
//    {
//      /* 将用户接收缓冲区的数据复制到p指向的位置 */
//      Rx_buffer_copy(p,UserRxBuffer,RX_goal_num);
//      /* 减少接收数据的数量 */
//        Rx_Date_Num-=RX_goal_num;
//      /* 将接收状态设置为1 */
//      Rx_status=1;
//    }
//  }
//  /* 设置USB设备的接收缓冲区 */
//  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
//  /* 接收USB数据包 */
//  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
//  /* 返回操作结果 */
//  return (USBD_OK);
//  /* USER CODE END 6 */
//}

///**
//  * @brief  这个函数用于复制接收缓冲区的内容，并将缓存区数据移位
//  * @param  Buffer_get: 获取缓冲区
//  * @param  Buffer_put: 放置缓冲区
//  * @param  num: 要复制的元素数量
//  * @retval 无
//  */

//void Rx_buffer_copy(uint8_t* Buffer_get,uint8_t* Buffer_put,uint16_t num)
//{
//    uint16_t i=0;
//    for(i=0;i<num;i++)//复制数据
//    {
//        Buffer_get[i]=Buffer_put[i];
//    }
//    for(i=0;i<Rx_Date_Num-num;i++)//剩余数据移位
//    {
//        Buffer_put[i]=Buffer_put[i+num];
//    }
//}

///**
//  * @brief  这个函数用于将一个数组的内容复制到另一个数组中，而不会丢失接收数组中的原始数据
//  * @param  src: 源数组
//  * @param  dest: 目标数组
//  * @param  n: 源数组中的元素数量
//  * @retval 无
//  */
//void Rx_date_save(uint8_t* src, uint8_t* dest, uint16_t n)
//{
//    uint16_t i=0,num=Rx_Date_Num;
//    if(num+n>APP_RX_DATA_SIZE)
//    return;//超出缓存区大小，这里直接停止。
//    for(i=0;i<n;i++)
//        dest[i+num]=src[i];
//}

///**
//  * @brief  这个函数用于获取USB接收缓存区的数据数量
//  * @param  无
//  * @retval 返回接收的数据数量
//  */

//uint16_t usb_Rx_Get_Num(void)
//{
//    return Rx_Date_Num;
//}
