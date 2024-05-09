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
//  /* �����ⲿ���� */
//  extern uint16_t Rx_Date_Num,RX_goal_num;
//  extern uint8_t UserRxBuffer[APP_RX_DATA_SIZE];
//  extern uint8_t Rx_status;
//  extern uint8_t* p;

//  /* ������յ������� */
//  Rx_date_save(Buf,UserRxBuffer,*Len);
//  /* ������յ���������С�ڻ���ڻ�������С�����ӽ������ݵ����� */
//  if(Rx_Date_Num<=APP_RX_DATA_SIZE)
//      Rx_Date_Num+=*Len;
//  /* ������յ������������ڻ�������С�����������ݵ���������Ϊ��������С */
//  else
//      Rx_Date_Num=APP_RX_DATA_SIZE;

//  /* �������״̬Ϊ0 */
//  if(Rx_status==0)
//  {
//    /* ������յ������������ڻ����Ŀ�������� */
//    if(Rx_Date_Num>=RX_goal_num)
//    {
//      /* ���û����ջ����������ݸ��Ƶ�pָ���λ�� */
//      Rx_buffer_copy(p,UserRxBuffer,RX_goal_num);
//      /* ���ٽ������ݵ����� */
//        Rx_Date_Num-=RX_goal_num;
//      /* ������״̬����Ϊ1 */
//      Rx_status=1;
//    }
//  }
//  /* ����USB�豸�Ľ��ջ����� */
//  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
//  /* ����USB���ݰ� */
//  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
//  /* ���ز������ */
//  return (USBD_OK);
//  /* USER CODE END 6 */
//}

///**
//  * @brief  ����������ڽ���USB���⴮�ڵ�����
//  * @param  Rx_Buffer: ���ջ�����
//  * @param  num: ��Ҫ���յ���������
//  * @param  overtime: ��ʱʱ��
//  * @retval ������ճɹ�������1�������ʱ������0
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
//  * @brief  �����������ݣ�����������ɽ��������ȫ�ֱ���Rx_status��һ������Ϊ0
//  * @param  Rx_Buffer: ���ջ�����
//  * @param  num: ��Ҫ���յ���������
//  * @retval ��
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
//  /* �����ⲿ���� */
//  extern uint16_t Rx_Date_Num,RX_goal_num;
//  extern uint8_t UserRxBuffer[APP_RX_DATA_SIZE];
//  extern uint8_t Rx_status;
//  extern uint8_t* p;

//  /* ������յ������� */
//  Rx_date_save(Buf,UserRxBuffer,*Len);
//  /* ������յ���������С�ڻ���ڻ�������С�����ӽ������ݵ����� */
//  if(Rx_Date_Num<=APP_RX_DATA_SIZE)
//      Rx_Date_Num+=*Len;
//  /* ������յ������������ڻ�������С�����������ݵ���������Ϊ��������С */
//  else
//      Rx_Date_Num=APP_RX_DATA_SIZE;

//  /* �������״̬Ϊ0 */
//  if(Rx_status==0)
//  {
//    /* ������յ������������ڻ����Ŀ�������� */
//    if(Rx_Date_Num>=RX_goal_num)
//    {
//      /* ���û����ջ����������ݸ��Ƶ�pָ���λ�� */
//      Rx_buffer_copy(p,UserRxBuffer,RX_goal_num);
//      /* ���ٽ������ݵ����� */
//        Rx_Date_Num-=RX_goal_num;
//      /* ������״̬����Ϊ1 */
//      Rx_status=1;
//    }
//  }
//  /* ����USB�豸�Ľ��ջ����� */
//  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
//  /* ����USB���ݰ� */
//  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
//  /* ���ز������ */
//  return (USBD_OK);
//  /* USER CODE END 6 */
//}

///**
//  * @brief  ����������ڸ��ƽ��ջ����������ݣ�����������������λ
//  * @param  Buffer_get: ��ȡ������
//  * @param  Buffer_put: ���û�����
//  * @param  num: Ҫ���Ƶ�Ԫ������
//  * @retval ��
//  */

//void Rx_buffer_copy(uint8_t* Buffer_get,uint8_t* Buffer_put,uint16_t num)
//{
//    uint16_t i=0;
//    for(i=0;i<num;i++)//��������
//    {
//        Buffer_get[i]=Buffer_put[i];
//    }
//    for(i=0;i<Rx_Date_Num-num;i++)//ʣ��������λ
//    {
//        Buffer_put[i]=Buffer_put[i+num];
//    }
//}

///**
//  * @brief  ����������ڽ�һ����������ݸ��Ƶ���һ�������У������ᶪʧ���������е�ԭʼ����
//  * @param  src: Դ����
//  * @param  dest: Ŀ������
//  * @param  n: Դ�����е�Ԫ������
//  * @retval ��
//  */
//void Rx_date_save(uint8_t* src, uint8_t* dest, uint16_t n)
//{
//    uint16_t i=0,num=Rx_Date_Num;
//    if(num+n>APP_RX_DATA_SIZE)
//    return;//������������С������ֱ��ֹͣ��
//    for(i=0;i<n;i++)
//        dest[i+num]=src[i];
//}

///**
//  * @brief  ����������ڻ�ȡUSB���ջ���������������
//  * @param  ��
//  * @retval ���ؽ��յ���������
//  */

//uint16_t usb_Rx_Get_Num(void)
//{
//    return Rx_Date_Num;
//}
