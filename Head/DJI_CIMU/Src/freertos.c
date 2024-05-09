/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Control_Vision.h"
#include "calibrate_task.h"
#include "typedef.h"
#include "INS_task.h"
#include "led_flow_task.h"
#include "Debug_DataScope.h"
#include "DJI_IMU.h"
#include "bsp_buzzer.h"
#include "BSP_CAN.h"
#include "bsp_delay.h"
#include "DR16_Remote.h"
#include "Chassis_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

osThreadId calibrate_tast_handle;
osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//CAN队列句柄
osMessageQId CAN2_Queue;
osMessageQId CAN1_ReceiveHandle;
osMessageQId CAN2_ReceiveHandle;
osThreadId IMU_Send_TaskHandle;
osThreadId CAN1_TaskHandle;
osThreadId CAN2_TaskHandle;
osThreadId Debug_TaskHandle;
osThreadId Task_ControlHandle;
osThreadId Host_Handle;
/* USER CODE END Variables */
osThreadId All_InitHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void IMU_Send(void const * argument);
extern void CAN1_REC(void const * argument);
extern void CAN2_REC(void const * argument);
extern void Debug(void const * argument);
extern void RobotControl(void const *argument);  
extern void Host_Run(void const *argument);
/* USER CODE END FunctionPrototypes */

void All_Init_Run(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	CAN2_Queue = xQueueCreate(64,sizeof(CAN_Rx_TypeDef));
	
	osMessageQDef(CAN1_Receive, 64, Can_Export_Data_t);
  CAN1_ReceiveHandle = osMessageCreate(osMessageQ(CAN1_Receive), NULL);
	
	osMessageQDef(CAN2_Receive, 64, Can_Export_Data_t);
  CAN2_ReceiveHandle = osMessageCreate(osMessageQ(CAN2_Receive), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of All_Init */
  osThreadDef(All_Init, All_Init_Run, osPriorityNormal, 0, 128);
  All_InitHandle = osThreadCreate(osThread(All_Init), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(IMU_Send_Task, IMU_Send, osPriorityHigh, 0, 128);
  IMU_Send_TaskHandle = osThreadCreate(osThread(IMU_Send_Task), NULL);
	
	osThreadDef(CAN1_Task, CAN1_REC, osPriorityHigh, 0, 256);
  CAN1_TaskHandle = osThreadCreate(osThread(CAN1_Task), NULL);

	osThreadDef(CAN2_Task, CAN2_REC, osPriorityHigh, 0, 256);
  CAN2_TaskHandle = osThreadCreate(osThread(CAN2_Task), NULL);
	
  osThreadDef(cali, calibrate_task, osPriorityAboveNormal, 0, 512);
  calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);

  osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  osThreadDef(led, led_RGB_flow_task, osPriorityNormal, 0, 256);
  led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);
	
	osThreadDef(Debug_Task, Debug, osPriorityNormal, 0, 128);
  Debug_TaskHandle = osThreadCreate(osThread(Debug_Task), NULL);
	
	osThreadDef(Host_send, Host_Run, osPriorityLow, 0, 256);
  Host_Handle = osThreadCreate(osThread(Host_send), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_All_Init_Run */
/**
  * @brief  Function implementing the All_Init thread.
  * @param  argument: Not used
  * @retval None
  */

/* USER CODE END Header_All_Init_Run */
void All_Init_Run(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN All_Init_Run */
	taskENTER_CRITICAL();   //进入临界区
	delay_init();
	Buzzer_Init();
  cali_param_init();
  CAN_1_Filter_Config();
  CAN_2_Filter_Config();
	DR16_Fun.DR16_USART_Receive_DMA(&huart3);
	Robot_Init();
	Chassis_Init();
	Control_Vision_FUN.Vision_USART_Receive_DMA(&huart1);
	Control_Vision_FUN.Vision_Init();//开机的时候先把位置给了
	
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_MspPostInit(&htim1);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1750);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 4100);
	
  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //  CAN_IT_FMP0
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); //  CAN_IT_FMP0
  osThreadDef(Task_Control, RobotControl, osPriorityRealtime, 0, 600);
  Task_ControlHandle = osThreadCreate(osThread(Task_Control), NULL);
	taskEXIT_CRITICAL();                  //退出临界区
	vTaskDelete(NULL);    
  /* USER CODE END All_Init_Run */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */
