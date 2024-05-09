/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"
#include "BSP_CAN.h"
#include "tim.h"
#include "Task_CanMsg.h"
#include "IMU_Compensate.h"
#include "DJI_IMU.h"
#include "Chassis_control.h"
#include "DR16_Remote.h"
#include "typedef.h"
#include "Shot.h"
#include "Control_Vision.h"
#include "Cloud_control.h"
#include "DEV_Buzzer.h"
#include "RM_JudgeSystem.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//Message
osMessageQId CAN1_ReceiveHandle;
osMessageQId CAN2_ReceiveHandle;
//Task
osThreadId CAN1_TaskHandle;
osThreadId CAN2_TaskHandle;
osThreadId IMU_ReceiveHandle;
osThreadId Task_ControlHandle;
osThreadId Host_Handle;
osThreadId Check_Handle;
/* USER CODE END Variables */
osThreadId All_InitHandle;
osThreadId RMClientUI_Handle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void CAN1_REC(void const * argument);
extern void CAN2_REC(void const * argument);
extern void IMU_Receive_Run(void const * argument);
extern void RobotControl(void const *argument);  
extern void Host_Run(void const *argument);
extern void DEV_Check(void const *argument);
extern void RMClientUI_Send(void const *argument);
void All_Init_Run(void const * argument);
/* USER CODE END FunctionPrototypes */

void All_Init_Run(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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
	osThreadDef(CAN1_Task, CAN1_REC, osPriorityHigh, 0, 256);
  CAN1_TaskHandle = osThreadCreate(osThread(CAN1_Task), NULL);

	osThreadDef(CAN2_Task, CAN2_REC, osPriorityHigh, 0, 256);
  CAN2_TaskHandle = osThreadCreate(osThread(CAN2_Task), NULL);
	
	osThreadDef(IMU_Receive, IMU_Receive_Run, osPriorityNormal, 0, 256);
  IMU_ReceiveHandle = osThreadCreate(osThread(IMU_Receive), NULL);
	
	osThreadDef(Host_send, Host_Run, osPriorityBelowNormal, 0, 256);
  Host_Handle = osThreadCreate(osThread(Host_send), NULL);
	
	osThreadDef(Check_Task, DEV_Check, osPriorityNormal, 0, 256);
  Check_Handle = osThreadCreate(osThread(Check_Task), NULL);
	
	osThreadDef(RMClientUI_Task, RMClientUI_Send, osPriorityNormal, 0, 256);
  RMClientUI_Handle = osThreadCreate(osThread(RMClientUI_Task), NULL);
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
  /* USER CODE BEGIN All_Init_Run */
  /* Infinite loop */
	taskENTER_CRITICAL();   //进入临界区
	HAL_TIM_Base_Start_IT(&htim2);  //定时器初始化
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim12);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3); //定时器PWM输出初始化
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 630);
  
	CAN_1_Filter_Config();      //CAN过滤器初始化
	CAN_2_Filter_Config();
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //  CAN_IT_FMP0
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); //  CAN_IT_FMP0
	dbus_uart_init();   //dbus初始化
	Control_Vision_FUN.Vision_USART_Receive_DMA(&huart7);  
	Control_Vision_FUN.Vision_Init();
	JudgeSystem_FUN.JudgeSystem_USART_Receive_DMA(&huart3);
	/*********************************临界区保护***************************************/
	//DJI_IMUFUN.DJI_IMU_Init();  
	Robot_Init(); //机器人整体初始化
	Chassis_Init(); //底盘初始化
	osThreadDef(Task_Control, RobotControl, osPriorityRealtime, 0, 600);
  Task_ControlHandle = osThreadCreate(osThread(Task_Control), NULL);
  vTaskDelete(NULL);                    //删除当前任务。
  taskEXIT_CRITICAL();                  //退出临界区
	/*********************************************************************************/
  /* USER CODE END All_Init_Run */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
