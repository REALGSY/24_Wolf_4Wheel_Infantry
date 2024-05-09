#ifndef __BSP_UART_H
#define __BSP_UART_H
#ifdef __cplusplus
extern "C" {
#endif
	
    /* Includes ------------------------------------------------------------------*/
#if defined(USE_HAL_DRIVER)
#if defined(STM32F427xx) || defined(STM32F429xx)
#include <stm32f4xx_hal.h>
#endif
#if defined(STM32F103xx)
#include <stm32f1xx_hal.h>
#endif
#if defined(STM32H750xx)
#include <stm32h7xx_hal.h>
#endif	
#endif
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
    typedef uint32_t(*usart_call_back)(uint8_t* buf, uint16_t len);

/**
* @brief Contain uart control info.
*/
typedef struct
{
    UART_HandleTypeDef* uart_x;
    uint16_t rx_buffer_size;
    uint8_t* rx_buffer;
    usart_call_back call_back_fun;
} usart_manage_t;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern usart_manage_t usart4_manage;
extern usart_manage_t usart5_manage;
extern usart_manage_t usart1_manage;
extern usart_manage_t usart2_manage;
extern usart_manage_t usart3_manage;
extern usart_manage_t usart6_manage;
extern usart_manage_t usart7_manage;

/* Exported function declarations --------------------------------------------*/
void Uart_Init(UART_HandleTypeDef* huart, uint8_t* Rxbuffer, uint32_t length, usart_call_back fun);
void Uart_Receive_Handler(usart_manage_t* m_obj);
uint32_t Uart1_Transmit(uint8_t* msg, uint16_t len);
uint32_t Uart2_Transmit(uint8_t* msg, uint16_t len);
uint32_t Uart3_Transmit(uint8_t* msg, uint16_t len);
uint32_t Uart4_Transmit(uint8_t* msg, uint16_t len);
uint32_t Uart5_Transmit(uint8_t* msg, uint16_t len);
uint32_t Uart6_Transmit(uint8_t* msg, uint16_t len);
uint32_t Uart7_Transmit(uint8_t* msg, uint16_t len);



#ifdef __cplusplus
}
#endif
#endif


