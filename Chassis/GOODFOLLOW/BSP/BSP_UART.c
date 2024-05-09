#include "BSP_UART.h"
/* Includes ------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

usart_manage_t usart4_manage =
{
	.call_back_fun = NULL 
};

usart_manage_t usart5_manage =
{
	.call_back_fun = NULL 
};

usart_manage_t usart1_manage =
{
	.call_back_fun = NULL 
};

usart_manage_t usart2_manage =
{
	.call_back_fun = NULL 
};

usart_manage_t usart3_manage =
{
	.call_back_fun = NULL 
};

usart_manage_t usart6_manage =
{
	.call_back_fun = NULL 
};

usart_manage_t usart7_manage =
{
	.call_back_fun = NULL 
};
/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
extern UART_HandleTypeDef huart1;

/* function prototypes -------------------------------------------------------*/


/**
* @brief  Initialize uart device
* @param  *huart: pointer of uart IRQHandler
* @param	Rxbuffer: user buffer array
* @param	length: the length of array
* @retval None
*/
void Uart_Init(UART_HandleTypeDef* huart, uint8_t* Rxbuffer, uint32_t length, usart_call_back fun)
{
	if (huart->Instance == UART4)
	{
		usart4_manage.rx_buffer = Rxbuffer;
		usart4_manage.rx_buffer_size = length;
		usart4_manage.uart_x = huart;
		usart4_manage.call_back_fun = fun;
		__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}

	else if (huart->Instance == UART5)
	{
		usart5_manage.rx_buffer = Rxbuffer;
		usart5_manage.rx_buffer_size = length;
		usart5_manage.uart_x = huart;
		usart5_manage.call_back_fun = fun;
		__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}

	else if (huart->Instance == USART1)
	{
		usart1_manage.rx_buffer = Rxbuffer;
		usart1_manage.rx_buffer_size = length;
		usart1_manage.uart_x = huart;
		usart1_manage.call_back_fun = fun;	
		__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}

	else if (huart->Instance == USART2)
	{
		usart2_manage.rx_buffer = Rxbuffer;
		usart2_manage.rx_buffer_size = length;
		usart2_manage.uart_x = huart;
		usart2_manage.call_back_fun = fun;
		__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}

	else if (huart->Instance == USART3)
	{
		usart3_manage.rx_buffer = Rxbuffer;
		usart3_manage.rx_buffer_size = length;
		usart3_manage.uart_x = huart;
		usart3_manage.call_back_fun = fun;
		__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}

	else if (huart->Instance == USART6)
	{
			usart6_manage.rx_buffer = Rxbuffer;
			usart6_manage.rx_buffer_size = length;
			usart6_manage.uart_x = huart;
			usart6_manage.call_back_fun = fun;
			__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
			__HAL_UART_CLEAR_IDLEFLAG(huart);
			HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}
	else if (huart->Instance == UART7)
	{
			usart7_manage.rx_buffer = Rxbuffer;
			usart7_manage.rx_buffer_size = length;
			usart7_manage.uart_x = huart;
			usart7_manage.call_back_fun = fun;
			__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
			__HAL_UART_CLEAR_IDLEFLAG(huart);
			HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}
}



/**
 * @brief Transmit function for specific Uart.
 * @param msg Message content to send.
 * @param len Message len to send.
 * @retval HAL Status
 */
uint32_t Uart1_Transmit(uint8_t* msg, uint16_t len)
{
    return HAL_UART_Transmit_DMA(usart1_manage.uart_x, msg, len);
}
uint32_t Uart2_Transmit(uint8_t* msg, uint16_t len)
{
    return HAL_UART_Transmit_DMA(usart2_manage.uart_x, msg, len);
}
uint32_t Uart3_Transmit(uint8_t* msg, uint16_t len)
{
    return HAL_UART_Transmit_DMA(usart3_manage.uart_x, msg, len);
}
uint32_t Uart4_Transmit(uint8_t* msg, uint16_t len)
{
    return HAL_UART_Transmit_DMA(usart4_manage.uart_x, msg, len);
}
uint32_t Uart5_Transmit(uint8_t* msg, uint16_t len)
{
    return HAL_UART_Transmit_DMA(usart5_manage.uart_x, msg, len);
}
uint32_t Uart6_Transmit(uint8_t* msg, uint16_t len)
{
    return HAL_UART_Transmit_DMA(usart6_manage.uart_x, msg, len);
}
uint32_t Uart7_Transmit(uint8_t* msg, uint16_t len)
{
    return HAL_UART_Transmit_DMA(usart7_manage.uart_x, msg, len);
}


/**
 * @brief   Determine if the idle interrupt is triggered
 * @param   m_obj: serial port handle
 * @retval  None
 */
void Uart_Receive_Handler(usart_manage_t* m_obj)
{	
	if(__HAL_UART_GET_FLAG(m_obj->uart_x,UART_FLAG_IDLE)!=RESET)
	{
		/* Check the parameters */
	    assert_param(m_obj != NULL);
	
        /* Private variables */
	    static uint16_t usart_rx_num;

        /* clear idle it flag avoid idle interrupt all the time */
	    __HAL_UART_CLEAR_IDLEFLAG(m_obj->uart_x);

        /* clear DMA transfer complete flag */
	    HAL_UART_DMAStop(m_obj->uart_x);

        /* handle received data in idle interrupt */
	    usart_rx_num = m_obj->rx_buffer_size - ((DMA_Stream_TypeDef*)m_obj->uart_x->hdmarx->Instance)->NDTR;
		
	    if(m_obj->call_back_fun != NULL)
        {
					m_obj->call_back_fun(m_obj->rx_buffer, usart_rx_num);
        }
	    HAL_UART_Receive_DMA(m_obj->uart_x, m_obj->rx_buffer, m_obj->rx_buffer_size);
	}
}




