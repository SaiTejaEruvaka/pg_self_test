/* Copyright (C) Eruvaka Technologies - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * 2018 */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdarg.h>
#ifdef PONDGUARD
#include "PGTest.h"
#include "LoRaTest.h"
#endif
#define MAX_DEBUG_MSG_SIZE 100
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
char usart_buffer[50];
extern uint8_t recv_buffer[100];

extern char lora_uart_buff[LORA_BUF_SIZE];
extern sBLE_RX_BUFF_t sBLErxBuff;
signed int usartTx(USART_TypeDef *Usart, const char *pFormat, ...)
{
    va_list ap;
    signed int result, len;
    char msg_buf[MAX_DEBUG_MSG_SIZE];

    /* Forward call to vprintf */
    va_start(ap, pFormat);
    len = vsnprintf(msg_buf, sizeof(msg_buf), pFormat, ap);
    va_end(ap);

		if(len > sizeof(msg_buf))
		{
				len = sizeof(msg_buf) -1;
				msg_buf[len-2] = '\r';
				msg_buf[len-1] = '\n';
		}

		if(Usart == USART1)
    {
        HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, len, 5000);
    }
    else if (Usart == USART2)
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)msg_buf, len, 5000);
    }
    else if (Usart == USART3)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)msg_buf, len, 5000);
    }

    return result;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		HAL_UART_Receive_IT(&huart2, (uint8_t *)lora_uart_buff, LORA_BUF_SIZE);
	}
	#ifdef PONDGUARD
	else if (huart == &hlpuart1) {
		HAL_UART_Receive_IT(&hlpuart1,sGPSrxBuff.ui8DataBuff,MAX_GPS_RX_BUFFER_SIZE);	
	}	
	else if (huart == &huart3) {
		HAL_UART_Receive_IT(&huart3,sBLErxBuff.ui8DataBuff,MAX_BLE_RX_BUFFER_SIZE);	
	}
	#endif
	else if(huart==&huart1){
		HAL_UART_Receive_IT(&huart1,recv_buffer,sizeof(recv_buffer));
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {    
			usartTx(CONSOLE_USART,(const char *)"\r\n Error UART2");
    }
		#ifdef PONDGUARD
		else if (huart == &huart3) {
			HAL_UART_Receive_IT(&huart3,sBLErxBuff.ui8DataBuff,MAX_BLE_RX_BUFFER_SIZE);
			usartTx(CONSOLE_USART,(const char *)"\r\n Error UART3");
		}
		else if(huart == &hlpuart1)
    {
			HAL_UART_Receive_IT(&hlpuart1,sGPSrxBuff.ui8DataBuff,MAX_GPS_RX_BUFFER_SIZE);
			usartTx(CONSOLE_USART,(const char *)"\r\n Error hlpuart1");
    }
		else if(huart==&huart1){
			usartTx(CONSOLE_USART,(const char *)"\r\n Error UART1");
			HAL_UART_Receive_IT(&huart1,recv_buffer,sizeof(recv_buffer));
		}
		#endif
}
	
