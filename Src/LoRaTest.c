/* Copyright (C) Eruvaka Technologies - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * 2018 */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#ifdef PONDGUARD
#include "PGTest.h"
#endif
#include "LoRaTest.h"
#include "uart.h"
char lora_uart_buff[LORA_BUF_SIZE];
uint16_t lora_uart_read_counter;
char lora_data_buff[LORA_BUF_SIZE];

#ifdef MODULE_LORA_MCHIP
static void MChip_LoRaTest(void);
static int LoRaRx(void);
#else
static void HopeRF_LoRaTest(void);
#endif

#define MCHIP_DELIM "\n"
#define MCHIP_CMD(str) str MCHIP_DELIM

#ifdef MODULE_LORA_MCHIP
	static void MChip_LoRaTest(void)
	{
		int data_len = 0;
		HAL_UART_Receive_IT(&huart2, (uint8_t *)lora_uart_buff, LORA_BUF_SIZE);	
		/* power on LoRa chip */
    HAL_GPIO_WritePin(LORA_PWR_EN_GPIO_Port, LORA_PWR_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
		#ifndef LORA_CHANGES
		HAL_Delay(1000);
		#else
		HAL_Delay(330);
		#endif
		data_len = LoRaRx();
		if(data_len > 0)
		{
				usartTx(CONSOLE_USART,(const char *)"LORA CHIP.........%s\r\n",lora_data_buff);
				usartTx(LORA_USART,(const char *)MCHIP_CMD("mac pause\r"));
		    #ifndef LORA_CHANGES
				HAL_Delay(20);
		    #else
		    HAL_Delay(10);
		    #endif
				data_len = LoRaRx();
				if((data_len > 0) && (strcmp(lora_data_buff,"4294967245\r") == 0))
				{
					self_test_common.self_test_bits_comm.st_lora_b=1;
					#ifdef PONDGUARD
					PG_Cntrltest.self_test_PGcntrl.LoRa_comm_b = 1;
					#endif
					usartTx(CONSOLE_USART,(const char *)" LORA.........OK\r\n");
				}
				else
				{
					usartTx(CONSOLE_USART,(const char *)"LORA.........FAIL\r\n");
				}			
		}
		else
		{
			usartTx(CONSOLE_USART,(const char *)"LORA.........FAIL\r\n");
		}
	}	
		
	static int LoRaRx(void)
	{
			static uint16_t buf_count = 0;
			uint16_t ret_val = 0, write_counter;
			uint8_t *buffer = (uint8_t*)lora_uart_buff;
			write_counter = huart2.pRxBuffPtr - buffer;
			if (buf_count == 0)
					memset(lora_data_buff, 0, sizeof(lora_data_buff));
			if (lora_uart_read_counter != write_counter)
			{
			}

			while (lora_uart_read_counter != write_counter)
			{
					if (lora_uart_read_counter == LORA_BUF_SIZE)
					{
							lora_uart_read_counter = 0;
							if (lora_uart_read_counter == write_counter)
									break;
					}

					/* Take Data from Interrupt buffer to Parse Buffer */
					lora_data_buff[buf_count] = lora_uart_buff[lora_uart_read_counter];
					lora_uart_read_counter++;

					if (lora_data_buff[buf_count] == '\n')
					{
							lora_data_buff[buf_count] = '\0';
						  ret_val = buf_count;
							buf_count = 0;
							return ret_val;
					}
					else
					{
							/* Increment to read next character Message */
							buf_count++;
					}

					if (buf_count == sizeof(lora_data_buff))
							buf_count = 0;
			}
			return ret_val;
	}

#else
	static void HopeRF_LoRaTest(void)
	{
		
	}
	
#endif
	
void LoRaTest(void)
{
	#ifdef MODULE_LORA_MCHIP
		MChip_LoRaTest();
	#else
		HopeRF_LoRaTest();
	#endif
}
