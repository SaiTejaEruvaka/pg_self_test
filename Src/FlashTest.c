/* Copyright (C) Eruvaka Technologies - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * 2018 */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#ifdef PONDGUARD
#include "PGTest.h"
#endif
#include "uart.h"
#include "FlashTest.h"


static uint32_t Flash_ReadId(void);

void FlashTest(void)
{
	uint32_t FlashID = 0;
	/* Enable Flash chip */
	HAL_GPIO_WritePin(SPI_Flash_CS_GPIO_Port, SPI_Flash_CS_Pin, GPIO_PIN_RESET);
	/* Read Flash Id */
	FlashID = Flash_ReadId();
  usartTx(CONSOLE_USART, (const char*)"Flash ID.........%x\r\n", FlashID);
	/* Disable Flash chip */
	HAL_GPIO_WritePin(SPI_Flash_CS_GPIO_Port, SPI_Flash_CS_Pin, GPIO_PIN_SET);
	
	if(FlashID == 0x1f8501)
	{
		self_test_common.self_test_bits_comm.st_flash_b=1;
		#ifdef PONDGUARD
		PG_Cntrltest.self_test_PGcntrl.flash_b = 1;
		#endif
		usartTx(CONSOLE_USART, (const char*)" FLASH..........OK\r\n", FlashID);
	}
	else
	{
		usartTx(CONSOLE_USART, (const char*)"FLASH..........FAIL\r\n", FlashID);
	}
}

static uint8_t sFLASH_SendByte(uint8_t byte)
{
    uint8_t recv;
    HAL_StatusTypeDef status;

    status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&byte, &recv, 1, 1000);
    if(status != HAL_OK)
			usartTx(CONSOLE_USART,"Flash error=%d\r\n",status);
    return recv;
}

static uint32_t Flash_ReadId()
{
	  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
	    /*!< Send "RDID " instruction */
    sFLASH_SendByte(0x9F);

    /*!< Read a byte from the FLASH */
    Temp0 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

    /*!< Read a byte from the FLASH */
    Temp1 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

    /*!< Read a byte from the FLASH */
    Temp2 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
	
		Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

    return Temp;
}



