/* Copyright (C) Eruvaka Technologies - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * 2018 */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
#include "LCDTest.h"

char LCD[LCD_MAX_LINES][LINE_SIZE + 1];
int LCDClickevent = 0;
lcd_click_status_t lcd_click_status;

extern SPI_HandleTypeDef hspi1;
extern void MX_SPI1_Init(uint32_t lines);
void LCD_Init(void)
{
		HAL_SPI_DeInit(&hspi1);
	  HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_RESET);
	  /*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, SPI_NSS_Pin,GPIO_PIN_RESET);
		MX_SPI1_Init(SPI_DIRECTION_1LINE);
	  HAL_Delay(10);
		GLCD_CS_LOW();
		glcd_ST7565R_init();
		glcd_select_screen(glcd_buffer, &glcd_bbox);
		glcd_clear();
		glcd_clear_now();		
	  glcd_write();
		GLCD_CS_HIGH();
}

void LCDTest(void)
{
	LCD_Init();
	// Draw the Eruvaka Technologies string to UI
	snprintf((char*)LCD[3], LINE_SIZE, "      ERUVAKA ");
	snprintf((char*)LCD[5], LINE_SIZE, "    TECHNOLOGIES");
	GLCD_CS_LOW();
	//glcd_clear();
	//glcd_clear_now();	
	set_tiny_font();
	/* Draw the strings of all the LCD buffers */
	glcd_tiny_draw_string(1, 3, LCD[3]);
	glcd_tiny_draw_string(1, 5, LCD[5]);
	glcd_write();
	GLCD_CS_HIGH();
	HAL_Delay(100);
}

void SWTest(void)
{
	if(LCDClickevent == 1)
	{
		LCDClickevent=0;
		if(lcd_click_status == MENU_CLICK_EVENT)
		{
			snprintf((char*)LCD[3], LINE_SIZE, "    PRESSED MENU ");
		}
		else if(lcd_click_status == UP_CLICK_EVENT)
		{
			snprintf((char*)LCD[3], LINE_SIZE, "    PRESSED UP ");
		}
		else if(lcd_click_status == DOWN_CLICK_EVENT)
		{
			snprintf((char*)LCD[3], LINE_SIZE, "    PRESSED DOWN ");
		}
		else if(lcd_click_status == BACK_CLICK_EVENT)
		{
			snprintf((char*)LCD[3], LINE_SIZE, "    PRESSED BACK ");
		}
		#ifdef PONDGUARD
		else if(lcd_click_status == TRAIL_RUN)
		{
			snprintf((char*)LCD[3], LINE_SIZE, " PRESSED MANUAL TRAIL RUN ");
		}
		#endif
		else
		{
			snprintf((char*)LCD[3], LINE_SIZE, " UNKNOWN INTERRUPT");		
		}
		GLCD_CS_LOW();
		glcd_clear();
	  glcd_clear_now();
		set_tiny_font();
		/* Draw the strings of all the LCD buffers */
		glcd_tiny_draw_string(1, 3, LCD[3]);
		glcd_write();
		GLCD_CS_HIGH();
		lcd_click_status = NO_CLICK_EVENT;
	}
}

int  PLATFORM_spi1_write(uint8_t *cmd, uint8_t len)
{
	int status;
	
	status = HAL_SPI_Transmit(&hspi1, (uint8_t *)cmd, 1,1000);
	return status;
}

//
