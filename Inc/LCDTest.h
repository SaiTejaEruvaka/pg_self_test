/* Copyright (C) Eruvaka Technologies - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * 2018 */

#ifndef __LCDTEST_H__
#define __LCDTEST_H__

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
#include "glcd.h"
#include "ST7565R.h"
#include "glcd_text_tiny.h"

#define LINE_SIZE			22
#define LCD_MAX_LINES	 8

typedef enum
{
	NO_CLICK_EVENT,
	MENU_CLICK_EVENT,
	UP_CLICK_EVENT,
	DOWN_CLICK_EVENT,
	BACK_CLICK_EVENT,
	WAKE_UP_CLICK_EVENT,
	#ifdef PONDGUARD
	TRAIL_RUN,
	#endif
}lcd_click_status_t;

extern lcd_click_status_t lcd_click_status;

void LCDTest(void);
void SWTest(void);
int  PLATFORM_spi1_write(uint8_t *cmd, uint8_t len);

extern int LCDClickevent;

#endif
//
