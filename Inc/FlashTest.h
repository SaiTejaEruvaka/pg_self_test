/* Copyright (C) Eruvaka Technologies - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * 2018 */

#ifndef __FLASHTEST_H__
#define __FLASHTEST_H__

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"

#define sFLASH_DUMMY_BYTE         0xA5

extern SPI_HandleTypeDef hspi1;

void FlashTest(void);

#endif

