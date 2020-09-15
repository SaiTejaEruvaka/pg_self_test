/* Copyright (C) Eruvaka Technologies - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * 2018 */

#ifndef __LORATEST_H__
#define __LORATEST_H__

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"

#define LORA_BUF_SIZE 50

extern char lora_uart_buff[LORA_BUF_SIZE];

void LoRaTest(void);

#endif

