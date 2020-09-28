/* Copyright (C) Eruvaka Technologies - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * 2018 */

#ifndef __ADC_H__
#define __ADC_H__

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_adc.h"
#ifndef PM_V16
#ifdef PONDMOTHER
  #ifdef PM_V13_RELAY_CHANGES
	#define ADC_NUM_OF_CHANNELS	4
	#else
	#define ADC_NUM_OF_CHANNELS	3
	#endif
#else
	#define ADC_NUM_OF_CHANNELS	2
#endif
#else
 #define ADC_NUM_OF_CHANNELS	4
#endif
extern ADC_HandleTypeDef hadc1;
extern uint16_t ADCConvertedValue[ADC_NUM_OF_CHANNELS];
extern uint16_t ADCConvertedValue_DMA[ADC_NUM_OF_CHANNELS];

void ADCTest(void);

#endif

