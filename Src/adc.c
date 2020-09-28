/* Copyright (C) Eruvaka Technologies - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * 2018 */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
static float measure12Voltage(void);
static float measureLiIonVoltage(void);
#ifdef PM_V16
extern void GPIO_Init(void);
extern void Doser_dispenser_check(void);
#endif
uint16_t ADCConvertedValue[ADC_NUM_OF_CHANNELS];
uint16_t ADCConvertedValue_DMA[ADC_NUM_OF_CHANNELS];
void ADCTest(void)
{
//	if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADCConvertedValue_DMA,ADC_NUM_OF_CHANNELS) != HAL_OK)
//	{
//		usartTx(CONSOLE_USART,(const char *)"Failed to start ADC");
//	}
	HAL_Delay(10);
	/* Test 12v Main supply */
	if(measure12Voltage() > 11.0)
	{
		self_test_common.self_test_bits_comm.st_12v_b=1;
		usartTx(CONSOLE_USART,(const char *)"12V ADC.............OK\r\n");
	}
	else
	{
		usartTx(CONSOLE_USART,(const char *)"12V ADC.............FAIL\r\n");
	}
	#ifndef PM_V16
	/* Test LiIon 3.3v Battery supply */
	if(measureLiIonVoltage() > 3.0)
	{
			self_test_common.self_test_bits_comm.st_3v_b=1;
			usartTx(CONSOLE_USART,(const char *)"3.3V ADC.............OK\r\n");
	}
	else
	{
			usartTx(CONSOLE_USART,(const char *)"3.3V ADC.............FAIL\r\n");
	}
	#endif
}
#ifdef PONDMOTHER
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)   
{
	int index_ADC;
	for(index_ADC=0;index_ADC<ADC_NUM_OF_CHANNELS;index_ADC++)
	{
		ADCConvertedValue[index_ADC]=ADCConvertedValue_DMA[index_ADC];
	}
}
#endif
static float measure12Voltage(void)
{
	 float volt = 0.0;
	    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel ########################*/
	  #ifndef PM_V16_1
    volt = ((ADCConvertedValue[0] * 2970) / 4096);
	  #else
	  volt = ((ADCConvertedValue[3] * 3000*1.0) / 4096);
	  #endif
    volt *= 6;
    volt /= 1000;
#ifndef PONDRUNNER
    volt += 0.7;
#endif
	usartTx(CONSOLE_USART,(const char *)"12V Reading........%.2f\r\n",volt);
	return volt;
}

static float measureLiIonVoltage(void)
{
	  float volt = 0.0;
    volt = ADCConvertedValue[1];
    volt = ((volt * 2970) / 4096);
    volt *= 2;
    volt /= 1000;
		usartTx(CONSOLE_USART,(const char *)"3.3V Reading........%.2f\r\n",volt);
	  return volt;
}
