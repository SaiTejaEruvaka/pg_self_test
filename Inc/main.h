/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <string.h>
#include "stm32l4xx_hal.h"
#include "uart.h"
#include "LoRaTest.h"
#include "FlashTest.h"
#include "adc.h"
#include "PMTest.h"
#include "LCDTest.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LCD_A0_Pin GPIO_PIN_13
#define LCD_A0_GPIO_Port GPIOC
#ifdef PONDGUARD
#define WP_LOAD_CURRENT_Pin GPIO_PIN_2
#define WP_LOAD_CURRENT_GPIO_Port GPIOC
#define MOTOR_CURRENT_Pin GPIO_PIN_3
#define MOTOR_CURRENT_GPIO_Port GPIOC

#elif PONDMOTHER
#define ACS_CURRENT_Pin GPIO_PIN_2
#define ACS_CURRENT_GPIO_Port GPIOC
#define DOS_MOTOR_CURRENT_Pin GPIO_PIN_3
#define DOS_MOTOR_CURRENT_GPIO_Port GPIOC
#define BATSENSE_ADC_Pin GPIO_PIN_1
#define BATSENSE_ADC_GPIO_Port GPIOB
#endif

#define VREF_D_Pin GPIO_PIN_0
#define VREF_D_GPIO_Port GPIOA

#define REF3V_GPIO_Pin VREF_D_Pin
#define REF3V_GPIO_GPIO_Port VREF_D_GPIO_Port
#define LCD_RESET_Pin GPIO_PIN_1
#define LCD_RESET_GPIO_Port GPIOA
#define SPI_NSS_Pin GPIO_PIN_4
#define SPI_NSS_GPIO_Port GPIOA
#ifdef PONDGUARD
#define ADC_BAT_VOLTAGE_Pin GPIO_PIN_4
#define ADC_BAT_VOLTAGE_GPIO_Port GPIOC
#define ADC_SOL_VOLTAGE_Pin GPIO_PIN_5
#define ADC_SOL_VOLTAGE_GPIO_Port GPIOC
#define CHARGE_CURRENT_Pin GPIO_PIN_0
#define CHARGE_CURRENT_GPIO_Port GPIOB
#define CTRL_LOAD_CURRENT_Pin GPIO_PIN_1
#define CTRL_LOAD_CURRENT_GPIO_Port GPIOB
#elif PONDMOTHER
#define GPS_PWR_EN_Pin GPIO_PIN_0
#define GPS_PWR_EN_GPIO_Port GPIOB
#define MOTOR_DIR_Pin GPIO_PIN_4
#define MOTOR_DIR_GPIO_Port GPIOC
#define MOTOR_EN_Pin GPIO_PIN_5
#define MOTOR_EN_GPIO_Port GPIOC
#endif
#ifdef PONDGUARD
#define DTCT_SW_Pin GPIO_PIN_12
#define DTCT_SW_GPIO_Port GPIOB
#define TRAIL_REQ_Pin GPIO_PIN_13
#define TRAIL_REQ_GPIO_Port GPIOB
#define TRAIL_REQ_EXTI_IRQn EXTI15_10_IRQn
#define BT_NRST_Pin GPIO_PIN_15
#define BT_NRST_GPIO_Port GPIOB
#define SENS_PWR_CTRL_Pin GPIO_PIN_6
#define SENS_PWR_CTRL_GPIO_Port GPIOC
#define IND_SENSE_Pin GPIO_PIN_8
#define IND_SENSE_GPIO_Port GPIOA
#elif PONDMOTHER
#define BT_PWR_EN_Pin GPIO_PIN_13
#define BT_PWR_EN_GPIO_Port GPIOB
#define DISP_MOTOR_EN_INT_Pin GPIO_PIN_14
#define DISP_MOTOR_EN_INT_GPIO_Port GPIOB
#define LOG_EN_Pin GPIO_PIN_15
#define LOG_EN_GPIO_Port GPIOB
#endif
#define LCD_BACKLIGHT_Pin GPIO_PIN_7
#define LCD_BACKLIGHT_GPIO_Port GPIOC
#ifdef PONDGUARD
#define M4_PUSH_DB_Pin GPIO_PIN_11
#define M4_PUSH_DB_GPIO_Port GPIOA
#define M4_PUSH_DB_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR_F_Pin GPIO_PIN_8
#define MOTOR_F_GPIO_Port GPIOC
#define MOTOR_R_Pin GPIO_PIN_9
#define MOTOR_R_GPIO_Port GPIOC
#elif PONDMOTHER
#define DISP_MOTOR_EN_Pin GPIO_PIN_11
#define DISP_MOTOR_EN_GPIO_Port GPIOA
#define LS_PWR_EN_Pin GPIO_PIN_15
#define LS_PWR_EN_GPIO_Port GPIOA
#define M4_PUSH_DB_Pin GPIO_PIN_6
#define M4_PUSH_DB_GPIO_Port GPIOB
#define M4_PUSH_DB_EXTI_IRQn EXTI9_5_IRQn
#define DOS_MOTOR_IN1_Pin GPIO_PIN_8
#define DOS_MOTOR_IN1_GPIO_Port GPIOC
#define DOS_MOTOR_IN2_Pin GPIO_PIN_9
#define DOS_MOTOR_IN2_GPIO_Port GPIOC
#endif
#ifdef PONDGUARD
#define LORA_PWR_EN_Pin GPIO_PIN_12
#define LORA_PWR_EN_GPIO_Port GPIOA
#define WATER_JET_EN_Pin GPIO_PIN_10
#define WATER_JET_EN_GPIO_Port GPIOC
#endif
#define LORA_RST_Pin GPIO_PIN_12
#define LORA_RST_GPIO_Port GPIOC
#define SPI_Flash_CS_Pin GPIO_PIN_2
#define SPI_Flash_CS_GPIO_Port GPIOD
#define M1_PUSH_DB_Pin GPIO_PIN_3
#define M1_PUSH_DB_GPIO_Port GPIOB
#define M1_PUSH_DB_EXTI_IRQn EXTI3_IRQn
#define M2_PUSH_DB_Pin GPIO_PIN_4
#define M2_PUSH_DB_GPIO_Port GPIOB
#define M2_PUSH_DB_EXTI_IRQn EXTI4_IRQn
#define M3_PUSH_DB_Pin GPIO_PIN_5
#define M3_PUSH_DB_GPIO_Port GPIOB
#define M3_PUSH_DB_EXTI_IRQn EXTI9_5_IRQn
#define GPS_RST_Pin GPIO_PIN_8
#define GPS_RST_GPIO_Port GPIOB
#ifdef PONDGUARD
#define GPS_PWR_EN_Pin GPIO_PIN_9
#define GPS_PWR_EN_GPIO_Port GPIOB
#elif PONDMOTHER
#define LORA_PWR_EN_Pin GPIO_PIN_9
#define LORA_PWR_EN_GPIO_Port GPIOB
#endif


/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#ifdef PONDMOTHER
#define DISPENSER_MOTOR_ON_OFF_PORT DISP_MOTOR_EN_GPIO_Port 
#define DISPENSER_MOTOR_ON_OFF_PIN DISP_MOTOR_EN_Pin
#define DOSER_MOTOR_IN1_IN2_PORT DOS_MOTOR_IN1_GPIO_Port 
#define DOSER_MOTOR_IN1_PIN DOS_MOTOR_IN1_Pin
#define DOSER_MOTOR_IN2_PIN DOS_MOTOR_IN2_Pin
#define LOG_ENABLE_PORT LOG_EN_GPIO_Port
#define LOG_ENABLE_PIN LOG_EN_Pin
#define EXTERNAL_RELAY_PORT DISP_MOTOR_EN_GPIO_Port
#define EXTERNAL_MOTOR_ENABLE_PIN MOTOR_EN_Pin
#define EXTERNAL_MOTOR_DIRECTION_PIN MOTOR_DIR_Pin
#endif

#define CONSOLE_USART  	(USART_TypeDef *)USART1
#define LORA_USART			(USART_TypeDef *)USART2

#ifdef PM_V16
#define SELF_TEST_COMMON_TEST_COUNT		3
#elif PM_V16_1
#define SELF_TEST_COMMON_TEST_COUNT		3
#elif PM_V17
#define SELF_TEST_COMMON_TEST_COUNT		3
#else
#define SELF_TEST_COMMON_TEST_COUNT		4
#endif

#ifdef PM_V16
 #define DIVIDING_FACTOR_FOR_ACS_CHIP 264
#endif
typedef union self_test_comm
{
	uint8_t self_test;
	
	struct self_test_bits_comm_t
	{
		uint8_t st_flash_b:1;
		uint8_t st_lora_b:1;
		uint8_t st_12v_b:1;
		uint8_t st_3v_b:1;
		uint8_t unused:4;
	}self_test_bits_comm;
	
}self_test_common_t;

extern self_test_common_t self_test_common;

#ifdef PONDMOTHER
#ifdef PM_V16_1
#define SELF_TEST_PM_TEST_COUNT		4
#elif PM_V16
#define SELF_TEST_PM_TEST_COUNT		2
#elif PM_V17
#define SELF_TEST_PM_TEST_COUNT		2
#else
#define SELF_TEST_PM_TEST_COUNT		1
#endif

typedef union self_test_PM_u
{
	uint8_t self_test;
	
	struct self_test_bits_PM_t
	{
		uint8_t st_disp_b:1;
		#ifdef PM_V16
		uint8_t st_doser_b:1;
		#ifdef PM_V16_1
		uint8_t st_relay_disp_b:1;
		uint8_t st_relay_doser_b:1;
		uint8_t unused:4;
		#else
		uint8_t unused:6;
		#endif
		#else
		uint8_t unused:7;
		#endif
	}self_test_bits_PM;
	
}self_test_PM_t;

extern self_test_PM_t self_test_PM;
#endif

#ifdef PONDMOTHER
#ifdef BOARD_VERSION_9
	#define READ_MOTOR_RELAY	((HAL_GPIO_ReadPin(MOTOR1_GPIO_Port, MOTOR1_Pin) == 0)?1:0) 
#else
	#define READ_MOTOR_RELAY	((HAL_GPIO_ReadPin(DISP_MOTOR_EN_INT_GPIO_Port, DISP_MOTOR_EN_INT_Pin) == 1)?1:0)
#endif

#ifdef BOARD_VERSION_9
	#define WRITE_MOTOR_RELAY(x)  ((x==0)?HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin,GPIO_PIN_RESET))
#else
	#define WRITE_MOTOR_RELAY(x)  ((x==0)?HAL_GPIO_WritePin(DISP_MOTOR_EN_INT_GPIO_Port, DISP_MOTOR_EN_INT_Pin,GPIO_PIN_RESET):HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin,GPIO_PIN_SET))
#endif
#ifdef PM_V13_RELAY_CHANGES
	#define WRITE_RELAY(x)  ((x==0)?HAL_GPIO_WritePin(RELAY_Port, RELAY_Pin,GPIO_PIN_RESET):HAL_GPIO_WritePin(RELAY_Port, RELAY_Pin,GPIO_PIN_SET))
#endif
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
