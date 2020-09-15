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
#include "stm32l4xx_hal.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LCD_A0_Pin GPIO_PIN_13
#define LCD_A0_GPIO_Port GPIOC
#define WP_LOAD_CURRENT_Pin GPIO_PIN_2
#define WP_LOAD_CURRENT_GPIO_Port GPIOC
#define MOTOR_CURRENT_Pin GPIO_PIN_3
#define MOTOR_CURRENT_GPIO_Port GPIOC
#define VREF_D_Pin GPIO_PIN_0
#define VREF_D_GPIO_Port GPIOA
#define LCD_RESET_Pin GPIO_PIN_1
#define LCD_RESET_GPIO_Port GPIOA
#define SPI_NSS_Pin GPIO_PIN_4
#define SPI_NSS_GPIO_Port GPIOA
#define ADC_BAT_VOLTAGE_Pin GPIO_PIN_4
#define ADC_BAT_VOLTAGE_GPIO_Port GPIOC
#define ADC_SOL_VOLTAGE_Pin GPIO_PIN_5
#define ADC_SOL_VOLTAGE_GPIO_Port GPIOC
#define CHARGE_CURRENT_Pin GPIO_PIN_0
#define CHARGE_CURRENT_GPIO_Port GPIOB
#define CTRL_LOAD_CURRENT_Pin GPIO_PIN_1
#define CTRL_LOAD_CURRENT_GPIO_Port GPIOB
#define DTCT_SW_Pin GPIO_PIN_12
#define DTCT_SW_GPIO_Port GPIOB
#define TRAIL_REQ_Pin GPIO_PIN_13
#define TRAIL_REQ_GPIO_Port GPIOB
#define TRAIL_REQ_EXTI_IRQn EXTI15_10_IRQn
#define BT_NRST_Pin GPIO_PIN_15
#define BT_NRST_GPIO_Port GPIOB
#define SENS_PWR_CTRL_Pin GPIO_PIN_6
#define SENS_PWR_CTRL_GPIO_Port GPIOC
#define LCD_BACKLIGHT_Pin GPIO_PIN_7
#define LCD_BACKLIGHT_GPIO_Port GPIOC
#define MOTOR_F_Pin GPIO_PIN_8
#define MOTOR_F_GPIO_Port GPIOC
#define MOTOR_R_Pin GPIO_PIN_9
#define MOTOR_R_GPIO_Port GPIOC
#define M4_PUSH_DB_Pin GPIO_PIN_11
#define M4_PUSH_DB_GPIO_Port GPIOA
#define M4_PUSH_DB_EXTI_IRQn EXTI15_10_IRQn
#define LORA_PWR_EN_Pin GPIO_PIN_12
#define LORA_PWR_EN_GPIO_Port GPIOA
#define WATER_JET_EN_Pin GPIO_PIN_10
#define WATER_JET_EN_GPIO_Port GPIOC
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
#define GPS_PWR_EN_Pin GPIO_PIN_9
#define GPS_PWR_EN_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define CONSOLE_USART  	(USART_TypeDef *)USART1
#define LORA_USART			(USART_TypeDef *)USART2

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
