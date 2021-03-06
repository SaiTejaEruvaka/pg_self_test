
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "LCDTest.h"
#include "FlashTest.h"
#include "LoRaTest.h"
#include "PGTest.h"
#include "uart.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
self_test_common_t self_test_common = {0};
#ifdef PONDMOTHER
self_test_PM_t self_test_PM = {0};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
void MX_SPI1_Init(uint32_t lines);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_LPUART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//char LCD[LCD_MAX_LINES][LINE_SIZE + 1];
extern uint8_t RTC_AlarmEvntflag;


	uint8_t recv_buffer[100];
	uint8_t ble_buffer[100];
	uint8_t lora_buffer[100];
	uint8_t gps_buffer[100];
	uint8_t interupt_pin_press_count[5];
	//I2C
//	uint8_t i2c_dev_addr[8]={1,0,0,1,1,0,1,1};
	uint8_t i2c_data[16];
	HAL_StatusTypeDef ret;
	HAL_StatusTypeDef I2C_MCP3021_Access(uint8_t ui8address,int *ADC_encoder_voltage_ui8){
		HAL_StatusTypeDef status;
		uint8_t ui8mspwrite[] = {0,0};
		ret = HAL_I2C_Master_Receive(&hi2c1,ui8address,ui8mspwrite,2,2);
		if(status == HAL_OK) {
			*ADC_encoder_voltage_ui8 = (((ui8mspwrite[0] & 0x0F) << 6) | ((ui8mspwrite[1] & 0xFC)>>2));
		}
		return status;
	}
	 #define MCP3021_I2C_ADDRESS 0x9E
	int i32Encoder_Value;
HAL_StatusTypeDef i2cStatus;
	uint16_t adc_value[7];
	uint32_t cumm_value[7];
	uint16_t curr_adc_count[7];
	float conv_adc_value[7];
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	uint32_t val=0;
	GPIO_PinState abc;
	uint32_t start_time;
	extern sBLE_RX_BUFF_t sBLErxBuff;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//499 or 100K ohm
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init(SPI_DIRECTION_2LINES);
  MX_RTC_Init();
  #ifdef PONDGUARD
  MX_I2C1_Init();
  #endif
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
//	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port,LCD_RESET_Pin,GPIO_PIN_RESET);
//	HAL_Delay(5000);
//	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port,LCD_RESET_Pin,GPIO_PIN_SET);
  usartTx(CONSOLE_USART,(const char *)"Completed Initialization.......\r\n");
	usartTx(CONSOLE_USART,(const char *)"Enter 'start' to proceed\r\n");
	HAL_UART_Receive_IT(&huart1,recv_buffer,sizeof(recv_buffer));
	while(strstr((const char *)recv_buffer,(const char *)"start")==NULL);

	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_Delay(100);
	usartTx(CONSOLE_USART,(const char *)"Self Testing started.......\r\n");
	
	/*
	HAL_GPIO_WritePin(GPS_PWR_EN_GPIO_Port, GPS_PWR_EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPS_PWR_EN_GPIO_Port, GPS_PWR_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_RESET);
	HAL_UART_Receive_IT(&hlpuart1,gps_buffer,sizeof(gps_buffer));
	
//	HAL_UART_Receive_IT(&huart1,recv_buffer,sizeof(recv_buffer));
	HAL_GPIO_WritePin(BT_NRST_GPIO_Port,BT_NRST_Pin,GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(BT_NRST_GPIO_Port,BT_NRST_Pin,GPIO_PIN_SET);
	HAL_UART_Receive_IT(&huart3,ble_buffer,sizeof(ble_buffer));
	HAL_UART_Transmit(&huart3,(uint8_t*)"AT\r\n",4,1000);
	HAL_Delay(5000);
	HAL_UART_Transmit(&huart3,(uint8_t*)"AT+CGMM\r\n",9,1000);
	
	HAL_GPIO_WritePin(LORA_PWR_EN_GPIO_Port, LORA_PWR_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
	
	HAL_UART_Receive_IT(&huart2,lora_buffer,sizeof(lora_buffer));
	*/
//	HAL_UART_Receive_IT(&huart3,ble_buffer,sizeof(ble_buffer));
////	HAL_UART_Receive_IT(&huart3,sBLErxBuff.ui8DataBuff,MAX_BLE_RX_BUFFER_SIZE);
//	HAL_GPIO_WritePin(BT_PWR_EN_GPIO_Port,BT_PWR_EN_Pin,GPIO_PIN_SET);
//	HAL_Delay(10);
//	HAL_GPIO_WritePin(BT_PWR_EN_GPIO_Port,BT_PWR_EN_Pin,GPIO_PIN_RESET);
//	
//	HAL_Delay(500);
//	HAL_UART_Transmit(&huart3,(uint8_t*)"AT\r\n",4,1000);
//	HAL_Delay(5000);
//	HAL_UART_Transmit(&huart3,(uint8_t*)"AT+CGMM\r\n",9,1000);
//	HAL_Delay(500);
//	HAL_UART_Transmit(&huart3,(uint8_t*)"AT\r\n",4,1000);
	
//	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_value,4);
//	HAL_GPIO_WritePin(SENS_PWR_CTRL_GPIO_Port,SENS_PWR_CTRL_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_BACKLIGHT_GPIO_Port,LCD_BACKLIGHT_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port,LCD_RESET_Pin,GPIO_PIN_RESET);
//	HAL_UART_DeInit(&huart3);
//	i2cStatus = I2C_MCP3021_Access(MCP3021_I2C_ADDRESS,&i32Encoder_Value);
//	ret=HAL_I2C_Master_Receive(&hi2c1,0xAA,i2c_data,sizeof(i2c_data),5000);
//	ret=HAL_I2C_Master_Receive(&hi2c1,0x9F,i2c_data,sizeof(i2c_data),5000);
//	HAL_GPIO_WritePin(WATER_JET_EN_GPIO_Port,WATER_JET_EN_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(MOTOR_F_GPIO_Port,MOTOR_F_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(MOTOR_R_GPIO_Port,MOTOR_R_Pin,GPIO_PIN_RESET);
//	HAL_Delay(5000);
//	HAL_GPIO_WritePin(MOTOR_R_GPIO_Port,MOTOR_R_Pin,GPIO_PIN_SET);
//	HAL_Delay(1000);
//	HAL_GPIO_WritePin(MOTOR_F_GPIO_Port,MOTOR_F_Pin,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(MOTOR_R_GPIO_Port,MOTOR_R_Pin,GPIO_PIN_SET);
//	sTime.Hours=11;
//	sTime.Minutes=53;
//	sTime.Seconds=45;
//	
//	HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
//	val=HAL_RTCEx_BKUPRead(&hrtc,12);
////	FlashTest();
////	LCDTest();
////	LoRaTest();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
	HAL_Delay(10);
//	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port,LCD_RESET_Pin,GPIO_PIN_RESET);
//	HAL_Delay(10);
//	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port,LCD_RESET_Pin,GPIO_PIN_SET);
//	HAL_Delay(10);
//	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_RESET);
//	HAL_Delay(10);
//	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_SET);
//	HAL_Delay(10);
	#ifdef PONDGUARD
	PGTest();
	#elif PONDMOTHER
	FlashTest();
	HAL_GPIO_WritePin(SPI_Flash_CS_GPIO_Port, SPI_Flash_CS_Pin, GPIO_PIN_SET);
	LCDTest();
	LoRaTest();
	ADCTest();
	
	PMTest();
//	vGPS_Init();
	#endif


	start_time=HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//		HAL_UART_Transmit(&huart1,(uint8_t*)"Hello\r\n",8,1000);
//		HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
//		HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
//		HAL_Delay(1000);
		SWTest();
		#ifdef PONDGUARD
			if(HAL_GetTick()-start_time>=1000){
				HAL_GPIO_TogglePin(SENS_PWR_CTRL_GPIO_Port,SENS_PWR_CTRL_Pin);
				start_time=HAL_GetTick();
			}
		#endif
		
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
	#ifdef PONDGUARD
  hadc1.Init.NbrOfConversion = 7;
	#elif PONDMOTHER
	hadc1.Init.NbrOfConversion = 4;
	#endif
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	#ifdef PONDGUARD
    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	#endif
    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_16;
	#ifdef PONDGUARD
  sConfig.Rank = ADC_REGULAR_RANK_7;
	#elif PONDMOTHER
	sConfig.Rank = ADC_REGULAR_RANK_4;
	#endif
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	#ifdef PONDMOTHER
	if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADCConvertedValue_DMA,ADC_NUM_OF_CHANNELS) != HAL_OK)
	{
		usartTx(CONSOLE_USART,(const char *)"Failed to start ADC");
	}
	#elif PONDGUARD
	if (HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_value,7) != HAL_OK)
	{
		usartTx(CONSOLE_USART,(const char *)"Failed to start ADC");
	}
	#endif

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00506682;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* LPUART1 init function */
static void MX_LPUART1_UART_Init(void)
{

  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 11;
  sTime.Minutes = 49;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 3 */

  /* USER CODE END RTC_Init 3 */

  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_SEPTEMBER;
  sDate.Date = 14;
  sDate.Year = 20;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 4 */

  /* USER CODE END RTC_Init 4 */

    /**Enable the Alarm A 
    */
//  sAlarm.AlarmTime.Hours = 11;
//  sAlarm.AlarmTime.Minutes = 49;
//  sAlarm.AlarmTime.Seconds = 10;
//  sAlarm.AlarmTime.SubSeconds = 0;
//  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
//  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
//  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
//  sAlarm.AlarmDateWeekDay = 14;
//  sAlarm.Alarm = RTC_ALARM_A;
//  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
  /* USER CODE BEGIN RTC_Init 5 */

  /* USER CODE END RTC_Init 5 */

}

/* SPI1 init function */
void MX_SPI1_Init(uint32_t lines)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = lines;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
	#ifdef PONDGUARD
	HAL_GPIO_WritePin(GPIOC, LCD_A0_Pin|SENS_PWR_CTRL_Pin|LCD_BACKLIGHT_Pin|MOTOR_F_Pin
                        |MOTOR_R_Pin|WATER_JET_EN_Pin|LORA_RST_Pin, GPIO_PIN_RESET);
	#elif PONDMOTHER
  HAL_GPIO_WritePin(GPIOC, LCD_A0_Pin|LCD_BACKLIGHT_Pin|LORA_RST_Pin|MOTOR_DIR_Pin|MOTOR_EN_Pin
	|DOS_MOTOR_IN1_Pin|DOS_MOTOR_IN2_Pin, GPIO_PIN_RESET);
	#endif
	
	#ifdef PONDGUARD
	HAL_GPIO_WritePin(GPIOB, BT_NRST_Pin, GPIO_PIN_RESET);
	#endif
  /*Configure GPIO pin Output Level */
	#ifdef PONDGUARD
	HAL_GPIO_WritePin(GPIOA, LCD_RESET_Pin|IND_SENSE_Pin|LORA_PWR_EN_Pin, GPIO_PIN_RESET);
	#elif PONDMOTHER
  HAL_GPIO_WritePin(GPIOA, LCD_RESET_Pin|DISP_MOTOR_EN_Pin|LS_PWR_EN_Pin, GPIO_PIN_RESET);
	#endif

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
	#ifdef PONDGUARD
	HAL_GPIO_WritePin(GPIOB, BT_NRST_Pin|GPS_RST_Pin|GPS_PWR_EN_Pin, GPIO_PIN_RESET);
	#elif PONDMOTHER
  HAL_GPIO_WritePin(GPIOB, GPS_RST_Pin|GPS_PWR_EN_Pin|LORA_PWR_EN_Pin|BT_PWR_EN_Pin
	|DISP_MOTOR_EN_INT_Pin, GPIO_PIN_RESET);
	#endif

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_Flash_CS_GPIO_Port, SPI_Flash_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LCD_A0_Pin SENS_PWR_CTRL_Pin LCD_BACKLIGHT_Pin MOTOR_F_Pin 
                           MOTOR_R_Pin WATER_JET_EN_Pin LORA_RST_Pin */
  GPIO_InitStruct.Pin = LCD_A0_Pin|LCD_BACKLIGHT_Pin|LORA_RST_Pin;
	#ifdef PONDGUARD
	GPIO_InitStruct.Pin |=	SENS_PWR_CTRL_Pin|MOTOR_F_Pin 
                          |MOTOR_R_Pin|WATER_JET_EN_Pin;
	#elif PONDMOTHER
	GPIO_InitStruct.Pin |=	MOTOR_DIR_Pin|MOTOR_EN_Pin
	|DOS_MOTOR_IN1_Pin|DOS_MOTOR_IN2_Pin;
	#endif
													
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RESET_Pin IND_SENSE_Pin LORA_PWR_EN_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
	#ifdef PONDGUARD
	GPIO_InitStruct.Pin |=LORA_PWR_EN_Pin|IND_SENSE_Pin;
	#elif PONDMOTHER
	GPIO_InitStruct.Pin |=DISP_MOTOR_EN_Pin|LS_PWR_EN_Pin;
	#endif
	
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_NSS_Pin */
  GPIO_InitStruct.Pin = SPI_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DTCT_SW_Pin */
	#ifdef PONDGUARD
  GPIO_InitStruct.Pin = DTCT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DTCT_SW_GPIO_Port, &GPIO_InitStruct);
	#elif PONDMOTHER
	GPIO_InitStruct.Pin = LOG_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LOG_EN_GPIO_Port, &GPIO_InitStruct);
	#endif

  /*Configure GPIO pins : TRAIL_REQ_Pin M1_PUSH_DB_Pin M2_PUSH_DB_Pin M3_PUSH_DB_Pin */
  GPIO_InitStruct.Pin = M1_PUSH_DB_Pin|M2_PUSH_DB_Pin|M3_PUSH_DB_Pin;
	#ifdef PONDGUARD
	GPIO_InitStruct.Pin |= TRAIL_REQ_Pin;
	#endif
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_NRST_Pin GPS_RST_Pin GPS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = GPS_RST_Pin|GPS_PWR_EN_Pin;
	#ifdef PONDGUARD
	GPIO_InitStruct.Pin |= BT_NRST_Pin;
	#elif PONDMOTHER
	GPIO_InitStruct.Pin |= LORA_PWR_EN_Pin|BT_PWR_EN_Pin
	|DISP_MOTOR_EN_INT_Pin;
	#endif
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : M4_PUSH_DB_Pin */
  GPIO_InitStruct.Pin = M4_PUSH_DB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M4_PUSH_DB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_Flash_CS_Pin */
  GPIO_InitStruct.Pin = SPI_Flash_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_Flash_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart==&huart1){
//		HAL_UART_Receive_IT(&huart1,recv_buffer,sizeof(recv_buffer));
//	}
//	else if(huart==&huart3){
//		HAL_UART_Receive_IT(&huart3,ble_buffer,sizeof(ble_buffer));
//	}
//	else if(huart==&huart2){
//		HAL_UART_Receive_IT(&huart2,lora_buffer,sizeof(lora_buffer));
//	}
//}
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    RTC_AlarmEvntflag = SET;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==M1_PUSH_DB_Pin){
		interupt_pin_press_count[0]++;
		#ifdef PONDGUARD
		lcd_click_status = TRAIL_RUN;//UP_CLICK_EVENT;
		#elif PONDMOTHER
		lcd_click_status = MENU_CLICK_EVENT;
		#endif
		LCDClickevent = 1;
	}
	else if(GPIO_Pin==M2_PUSH_DB_Pin){
		interupt_pin_press_count[1]++;
		#ifdef PONDGUARD
		lcd_click_status = BACK_CLICK_EVENT;//DOWN_CLICK_EVENT;
		#elif PONDMOTHER
		lcd_click_status = UP_CLICK_EVENT;
		#endif
		LCDClickevent = 1;
	}
	else if(GPIO_Pin==M3_PUSH_DB_Pin){
		interupt_pin_press_count[2]++;
		#ifdef PONDGUARD
		lcd_click_status = UP_CLICK_EVENT;//DOWN_CLICK_EVENT;
		#elif PONDMOTHER
		lcd_click_status = BACK_CLICK_EVENT;
		#endif
		LCDClickevent = 1;
	}
	else if(GPIO_Pin==M4_PUSH_DB_Pin){
		interupt_pin_press_count[3]++;
		lcd_click_status = DOWN_CLICK_EVENT;
		LCDClickevent = 1;
	}
	#ifdef PONDGUARD
	else if(GPIO_Pin==TRAIL_REQ_Pin){
		interupt_pin_press_count[4]++;
		lcd_click_status = MENU_CLICK_EVENT;
		LCDClickevent = 1;
	}
	#endif
	else{
		LCDClickevent = 0;
	}
	
}
uint16_t max=0;
#ifdef PONDGUARD
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//INA Vo=(Is*Rs*Rl)/5K
	static uint16_t sample_count=0;
  if(hadc==&hadc1){
//		for(int i=0;i<7;i++){
//			if(i==0){
//				if(adc_value[0]>max){
//					max=adc_value[0];
//				}
//			}
//			conv_adc_value[i]=(adc_value[i]*0.00073);//(3/4095))/264
//		}
		for(int i=0;i<7;i++){
			cumm_value[i]+=adc_value[i];
			if(sample_count==100){
				curr_adc_count[i]=cumm_value[i]/100;
				cumm_value[i]=0;
				if(i==6)
					sample_count=0;
			}
		}
		sample_count++;
		
	}
}
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
