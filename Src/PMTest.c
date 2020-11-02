/* Copyright (C) Eruvaka Technologies - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * 2018 */

#ifdef PONDMOTHER

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "string.h"
#include "PMTest.h"
#include "rtc.h"
#ifdef PM_V16_1
#define EXTERNAL_RELAY_TIMEOUT_SECS 30
#define MILLIS_PER_SEC 1000
#endif
#define ACS_10AU_Zero_current_Vout 0.33
#define ACS_10AB_Zero_current_Vout 1.655
#define ACS_10AU_sensitivity 264
#define ACS_10AB_sensitivity 132

sGPS_RX_BUFF_t sGPSrxBuff;
sBLE_RX_BUFF_t sBLErxBuff;
const sGPS_NMEA_CMDS_t sGPSDeActCmndsInStandByConfig[] = {"$PMTK161,0*28\r\n","$PMTK001,161,3*36",NULL}; //to mke it standby
const sBLE_NMEA_CMDS_t sBLEInitCmnds[] = {{ "AT\r\n", "OK", NULL }};
const sBLE_NMEA_CMDS_t *sBLECmndBuffer;

static float measureDispCurr(void);
#ifdef PM_V13_RELAY_CHANGES
static float measureDosCurr(void);
#endif
#ifdef PM_V16
static void Cal_dispenser_curr(void)
{
	 float volt = 0;
	 #ifndef PM_V16
	    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel ########################*/
    volt = ((ADCConvertedValue[2] * 2970) / 4096);
    volt *= 6;
    volt /= 1000;
	#else
	HAL_GPIO_WritePin(DISP_MOTOR_EN_INT_GPIO_Port,DISP_MOTOR_EN_INT_Pin,GPIO_PIN_SET);
	HAL_Delay(3000);
//	HAL_ADC_Start(&hadc1);
//	if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADCConvertedValue_DMA,ADC_NUM_OF_CHANNELS) != HAL_OK)
//	{
//			usartTx(CONSOLE_USART,(const char *)"\r\nFailed to start ADC");
//	}
	HAL_Delay(1);
	/* ADC conversion completed */
   /*##-5- Get the converted value of regular channel ########################*/
	#ifdef PM_V16_1
	volt = ((ADCConvertedValue[0] * 3000) / 4095);
	#else
  volt = ((ADCConvertedValue[2] * 3000) / 4095);
	#endif
  volt = volt/DIVIDING_FACTOR_FOR_ACS_CHIP; 
	usartTx(CONSOLE_USART,(const char *)"\r\nDISPENSE MOTOR CURRENT...........%.2f Amps",volt);
  if((volt >= 3) && (volt <= 5))
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nDISPENSE MOTOR...........OK\r\n");
		self_test_PM.self_test_bits_PM.st_disp_b=1;
	}
	else
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nDISPENSE MOTOR...........FAIL\r\n");
	}	
	HAL_GPIO_WritePin(DISP_MOTOR_EN_INT_GPIO_Port,DISP_MOTOR_EN_INT_Pin,GPIO_PIN_RESET);
	#endif
}
static void Cal_doser_curr(void)
{
	
	 float volt = 0;
	 #ifndef PM_V16
	    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel ########################*/
    volt = ((ADCConvertedValue[3] * 2970) / 4096);
    volt *= 6;
    volt /= 1000;
	#else
	/* Forward direction current checking */
	 HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN1_PIN,GPIO_PIN_SET);
	 HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN2_PIN,GPIO_PIN_RESET);
	 HAL_Delay(3000);
//	 HAL_ADC_Start(&hadc1);
//	 if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADCConvertedValue_DMA,ADC_NUM_OF_CHANNELS) != HAL_OK)
//	 {
//	 		usartTx(CONSOLE_USART,(const char *)"Failed to start ADC");
//	 }
	 HAL_Delay(1);
	 /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel ########################*/
    volt = ((ADCConvertedValue[1] * 3000*1.0) / 4095);
    volt = volt/100;
	  volt = volt/0.1;
	usartTx(CONSOLE_USART,(const char *)"\r\nDOSER MOTOR FORWARD DIRECTION CURRENT...........%.2f mA",volt);
	if((volt >= 12) && (volt <= 25))
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nDOSER MOTOR FORWARD DIRECTION...........OK\r\n");
		self_test_PM.self_test_bits_PM.st_doser_b = 1;
	}
	else
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nDOSER MOTOR FORWARD DIRECTION...........FAIL\r\n");
	}
	/* stop doser */
	HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN2_PIN,GPIO_PIN_RESET);
	HAL_Delay(3000);
	/* Reverse direction current checking */
	HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN2_PIN,GPIO_PIN_SET);
	HAL_Delay(3000);
//	HAL_ADC_Start(&hadc1);
//	if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADCConvertedValue_DMA,ADC_NUM_OF_CHANNELS) != HAL_OK)
//	{
//			usartTx(CONSOLE_USART,(const char *)"\r\nFailed to start ADC");
//	}
	HAL_Delay(1);
	/* ADC conversion completed */
   /*##-5- Get the converted value of regular channel ########################*/
   volt = ((ADCConvertedValue[1] * 3000) / 4095);
   volt = volt/100;
	 volt = volt/0.1;
	usartTx(CONSOLE_USART,(const char *)"\r\nDOSER MOTOR REVERSE DIRECTION CURRENT...........%.2f mA",volt);
	if((volt >= 12) && (volt <= 25))
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nDOSER MOTOR REVERSE DIRECTION...........OK\r\n");
	}
	else
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nDOSER MOTOR REVERSE DIRECTION...........FAIL\r\n");
	}
	HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN2_PIN,GPIO_PIN_RESET);
	
	#endif
}
void Log_enable_pin_status()
{
	if(HAL_GPIO_ReadPin(LOG_ENABLE_PORT,LOG_ENABLE_PIN))
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nLOG ENABLE JUMPER.......OPEN\r\n");
	}
	else
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nLOG ENABLE JUMPER.......CLOSED\r\n");
	}
}
#endif
#ifdef PM_V16_1
static void Cal_exti_disp_curr(void)
{
	 float volt = 0;
	HAL_Delay(3000);
//	HAL_ADC_Start(&hadc1);
//	if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADCConvertedValue_DMA,ADC_NUM_OF_CHANNELS) != HAL_OK)
//	{
//			usartTx(CONSOLE_USART,(const char *)"\r\nFailed to start ADC");
//	}
	HAL_Delay(1);
	/* ADC conversion completed */
   /*##-5- Get the converted value of regular channel ########################*/
  volt = ((ADCConvertedValue[2] * 3000) / 4095);
  volt = volt/DIVIDING_FACTOR_FOR_ACS_CHIP; 
	usartTx(CONSOLE_USART,(const char *)"\r\nDISPENSE MOTOR CURRENT...........%.2f Amps",volt);
  if((volt >= 3) && (volt <= 5))
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nRELAY BOARD DISPENSE MOTOR...........OK\r\n");
    self_test_PM.self_test_bits_PM.st_relay_disp_b = 1;
	}
	else
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nRELAY BOARD DISPENSE MOTOR...........FAIL\r\n");
	}	
	HAL_GPIO_WritePin(DISPENSER_MOTOR_ON_OFF_PORT,DISPENSER_MOTOR_ON_OFF_PIN,GPIO_PIN_RESET);
}
static void Cal_exti_doser_curr(void)
{
	
	 float volt = 0;
	/* Forward direction current checking */
	 HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN1_PIN,GPIO_PIN_SET);
	 HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN2_PIN,GPIO_PIN_RESET);
	 HAL_Delay(3000);
//	 HAL_ADC_Start(&hadc1);
//	 if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADCConvertedValue_DMA,ADC_NUM_OF_CHANNELS) != HAL_OK)
//	 {
//	 		usartTx(CONSOLE_USART,(const char *)"Failed to start ADC");
//	 }
	 HAL_Delay(1);
	 /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel ########################*/
    volt = ((ADCConvertedValue[3] * 3000) / 4095);
    volt = volt/20;
	  volt = volt/0.1;
	usartTx(CONSOLE_USART,(const char *)"\r\nRELAY BOARD DOSER MOTOR FORWARD DIRECTION CURRENT...........%.2f mA",volt);
	if((volt >= 60) && (volt <= 250))
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nRELAY BOARD DOSER MOTOR FORWARD DIRECTION...........OK\r\n");
		self_test_PM.self_test_bits_PM.st_relay_doser_b = 1;
	}
	else
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nRELAY BOARD DOSER MOTOR FORWARD DIRECTION...........FAIL\r\n");
	}
	/* stop doser */
	HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN2_PIN,GPIO_PIN_RESET);
	HAL_Delay(3000);
	/* Reverse direction current checking */
	HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN2_PIN,GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_ADC_Start(&hadc1);
//	if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADCConvertedValue_DMA,ADC_NUM_OF_CHANNELS) != HAL_OK)
//	{
//			usartTx(CONSOLE_USART,(const char *)"\r\nFailed to start ADC");
//	}
	HAL_Delay(1);
	/* ADC conversion completed */
   /*##-5- Get the converted value of regular channel ########################*/
   volt = ((ADCConvertedValue[2] * 3000) / 4095);
   volt = volt/20;
	 volt = volt/0.1;
	usartTx(CONSOLE_USART,(const char *)"\r\nRELAY BOARD DOSER MOTOR REVERSE DIRECTION CURRENT...........%.2f mA",volt);
	if((volt >= 60) && (volt <= 250))
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nRELAY BOARD DOSER MOTOR REVERSE DIRECTION...........OK\r\n");
	}
	else
	{
		usartTx(CONSOLE_USART,(const char *)"\r\nRELAY BOARD DOSER MOTOR REVERSE DIRECTION...........FAIL\r\n");
	}
	HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DOSER_MOTOR_IN1_IN2_PORT,DOSER_MOTOR_IN2_PIN,GPIO_PIN_RESET);
	
}
#endif
static float measureDispCurr(void)
{
	float dispenser_current = 0.0, avgValue =0;
  static	float offset = 0;
	uint8_t i;
	static uint16_t sensitivity = 1;
	/* Start ADC */
	for(i = 0;i < 20; i++)
	{
//	HAL_ADC_Start(&hadc1);
//	if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADCConvertedValue_DMA,ADC_NUM_OF_CHANNELS) != HAL_OK)
//	{
//			usartTx(CONSOLE_USART,(const char *)"Failed to start ADC");
//	}
		HAL_Delay(50);
	/* Dispenser current */
	dispenser_current = ((ADCConvertedValue[2] * 3.05) / 4096);
		avgValue += dispenser_current;
	}
	dispenser_current = avgValue/20;
	avgValue = 0;
	#if !defined(BOARD_VERSION_9) && !defined(BOARD_VERSION_11)
	dispenser_current *= 2;
	#endif
	if(READ_MOTOR_RELAY==0)
	{
		if((dispenser_current<0.5)&&(dispenser_current>0))
	{
		offset = ACS_10AU_Zero_current_Vout;
		sensitivity = ACS_10AU_sensitivity;
	  return 1;
	}
	else if ((dispenser_current<1.68)&&(dispenser_current>1.62))
	{
		offset = ACS_10AB_Zero_current_Vout;
		sensitivity = ACS_10AB_sensitivity;
	  return 1;
  }
	else  	
	{
		usartTx(CONSOLE_USART,(const char *)"Relay board is not connected. Please connect it\r\n");
	  return 0;
	}
  }
	else
	{
	dispenser_current -= offset;
	dispenser_current =  (dispenser_current/sensitivity) * 1000;
	usartTx(CONSOLE_USART,(const char *)"Dispenser current.......%.2f\r\n",dispenser_current);
	return dispenser_current;
	}
}

void vGPS_Init(){
	//turn on GPS here 
	
	HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPS_PWR_EN_GPIO_Port, GPS_PWR_EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_UART_Receive_IT(&hlpuart1,sGPSrxBuff.ui8DataBuff,MAX_GPS_RX_BUFFER_SIZE);
	HAL_GPIO_WritePin(GPS_PWR_EN_GPIO_Port, GPS_PWR_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_RESET);
	#ifdef GPS_PWR_ON_RESP_CHK
	HAL_Delay(1000);
	if(strstr((const char *)sGPSrxBuff.ui8DataBuff,"$PMTK010,001*2E")){ 
	#else
	usartTx(CONSOLE_USART,"\r\n GPS ON and waiting for 40 sec");
  HAL_Delay(GPS_ON_TIME * 1000); 	
	HAL_UART_Transmit(&hlpuart1,(uint8_t *)sGPSDeActCmndsInStandByConfig->CmndString,strlen(sGPSDeActCmndsInStandByConfig->CmndString),5000);  	
	HAL_Delay(CMD_RESP_WAIT_TIME * 1000); 
	if(strstr((const char *)sGPSrxBuff.ui8DataBuff,sGPSDeActCmndsInStandByConfig->ResponseString)){ 
	#endif
//		  PG_Cntrltest.self_test_PGcntrl.GPS_comm_b = 1;
			usartTx(CONSOLE_USART,"\r\n GPS............OK");
	}
	else {
		usartTx(CONSOLE_USART,"\r\n GPS............FAIL");
	}
}

void vSendBLECommands(const sBLE_NMEA_CMDS_t *cmndsBuff,uint8_t ui8NoOfCmnds){
	sBLECmndBuffer = cmndsBuff;
//	usartTx(CONSOLE_USART,(const char *)"\r\nBLE Cmd.........%s",(uint8_t *)sBLECmndBuffer->CmndString);
	HAL_UART_Transmit(&huart3,(uint8_t *)sBLECmndBuffer->CmndString,strlen(sBLECmndBuffer->CmndString),1000);
}

void vBLE_Init(){

	HAL_UART_Init(&huart3);
	HAL_UART_Receive_IT(&huart3,sBLErxBuff.ui8DataBuff,MAX_BLE_RX_BUFFER_SIZE);
	HAL_Delay(1000);
	
	HAL_GPIO_WritePin(BT_PWR_EN_GPIO_Port, BT_PWR_EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(BT_PWR_EN_GPIO_Port, BT_PWR_EN_Pin, GPIO_PIN_SET);
	
	usartTx(CONSOLE_USART,"\r\nBLE ON and waiting for 5 sec");
  HAL_Delay(BLE_ON_TIME * 1000); 	
	
	vSendBLECommands(sBLEInitCmnds,sizeof(sBLEInitCmnds)/sizeof(sBLE_NMEA_CMDS_t));
	
	HAL_Delay(CMD_RESP_WAIT_TIME * 1000); 
	sBLErxBuff.ui8DataBuff[0]=0x0A;
	if(strstr((const char *)sBLErxBuff.ui8DataBuff,sBLEInitCmnds->ResponseString)){ 
//		PG_Cntrltest.self_test_PGcntrl.BLE_comm_b = 1;
		//usartTx(CONSOLE_USART,"\r\n if BLE............%s\r\n",(uint8_t *)sBLErxBuff.ui8DataBuff);
		usartTx(CONSOLE_USART,"\r\n BLE............OK");
	}
	else {
		//usartTx(CONSOLE_USART,"\r\n BLE............%s\r\n",(uint8_t *)sBLErxBuff.ui8DataBuff);
		usartTx(CONSOLE_USART,"\r\n BLE............FAIL");
	}
}
void RTC_Chck()
{
	extern uint8_t RTC_AlarmEvntflag;
	usartTx(CONSOLE_USART,"\r\n Initialising RTC");
	PLATFORM_RTC_Set_Alarm(2);
	HAL_Delay(5000);
	if(RTC_AlarmEvntflag == SET)
	{
//		PG_Cntrltest.self_test_PGcntrl.rtc_b = 1;
		usartTx(CONSOLE_USART,"\r\n RTC............OK");
	}
	else {
		usartTx(CONSOLE_USART,"\r\n RTC............FAIL");
	}
}
void Level_Sensor_Check(){
	extern uint8_t recv_buffer[100];
	HAL_GPIO_WritePin(LS_PWR_EN_GPIO_Port,LS_PWR_EN_Pin,GPIO_PIN_SET);
//	if(HAL_GPIO_ReadPin(LOG_ENABLE_PORT,LOG_ENABLE_PIN))
//	{
//		usartTx(CONSOLE_USART,(const char *)"\r\nLOG ENABLE JUMPER.......OPEN\r\n");
//	}
//	else
//		usartTx(CONSOLE_USART,(const char *)"Connect jumper and send 1 within 20 seconds\r\n");
//	
//	uint32_t start_time=HAL_GetTick();
//	while(HAL_GetTick()-start_time<20000){
//		if(HAL_GPIO_ReadPin(LOG_ENABLE_PORT,LOG_ENABLE_PIN)){
//			break;
//		}
//	}
//	if(HAL_GPIO_ReadPin(LOG_ENABLE_PORT,LOG_ENABLE_PIN))
//	{
//		usartTx(CONSOLE_USART,(const char *)"\r\nlevel sensor check fail\r\n");
//	}
//	else{
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_UART_Receive_IT(&huart1,recv_buffer,sizeof(recv_buffer));
		usartTx(CONSOLE_USART,(const char *)"level");
		if(strstr((const char *)recv_buffer,"level")){
			usartTx(CONSOLE_USART,(const char *)"\r\nlevel sensor check pass\r\n");
		}
		else{
			usartTx(CONSOLE_USART,(const char *)"\r\nlevel sensor check fail\r\n");
		}
//	}
	
	
}
void PMTest(void)
{
	#ifndef PM_V16
	float measure_curr = 0.0, is_sensor_connected = 0.0;
	/* Test Motor current */
	is_sensor_connected = measureDispCurr();	/*To check whether current sensor is connected to the device*/	
	if(is_sensor_connected==0) 
	{
		usartTx(CONSOLE_USART,(const char *)"Dispenser Motor.......FAIL\r\n");
		return;
	}
	WRITE_MOTOR_RELAY(1);
	HAL_Delay(3000);
	/* Test Dispenser motor */
	measure_curr = measureDispCurr();
	#ifdef PM_V13_RELAY_CHANGES
	if((measure_curr > 3) && (measure_curr < 5))
	#else
	if((measure_curr > 3) && (measure_curr < 4))
	#endif
	{
		self_test_PM.self_test_bits_PM.st_disp_b=1;
		usartTx(CONSOLE_USART,(const char *)"Dispenser Motor.......OK\r\n");
	}
	else
	{
		usartTx(CONSOLE_USART,(const char *)"Dispenser Motor.......FAIL\r\n");
	}
	WRITE_MOTOR_RELAY(0);
	#else
	uint8_t recv_data_ui8 = RESET;
	uint32_t external_relay_time_millis = RESET;
	vBLE_Init();
	vGPS_Init();
	Cal_doser_curr();
	Cal_dispenser_curr();
	Level_Sensor_Check();

	#ifndef PM_V16_1	
	Log_enable_pin_status();
	#else
//	usartTx(CONSOLE_USART,(const char *)"Connect external relay board and send 1 \r\n");
//	external_relay_time_millis = HAL_GetTick();
//	HAL_GPIO_WritePin(EXTERNAL_RELAY_PORT,EXTERNAL_MOTOR_ENABLE_PIN,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(EXTERNAL_RELAY_PORT,EXTERNAL_MOTOR_DIRECTION_PIN,GPIO_PIN_RESET);
//	while(HAL_GetTick() - external_relay_time_millis <=  EXTERNAL_RELAY_TIMEOUT_SECS * MILLIS_PER_SEC)
//	{
//		HAL_UART_Receive_IT(&huart1,&recv_data_ui8,1);
//		if(recv_data_ui8 == 0x01)
//		{
//			HAL_GPIO_WritePin(EXTERNAL_RELAY_PORT,EXTERNAL_MOTOR_ENABLE_PIN,GPIO_PIN_SET);
//	    HAL_GPIO_WritePin(EXTERNAL_RELAY_PORT,EXTERNAL_MOTOR_DIRECTION_PIN,GPIO_PIN_SET);
//	    Cal_exti_doser_curr();
//			Cal_exti_disp_curr();
//		}
//	}
//	if(HAL_GPIO_ReadPin(EXTERNAL_RELAY_PORT,EXTERNAL_MOTOR_ENABLE_PIN) && HAL_GPIO_ReadPin(EXTERNAL_RELAY_PORT,EXTERNAL_MOTOR_DIRECTION_PIN))
//	{
//		;
//	}
//	else
//	{
//		usartTx(CONSOLE_USART,(const char *)"EXTERNAL RELAY BOARD MEASUREMENTS INCOMPLETE.......\r\n");
//	}
	RTC_Chck();
	#endif
	#endif
}
#endif

