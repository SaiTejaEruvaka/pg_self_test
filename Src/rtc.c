/**
  ******************************************************************************
  * File Name          : RTC.c
  * Description        : This file provides code for the configuration
  *                      of the RTC instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#ifdef PONDGUARD
#include "rtc.h"
#include "uart.h"
/* USER CODE BEGIN 0 */
//#include "platform_config.h"
extern void debug_log(char *msg);
void PLATFORM_RTC_Set_Alarm(int seconds);
extern void RTC_Alarm_Handler(void);
RTC_AlarmTypeDef next_sAlarm;
extern uint8_t RTC_AlarmEvntflag;
/* USER CODE END 0 */

void Read_Date(void)
{
    RTC_DateTypeDef sDate;
    sDate.Date = BKP_ReadBackupRegister(BKP_DR4);
    sDate.Month = BKP_ReadBackupRegister(BKP_DR3);
    sDate.Year = BKP_ReadBackupRegister(BKP_DR2);
    if ((sDate.Date == 0) || (sDate.Month == 0) || (sDate.Year == 0))
    {
        usartTx(CONSOLE_USART, "BackUp Reg are not updated\n");
        BKP_WriteBackupRegister(BKP_DR1, 0xA5A7);
    }
    else
    {
        /* Update Date with backup registers value */
//        hrtc.DateToUpdate.Year = sDate.Year;
//        hrtc.DateToUpdate.Month = sDate.Month;
//        hrtc.DateToUpdate.Date = sDate.Date;
					HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
//        /* Get Date based on the days elapsed */
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
        usartTx(CONSOLE_USART, "Days Elapsed %.2d/%.2d/%.2d\n", sDate.Date,
                     sDate.Month, sDate.Year);
    }
    

}

/* USER CODE BEGIN 1 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    RTC_AlarmEvntflag = SET;
}

void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{
    usartTx(CONSOLE_USART,"RTC Tick");
}

void PLATFORM_RTC_Set_Time(uint8_t hours, uint8_t mins, uint8_t secs)
{
    RTC_TimeTypeDef sTime;

    HAL_RTC_WaitForSynchro(&hrtc);
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    usartTx(CONSOLE_USART, "Orig: %.2d:%.2d:%.2d\r\n", sTime.Hours, sTime.Minutes, sTime.Seconds);

    sTime.Hours = hours;
    sTime.Minutes = mins;
    sTime.Seconds = secs;
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);    
	  usartTx(CONSOLE_USART, "New: %.2d:%.2d:%.2d\r\n", sTime.Hours, sTime.Minutes, sTime.Seconds);

    HAL_RTC_WaitForSynchro(&hrtc);
}

void PLATFORM_RTC_Set_Date(uint8_t year, uint8_t month, uint8_t day)
{
    RTC_DateTypeDef DateToUpdate;

    HAL_RTC_WaitForSynchro(&hrtc);
    HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
    usartTx(CONSOLE_USART, "Orig: %d/%d/%d\r\n", DateToUpdate.Date, DateToUpdate.Month, DateToUpdate.Year);
    DateToUpdate.WeekDay = RTC_WEEKDAY_TUESDAY;
    DateToUpdate.Month = month;
    DateToUpdate.Date = day;
    DateToUpdate.Year = year;
    HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
    usartTx(CONSOLE_USART, "New: %d/%d/%d\r\n", DateToUpdate.Date, DateToUpdate.Month, DateToUpdate.Year);
    HAL_RTC_WaitForSynchro(&hrtc);
}
void PLATFORM_RTC_Set_Alarm(int seconds)
{
    RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;
		uint32_t nextAlarm;
	
//    debug_log("Setting Alarm");
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
	  nextAlarm = ((sTime.Hours * 3600) + (sTime.Minutes * 60) + (sTime.Seconds)) + seconds;
    /*
		 * Enable the Alarm A
     */
    next_sAlarm.AlarmTime.Hours = nextAlarm / 3600;
    next_sAlarm.AlarmTime.Minutes = (nextAlarm % 3600) / 60;
    next_sAlarm.AlarmTime.Seconds = (nextAlarm % 3600) % 60;
    if (next_sAlarm.AlarmTime.Seconds > 59)
    {
        next_sAlarm.AlarmTime.Seconds -= 60;
        next_sAlarm.AlarmTime.Minutes++;
    }
    if (next_sAlarm.AlarmTime.Minutes > 59)
    {
        next_sAlarm.AlarmTime.Minutes -= 60;
        next_sAlarm.AlarmTime.Hours++;
    }
    if (next_sAlarm.AlarmTime.Hours > 23)
    {
        // TODO Check if date needs to be set
        // next_sAlarm.AlarmTime.Hours = next_sAlarm.AlarmTime.Hours % 24;
			 /* To avoid day change Schedule Running */
			 next_sAlarm.AlarmTime.Hours = 23;
			 next_sAlarm.AlarmTime.Minutes = 59;
			 next_sAlarm.AlarmTime.Seconds = 00;
    }
    next_sAlarm.Alarm = RTC_ALARM_A;
    HAL_RTC_SetAlarm_IT(&hrtc, &next_sAlarm, RTC_FORMAT_BIN);
}

uint32_t PLATFORM_RTC_get_expiry(void)
{
    RTC_TimeTypeDef sTime;
    RTC_AlarmTypeDef sAlarm;
    int32_t counter;
    HAL_RTC_GetAlarm(&hrtc, &sAlarm, RTC_ALARM_A, RTC_FORMAT_BIN);
    counter = HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    if(counter == HAL_OK) {
	    counter = (sAlarm.AlarmTime.Hours - sTime.Hours) * 3600 + (sAlarm.AlarmTime.Minutes - sTime.Minutes) * 60 + (sAlarm.AlarmTime.Seconds - sTime.Seconds) ;
    }
    else {
       counter = 0;
    }

    //usartprintf(USART1, "Alarm=%d:%d:%d, Time=%d:%d:%d counter =%d\r\n\n", sAlarm.AlarmTime.Hours, sAlarm.AlarmTime.Minutes, sAlarm.AlarmTime.Seconds, sTime.Hours, sTime.Minutes, sTime.Seconds, counter);
    return counter;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#endif
