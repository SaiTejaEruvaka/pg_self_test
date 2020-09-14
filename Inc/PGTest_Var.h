#ifndef __PGTEST_VAR_H__
#define __PGTEST_VAR_H__
#include "string.h"
uint8_t ADSwrite[6];
uint8_t i2cCommunicationError;
HAL_StatusTypeDef status;
uint16_t adcOut[2][4];
sSELF_CLEAN_t selfClean;
uint8_t PCA9536_Write[2] = {0,0};
uint8_t PCA9536_Read[2] = {0,0};
uint8_t ADC128_Write[4] = {0,0,0,0};
uint8_t ADC128_Read[4] = {0,0,0,0};
uint16_t ADC128_data[8];
uint8_t readADC128(uint8_t);
uint8_t i2c_comm_status_ui8 = RESET;
uint8_t ADC_encoder_voltage_ui8[2]={0,0};
uint16_t Chrg_curr_ui16 = RESET,Load_curr_ui16 = RESET,Solar_Panel_vltg_ui16 = RESET,AdcBatCntrlvltgSum = RESET,fSlfCleanMotorCurrentSum_ui16 = RESET,jetCurrentSum=RESET,Bat_curr_ui16 = RESET;
float Chrgcurravg_fl,Loadcurravg_fl,Panelavg_vltg_fl,AdcBatCntrlvltgAvg_fl,fSlfCleanMotorCurrent_mA,jetCurrent_mA,BatVltg_fl;
const sGPS_NMEA_CMDS_t sGPSDeActCmndsInStandByConfig[] = {"$PMTK161,0*28\r\n","$PMTK001,161,3*36",NULL}; //to mke it standby
sGPS_RX_BUFF_t sGPSrxBuff;
sBLE_RX_BUFF_t sBLErxBuff;
sDO_I2C sDo_I2c_stat;
self_test_PG_Cntrl_t PG_Cntrltest;
self_test_PG_Powr_t PG_Powrtest;
uint8_t RTC_AlarmEvntflag;
#endif
