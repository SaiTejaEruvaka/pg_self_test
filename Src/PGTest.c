#include "PGTest.h"
#include "PGTest_Var.h"
#include "math.h"
//#include "rtc.h"
#include "uart.h"
#include "main.h"
#include "LCDTest.h"
#include "FlashTest.h"
#include "LoRaTest.h"
#include "rtc.h"
extern void MX_USART3_UART_Init(int);

extern char LCD[LCD_MAX_LINES][LINE_SIZE + 1];
uint32_t TempLCDUpdTimout = RESET;
uint8_t ui8CmndsToBeSent;
uint8_t ui8BleRetryCmndCnt;
const sBLE_NMEA_CMDS_t *sBLECmndBuffer;

UART_HandleTypeDef * huartPtr;


#ifdef PONDGUARD
extern I2C_HandleTypeDef hi2c1;

const sBLE_NMEA_CMDS_t sBLEInitCmnds[] = {{ "AT\r\n", "OK", NULL }};
																					/*{ "ATE0\r\n", "OK", NULL},						//Disable Echo
																					{ "AT+CGMI\r\n", "OK", NULL },				//Identifies the manufacturer
																					{ "AT+CGMM\r\n", "OK", NULL }};				//Identifies the mode
																					*/

void readADSSensor(void) {
	HAL_GPIO_WritePin(ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT, ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN,GPIO_PIN_SET);
	HAL_Delay(10);
	int id,i;
	for(id = 0;id<1;id++){ /*I2C address 0x48 */
		for(i = 0; i < 4; i++) {
			ADSwrite[0] = 0x01;
			ADSwrite[1] = 0xC3 | i<<4;
			ADSwrite[2] = 0x83; // 10000011
			status = HAL_I2C_Master_Transmit(&hi2c1, (ADS1015_ADDRESS+id)<<1, ADSwrite, 3, 50);
			if(status != HAL_OK) {
				usartTx(CONSOLE_USART, "\r\n ADC128i2c comm failure\r\n");
				return;
			}
			else {
				i2cCommunicationError = 0;
			}
			ADSwrite[0] = 0x00;
			HAL_I2C_Master_Transmit(&hi2c1, (ADS1015_ADDRESS+id)<<1, ADSwrite, 1, 50);
			if(status != HAL_OK) {
				usartTx(CONSOLE_USART, "\r\n ADC128i2c comm failure\r\n");
				return;
			}
			else {
				i2cCommunicationError = 0;
			}
			HAL_Delay(5);
			status = HAL_I2C_Master_Receive(&hi2c1, (ADS1015_ADDRESS+id)<<1, ADSwrite, 2, 50);
			if(status != HAL_OK) {
				usartTx(CONSOLE_USART, "\r\n ADC128i2c comm failure\r\n");
				return;
			}
			else {
				i2cCommunicationError = 0;
			}
			adcOut[id][i] = (ADSwrite[0] << 8 | ADSwrite[1]);
			selfClean.rawADC.data[id][i] = ((adcOut[id][i] >> 4)&0xfff);
			usartTx(CONSOLE_USART, "\r\n ADC128 Channel Num: %d,Data:%d\r\n",i,selfClean.rawADC.data[id][i]);
		}
	}
}
void ModifdreadADSSensor(void) {
	HAL_GPIO_WritePin(ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT, ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN,GPIO_PIN_SET);
	HAL_Delay(10);
	int id,i;
	for(id = 0;id<1;id++){ /*I2C address 0x48 */
		for(i = 0; i < 4; i++) {
			ADSwrite[0] = 0x01;
			if((i == 0) || (i == 3)) {
			ADSwrite[1] = 0xC5 | i<<4;
			ADSwrite[2] = 0x83; // 10000011
			} else if(i == 1) {
			ADSwrite[1] = 0xA9;
			ADSwrite[2] = 0x83; // 10000011
			}if(i == 2) {
			ADSwrite[1] = 0xB9;
			ADSwrite[2] = 0x83; // 10000011
			}
			status = HAL_I2C_Master_Transmit(&hi2c1, (ADS1015_ADDRESS+id)<<1, ADSwrite, 3, 50);
			if(status != HAL_OK) {
				usartTx(CONSOLE_USART, "\r\n ADC128i2c comm failure\r\n");
				return;
			}
			else {
				i2cCommunicationError = 0;
			}
			ADSwrite[0] = 0x00;
			HAL_I2C_Master_Transmit(&hi2c1, (ADS1015_ADDRESS+id)<<1, ADSwrite, 1, 50);
			if(status != HAL_OK) {
				usartTx(CONSOLE_USART, "\r\n ADC128i2c comm failure\r\n");
				return;
			}
			else {
				i2cCommunicationError = 0;
			}
			HAL_Delay(5);
			status = HAL_I2C_Master_Receive(&hi2c1, (ADS1015_ADDRESS+id)<<1, ADSwrite, 2, 50);
			if(status != HAL_OK) {
				usartTx(CONSOLE_USART, "\r\n ADC128i2c comm failure\r\n");
				return;
			}
			else {
				i2cCommunicationError = 0;
			}
			adcOut[id][i] = (ADSwrite[0] << 8 | ADSwrite[1]);
			selfClean.rawADC.data[id][i] = ((adcOut[id][i] >> 4)&0xfff);
			usartTx(CONSOLE_USART, "\r\n ADC128 Channel Num: %d,Modified Data:%.2f\r\n",i,(selfClean.rawADC.data[id][i] * 0.25));
		}
	}
}
float GetPlatinumRTD(float R, float R0)
{
    float A = 3.9083E-3;
    float B = -5.775E-7;
    float T;
    R = R / R0;
    // T = (0.0-A + sqrt((A*A) - 4.0 * B * (1.0 - R))) / 2.0 * B;
    T = 0.0 - A;
    T += sqrt((A * A) - 4.0 * B * (1.0 - R));
    T /= (2.0 * B);
    return T;
}
float getTemperatureValueFromADS() {
	float Rtemp ;
	float SourceVolt;
	float ReceivedVoltage;
	float temp;
	ReceivedVoltage = selfClean.rawADC.data[0][3]; /* ADS sensor result will be half of actual and in milli volts */
	SourceVolt = (selfClean.rawADC.data[0][1] - selfClean.rawADC.data[0][2]);
	SourceVolt = (( SourceVolt *24.7)/4.7);   /* Resistor divider calculation */
	Rtemp = 1000 * ((SourceVolt/ReceivedVoltage)-1U); 
	temp = GetPlatinumRTD(Rtemp,1000);
	return temp;
}
float ModgetTemperatureValueFromADS() {
	float Rtemp ;
	float SourceVolt;
	float ReceivedVoltage;
	float temp;
	SourceVolt = (selfClean.rawADC.data[0][1] - (selfClean.rawADC.data[0][2] - 0xFFF)) *0.25;
	SourceVolt = (( SourceVolt *24.7)/4.7);   /* Resistor divider calculation */
	ReceivedVoltage = ((SourceVolt - ((selfClean.rawADC.data[0][1] + (selfClean.rawADC.data[0][2] - 0xFFF)) * 0.25))/2); /* ADS sensor result will be half of actual and in milli volts */
	Rtemp = 1000 * ((SourceVolt/ReceivedVoltage)-1U); 
	temp = GetPlatinumRTD(Rtemp,1000);
	return temp;
}
uint8_t vInit_ADC128()
{
	ADC128_Write[0] = 0x3E; /* Manfacurer ID */
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 1, 2);
	usartTx(CONSOLE_USART,"\r\n Initialising ADC128");
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		usartTx(CONSOLE_USART, "\r\n ADC128 communication fail i2c status: %d\r\n",i2c_comm_status_ui8);
		return i2c_comm_status_ui8;
	}
	i2c_comm_status_ui8 = HAL_I2C_Master_Receive(&hi2c1, (ADC128_ADDRESS), &ADC128_Read[0], 1, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		usartTx(CONSOLE_USART, "\r\n ADC128 communication fail i2c status: %d\r\n",i2c_comm_status_ui8);
		return i2c_comm_status_ui8;
	}
	ADC128_Write[0] = 0x3F; /* Device ID*/
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 1, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
    usartTx(CONSOLE_USART, "\r\n ADC128 communication fail i2c status: %d\r\n",i2c_comm_status_ui8);
		return i2c_comm_status_ui8;
	}
	i2c_comm_status_ui8 = HAL_I2C_Master_Receive(&hi2c1, (ADC128_ADDRESS), &ADC128_Read[1], 1, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		usartTx(CONSOLE_USART, "\r\n ADC128 communication fail i2c status: %d\r\n",i2c_comm_status_ui8);
		return i2c_comm_status_ui8;
	}
	if( !((ADC128_Read[0] == 0x01) &&(ADC128_Read[1] == 0x09))  )
	{
		usartTx(CONSOLE_USART, "\r\n ADC128 communication fail i2c status: %d\r\n",i2c_comm_status_ui8);
    return HAL_ERROR;
	}
	ADC128_Write[0] = 0x00; /* Configuration register */
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 1, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
	  usartTx(CONSOLE_USART, "\r\n ADC128 communication fail i2c status: %d\r\n",i2c_comm_status_ui8);
		return i2c_comm_status_ui8;
	}
	i2c_comm_status_ui8 = HAL_I2C_Master_Receive(&hi2c1, (ADC128_ADDRESS), ADC128_Read, 1, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		usartTx(CONSOLE_USART, "\r\n ADC128 communication fail i2c status: %d\r\n",i2c_comm_status_ui8);
		return i2c_comm_status_ui8;
	}
	ADC128_Write[0] = 0x0C; /* Busy status register */
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 1, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		usartTx(CONSOLE_USART, "\r\n ADC128 communication fail i2c status: %d\r\n",i2c_comm_status_ui8);
		return i2c_comm_status_ui8;
	}
	i2c_comm_status_ui8 = HAL_I2C_Master_Receive(&hi2c1, (ADC128_ADDRESS), ADC128_Read , 1, 3);
	if(i2c_comm_status_ui8 != HAL_OK)
	{
		usartTx(CONSOLE_USART, "\r\n ADC128 communication fail i2c status: %d\r\n",i2c_comm_status_ui8);
		return i2c_comm_status_ui8;
	}
	if(ADC128_Read[0] & (1 << NOT_READY_BIT_POSITION))
	{
		HAL_Delay(50);
	}
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 2, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	i2c_comm_status_ui8 = HAL_I2C_Master_Receive(&hi2c1, (ADC128_ADDRESS), ADC128_Read , 1, 3);
	if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	if(ADC128_Read[0] & (1 << NOT_READY_BIT_POSITION))
	{
		return i2c_comm_status_ui8;
	}
	ADC128_Write[0] = 0x0B; /* Advanced Configuration register */
	ADC128_Write[1] = 0x02; /* Set mode 1*/
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 2, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	ADC128_Write[0] = 0x07; /* Conversion Rate register */
	ADC128_Write[1] = 0x01; /* Set continous version */
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 2, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	ADC128_Write[0] = 0x08; /* Channel Disable register */
	ADC128_Write[1] = 0x00; /* Enable all channels */
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 2, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	ADC128_Write[0] = 0x00; /* Configuration register */
	ADC128_Write[1] = 0x01; /* Set start bit */
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 2, 2);
	return i2c_comm_status_ui8;
}
uint8_t SetADC18_ConvChanls(uint8_t channels)
{
	ADC128_Write[0] = 0x00; /* Configuration register */
	ADC128_Write[1] = 0x00; /* Reset start bit */
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 2, 2);
	if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	ADC128_Write[0] = 0x08; /* Channel Disable register */
	ADC128_Write[1] = (~channels); 
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 2, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	ADC128_Write[0] = 0x00; /* Configuration register */
	ADC128_Write[1] = 0x01; /* Set start bit */
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 2, 2);
	return i2c_comm_status_ui8;
}
uint8_t vInit_ADC_channels()
{
	uint8_t i2c_status=RESET;
	i2c_status = vInit_ADC128();
	if(i2c_status != HAL_OK)
   {
		return i2c_status;
   }
	 else
	 {
		 usartTx(CONSOLE_USART, "\r\n ADC128 communication success");
	 }
	 return i2c_status;
}
uint8_t readADC128(uint8_t channel_num)
{
	ADC128_Write[0] = 0x3E; /* Manfacturer ID */
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 1, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	i2c_comm_status_ui8 = HAL_I2C_Master_Receive(&hi2c1, (ADC128_ADDRESS), &ADC128_Read[0], 1, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	ADC128_Write[0] = 0x3F; /* Device ID*/
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 1, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	i2c_comm_status_ui8 = HAL_I2C_Master_Receive(&hi2c1, (ADC128_ADDRESS), &ADC128_Read[1], 1, 2);
  if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	if( !((ADC128_Read[0] == 0x01) &&(ADC128_Read[1] == 0x09))  )
	{
    return i2c_comm_status_ui8;
	}
	ADC128_Write[0] = ADC128_CHANNEL_REGISTERS_OFFSET + channel_num; /* Conversion Rate register */
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, (ADC128_ADDRESS), ADC128_Write, 1, 2);
	if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	i2c_comm_status_ui8 = HAL_I2C_Master_Receive(&hi2c1, (ADC128_ADDRESS), ADC128_Read , 2, 3);
	ADC128_data[channel_num] = ( ADC128_Read[0] << 8 | ADC128_Read[1] );  
	ADC128_data[channel_num] = ADC128_data[channel_num] >> 4;
	return i2c_comm_status_ui8;
}
void Chck_Powr_PCB()
{
	uint8_t j=RESET,i2c_status = RESET;
	usartTx(CONSOLE_USART,"\r\n\r\n\r\n\r\n\r\n Power supply PCB testing started");
	HAL_GPIO_WritePin(ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT, ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN,GPIO_PIN_SET);
	HAL_Delay(2);
	i2c_status = vInit_ADC_channels();
	if(i2c_status == HAL_OK)
	{
		PG_Cntrltest.self_test_PGcntrl.ADC128_comm_b = 1;
		SetADC18_ConvChanls((( 1 << ADC128_SOLAR_VOLTAGE_INDEX) | ( 1 << ADC_BAT_CNTRL_VLTG_INDEX)) | ( 1 << ADC128_CHARGE_CURRENT_INDEX) | ( 1 << ADC128_LOAD_CURRENT_INDEX) | ( 1 << BATTERY_VOLTAGE_INDEX));
		for(j=0;j< ADC128_SAMPLES;j++)
		{
			HAL_Delay(13);
			readADC128(ADC128_SOLAR_VOLTAGE_INDEX);
			HAL_Delay(13);
			readADC128(ADC_BAT_CNTRL_VLTG_INDEX);
			HAL_Delay(13);
			readADC128(ADC128_CHARGE_CURRENT_INDEX);
			HAL_Delay(13);
			readADC128(ADC128_LOAD_CURRENT_INDEX);
			HAL_Delay(13);
			readADC128(BATTERY_VOLTAGE_INDEX);
			Solar_Panel_vltg_ui16 += ADC128_data[ADC128_SOLAR_VOLTAGE_INDEX];
			AdcBatCntrlvltgSum += ADC128_data[ADC_BAT_CNTRL_VLTG_INDEX];	
			Chrg_curr_ui16 += ADC128_data[ADC128_CHARGE_CURRENT_INDEX];
			Load_curr_ui16 += ADC128_data[ADC128_LOAD_CURRENT_INDEX];
			Bat_curr_ui16 += ADC128_data[BATTERY_VOLTAGE_INDEX];
		}
		Chrgcurravg_fl = Chrg_curr_ui16 / ADC128_SAMPLES;
		Chrgcurravg_fl = (float)(Chrgcurravg_fl*ADC_REF_VOLTAGE_ADC128_MILLIS)/MAX_ADC_COUNT_VALUE;
		Loadcurravg_fl = Load_curr_ui16 / ADC128_SAMPLES;
		Loadcurravg_fl = (Loadcurravg_fl*ADC_REF_VOLTAGE_ADC128_MILLIS)/MAX_ADC_COUNT_VALUE;
		Loadcurravg_fl = (Loadcurravg_fl/CHRG_CURR_OP_AMP_GAIN);
		Loadcurravg_fl = (Loadcurravg_fl/CHRG_CURRENT_LOAD_RESISTOR);
		Panelavg_vltg_fl = Solar_Panel_vltg_ui16 / ADC128_SAMPLES;
		Panelavg_vltg_fl = (Panelavg_vltg_fl*ADC_REF_VOLTAGE_ADC128_MILLIS)/MAX_ADC_COUNT_VALUE;
		Panelavg_vltg_fl = Panelavg_vltg_fl/1000;
		Panelavg_vltg_fl = (Panelavg_vltg_fl *(SOLAR_PANEL_RESISTANCE1_KOHM+SOLAR_PANEL_RESISTANCE2_KOHM))/SOLAR_PANEL_RESISTANCE2_KOHM;
		AdcBatCntrlvltgAvg_fl = AdcBatCntrlvltgSum / ADC128_SAMPLES;
		AdcBatCntrlvltgAvg_fl = (AdcBatCntrlvltgAvg_fl*ADC_REF_VOLTAGE_ADC128_MILLIS)/MAX_ADC_COUNT_VALUE;
		AdcBatCntrlvltgAvg_fl = AdcBatCntrlvltgAvg_fl/1000;
		AdcBatCntrlvltgAvg_fl = (AdcBatCntrlvltgAvg_fl *(ADC_BAT_RESISTANCE1_KOHM+ADC_BAT_RESISTANCE2_KOHM))/ADC_BAT_RESISTANCE2_KOHM;
		AdcBatCntrlvltgAvg_fl = AdcBatCntrlvltgAvg_fl + 1;
		BatVltg_fl = Bat_curr_ui16/ADC128_SAMPLES;
		BatVltg_fl = (BatVltg_fl*ADC_REF_VOLTAGE_ADC128_MILLIS)/MAX_ADC_COUNT_VALUE;
		BatVltg_fl = BatVltg_fl/1000;
		BatVltg_fl = (BatVltg_fl *(ADC_BAT_RESISTANCE1_KOHM+ADC_BAT_RESISTANCE2_KOHM))/ADC_BAT_RESISTANCE2_KOHM;
		BatVltg_fl = BatVltg_fl + 1;
		Panelavg_vltg_fl = Panelavg_vltg_fl +1; /* Diode drop added 1V */
		usartTx(CONSOLE_USART,"\r\n\r\n ADC Bat Control voltage: %.2f V",AdcBatCntrlvltgAvg_fl);	
		if(AdcBatCntrlvltgAvg_fl >= 11 && AdcBatCntrlvltgAvg_fl <=13)
		{
			usartTx(CONSOLE_USART,"\r\n ADC BAT CONTROL VOLTAGE..........OK");	
			PG_Powrtest.self_test_PGcntrl.ADCBat_vltg_b = 1;
		}
		else
		{
			usartTx(CONSOLE_USART,"\r\n ADC BAT CONTROL VOLTAGE..........FAIL");	
		}
		#ifdef SOLAR_PANEL_VOLTAGE
		usartTx(CONSOLE_USART,"\r\n\r\n Solar Panel voltage:  %.2f V",Panelavg_vltg_fl);	
		if(Panelavg_vltg_fl >= 11 && Panelavg_vltg_fl <=13)
		{
			PG_Powrtest.self_test_PGcntrl.Solar_vltg_b = 1;
			usartTx(CONSOLE_USART,"\r\n SOLAR PANEL VOLTAGE..........OK");	
		}
		else
		{
			usartTx(CONSOLE_USART,"\r\n SOLAR PANEL VOLTAGE..........FAIL");	
		}
		#else
		PG_Powrtest.self_test_PGcntrl.Solar_vltg_b = 1;
		#endif
		usartTx(CONSOLE_USART,"\r\n\r\n Battery voltage: %.2f V",BatVltg_fl);	
		if(BatVltg_fl >= 11 && BatVltg_fl <=14)
		{
			PG_Powrtest.self_test_PGcntrl.Battery_vltg_b = 1;
			usartTx(CONSOLE_USART,"\r\n BATTERY VOLTAGE..........OK");	
		}
		else
		{
			usartTx(CONSOLE_USART,"\r\n BATTERY VOLTAGE..........FAIL");	
		}
		usartTx(CONSOLE_USART,"\r\n\r\n Load current: %.2f mA",Loadcurravg_fl);	
		if(Loadcurravg_fl >= 10 && Loadcurravg_fl <=60)
		{
			PG_Powrtest.self_test_PGcntrl.LoadCurr_b = 1;
			usartTx(CONSOLE_USART,"\r\n LOAD CURRENT..........OK");	
		}
		else
		{
			usartTx(CONSOLE_USART,"\r\n LOAD CURRENT..........FAIL");	
		}
		SetADC18_ConvChanls(( 1 << POSITION_MOTOR_CURRENT_INDEX));
		HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_FWD_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_REV_PIN,GPIO_PIN_RESET);
		usartTx(CONSOLE_USART,"\r\n\r\n Position motor turned ON forward direction, waiting for 2 sec");
		HAL_Delay(2000);
		for(j=0;j< ADC128_SAMPLES;j++)
		{
			readADC128(POSITION_MOTOR_CURRENT_INDEX);
			fSlfCleanMotorCurrentSum_ui16 += ADC128_data[POSITION_MOTOR_CURRENT_INDEX];	
			HAL_Delay(13);
		}
		fSlfCleanMotorCurrent_mA = fSlfCleanMotorCurrentSum_ui16 / ADC128_SAMPLES;
		fSlfCleanMotorCurrent_mA = (fSlfCleanMotorCurrent_mA *ADC_REF_VOLTAGE_ADC128_MILLIS)/MAX_ADC_COUNT_VALUE;
		fSlfCleanMotorCurrent_mA = fSlfCleanMotorCurrent_mA/POS_MOTOR_CURR_OP_AMP_GAIN;
		fSlfCleanMotorCurrent_mA = fSlfCleanMotorCurrent_mA/CHRG_CURRENT_LOAD_RESISTOR;
		usartTx(CONSOLE_USART,"\r\n\r\n Position motor forward current...........%.2f mA",fSlfCleanMotorCurrent_mA);	
		if(fSlfCleanMotorCurrent_mA >= 8.5 && fSlfCleanMotorCurrent_mA <= 30) 
		{
			PG_Powrtest.self_test_PGcntrl.pos_motor_for_b = 1;
			usartTx(CONSOLE_USART,"\r\n POSITION MOTOR FORWARD DIRECTION...........OK ");	
		}
		else
		{
			usartTx(CONSOLE_USART,"\r\n POSITION MOTOR FORWARD DIRECTION...........FAIL ");	
		}
		fSlfCleanMotorCurrentSum_ui16 = RESET;
		HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_FWD_PIN,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_REV_PIN,GPIO_PIN_SET);
		usartTx(CONSOLE_USART,"\r\n\r\n Position motor turned ON reverse direction, waiting for 2 sec");
		HAL_Delay(2000);
		for(j=0;j< ADC128_SAMPLES;j++)
		{
			readADC128(POSITION_MOTOR_CURRENT_INDEX);
			fSlfCleanMotorCurrentSum_ui16 += ADC128_data[POSITION_MOTOR_CURRENT_INDEX];	
			HAL_Delay(13);
		}
		fSlfCleanMotorCurrent_mA = fSlfCleanMotorCurrentSum_ui16 / ADC128_SAMPLES;
		fSlfCleanMotorCurrent_mA = (fSlfCleanMotorCurrent_mA *ADC_REF_VOLTAGE_ADC128_MILLIS)/MAX_ADC_COUNT_VALUE;
		fSlfCleanMotorCurrent_mA = fSlfCleanMotorCurrent_mA/POS_MOTOR_CURR_OP_AMP_GAIN;
		fSlfCleanMotorCurrent_mA = fSlfCleanMotorCurrent_mA/CHRG_CURRENT_LOAD_RESISTOR;
		usartTx(CONSOLE_USART,"\r\n\r\n Position motor reverse current...........%.2f mA",fSlfCleanMotorCurrent_mA);	
		if(fSlfCleanMotorCurrent_mA >= 8.5 && fSlfCleanMotorCurrent_mA <= 30) 
		{
			PG_Powrtest.self_test_PGcntrl.pos_motor_rev_b = 1;
			usartTx(CONSOLE_USART,"\r\n POSITION MOTOR REVERSE DIRECTION...........OK ");	
		}
		else
		{
			usartTx(CONSOLE_USART,"\r\n POSITION MOTOR REVERSE DIRECTION...........FAIL ");	
		}
		HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_FWD_PIN,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_REV_PIN,GPIO_PIN_RESET);
		SetADC18_ConvChanls(( 1 << WATER_JET_CURRENT));
		usartTx(CONSOLE_USART,"\r\n\r\n Water jet turned ON reverse direction, waiting for 2 sec");
		HAL_GPIO_WritePin(WATER_JET_MOTOR_PORT, WATER_JET_MOTOR_PIN,GPIO_PIN_SET);
		HAL_Delay(2000);
		for(j=0;j< ADC128_SAMPLES;j++)
		{
			readADC128(WATER_JET_CURRENT);
			jetCurrentSum += ADC128_data[WATER_JET_CURRENT];	
			HAL_Delay(13);
		}
		jetCurrentSum = jetCurrentSum /ADC128_SAMPLES; 
		jetCurrent_mA = jetCurrentSum; 
		jetCurrent_mA = (jetCurrent_mA *ADC_REF_VOLTAGE_ADC128_MILLIS)/MAX_ADC_COUNT_VALUE;
		jetCurrent_mA = jetCurrent_mA/260;
		usartTx(CONSOLE_USART,"\r\n\r\n Water jet motor current...........%.2f A",jetCurrent_mA);	
		if(jetCurrentSum >= WATER_JET_OFFSET_COUNT && jetCurrentSum <= WATER_JET_NO_LOAD_COUNT)   /* 1.5 to 2 Amps */ 
		{
			PG_Powrtest.self_test_PGcntrl.water_jet_motor = 1;
			usartTx(CONSOLE_USART,"\r\n WATER JET MOTOR...........OK ");	
		}
		else
		{
			usartTx(CONSOLE_USART,"\r\n WATER JET MOTOR...........FAIL");	;
		}
		HAL_GPIO_WritePin(WATER_JET_MOTOR_PORT, WATER_JET_MOTOR_PIN,GPIO_PIN_RESET);
		Chrg_curr_ui16 = RESET;
		Load_curr_ui16 = RESET;
		Solar_Panel_vltg_ui16 = RESET;
		AdcBatCntrlvltgSum = RESET;
		if(PG_Powrtest.self_test == 0x7F)
		{
			usartTx(CONSOLE_USART,"\r\n\r\n POWER SUPPLY PCB...........SUCCESS ");	
		}
		else {
			usartTx(CONSOLE_USART,"\r\n POWER SUPPLY PCB...........FAIL");	
		}
	}
	else 
	{
	   usartTx(CONSOLE_USART,"\r\n ADC128 Comm fail");
     usartTx(CONSOLE_USART,"\r\n POWER_SUPLLY_PCB................FAIL");		
	}
}
uint16_t get_encoder_value()
{
	 i2c_comm_status_ui8 = HAL_I2C_Master_Receive(&hi2c1,MSP3021_I2C_ADDRESS,(uint8_t *)&ADC_encoder_voltage_ui8,2,2);
	 if(i2c_comm_status_ui8 != HAL_OK)
	 {
		 usartTx(CONSOLE_USART, "\r\n ENCODER COMMUNCIATION...........FAIL\r\n");	
		 return NULL;
	 }
	 return (((ADC_encoder_voltage_ui8[0] & 0x0F) << 6) | ((ADC_encoder_voltage_ui8[1] & 0xFC)>>2));
}
void Check_encoder()
{
	usartTx(CONSOLE_USART,"\r\n\r\n\r\n\r\n\r\n ENCODER PCB TESTING STARTED");
	uint8_t i=RESET;
	uint32_t encoder_sum = RESET;
	HAL_GPIO_WritePin(ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT, ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN,GPIO_PIN_SET);
	for(i=0;i<ADC128_SAMPLES;i++)
	{
		encoder_sum += get_encoder_value();
	}
	encoder_sum = encoder_sum/ADC128_SAMPLES;
	usartTx(CONSOLE_USART,"Encoder value: %d\r\n",encoder_sum);
  if(encoder_sum == 0)
  {
		usartTx(CONSOLE_USART, "\r\n ENCODER VALUE...........FAIL\r\n");	
  }
  else {
		usartTx(CONSOLE_USART, "\r\n ENCODER...........OK\r\n");	
  }		
}
uint8_t vConfig_PCA9536()
{
	PCA9536_Write[PCA9536_ADDRESS_INDEX] = 0x03; /* Configuration register */
	PCA9536_Write[PCA9536_DATA_INDEX] = 0xFF;   /* 0 output and 1 input */
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, PCA9536_ADDRESS, PCA9536_Write, 2, 10);
	if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	PCA9536_Write[PCA9536_ADDRESS_INDEX] = 0x03;
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, PCA9536_ADDRESS, PCA9536_Write, 1, 10);
	if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	i2c_comm_status_ui8 = HAL_I2C_Master_Receive(&hi2c1,PCA9536_ADDRESS, PCA9536_Read , 1, 10);
	if(!(PCA9536_Write[PCA9536_DATA_INDEX] == PCA9536_Read[0]))
	{
		usartTx(CONSOLE_USART, "\r\n PCA9536 configure register mismatch\r\n");
	}
	PCA9536_Write[PCA9536_ADDRESS_INDEX] = 0x00;
	i2c_comm_status_ui8 = HAL_I2C_Master_Transmit(&hi2c1, PCA9536_ADDRESS, PCA9536_Write, 1, 10);
	if(i2c_comm_status_ui8 != HAL_OK)
	{
		return i2c_comm_status_ui8;
	}
	i2c_comm_status_ui8 = HAL_I2C_Master_Receive(&hi2c1,PCA9536_ADDRESS, PCA9536_Read , 1, 10);
	usartTx(CONSOLE_USART,"\r\n Capacitor sensor GPIO status: 0x%x \r\n",((PCA9536_Read[0] << 8) | (PCA9536_Read[1])));
	if( ((PCA9536_Read[0] << 8) | (PCA9536_Read[1])) == 0xFF00 )
	{		
	  usartTx(CONSOLE_USART,"\r\n CAPACITIVE SENSOR......... OK \r\n");
		sDo_I2c_stat.DO_I2C_bits.capsense_b = 1;
	}
	else
	{
		usartTx(CONSOLE_USART,"\r\n CAPACITIVE SENSOR.........FAIL \r\n");
	}
	return i2c_comm_status_ui8;
}

void Check_DOI2C_PCB()
{
	float vref=0.0,temperature = 0.0,DO_voltage=0.0;
	uint8_t capSense_stat = RESET;
	usartTx(CONSOLE_USART,"\r\n\r\n\r\n\r\n\r\n DOI2C PCB TESTING STARTED");
	readADSSensor();
	vref = (selfClean.rawADC.data[0][1] - selfClean.rawADC.data[0][2]) * 2;
	vref = ((vref *24.7)/4.7);   /* Resistor divider calculation */
	usartTx(CONSOLE_USART,"\r\n +3V Vref channel voltage: %.2f V",(vref/1000));	
  if((vref >2800) && (vref < 3100) )
  {
		sDo_I2c_stat.DO_I2C_bits.vref_b = 1;
		usartTx(CONSOLE_USART,"\r\n +3V CHANNEL..........OK",vref);	
  }
	else
	{
		usartTx(CONSOLE_USART,"\r\n +3V CHANNEL..........FAIL",vref);	
	}
  temperature = selfClean.rawADC.data[0][3] * 2;
	usartTx(CONSOLE_USART,"\r\n Temperature channel voltage: %.2f V",(temperature/1000));		
	if(temperature >= 1000 && temperature<= 2000)
	{
		sDo_I2c_stat.DO_I2C_bits.temp_b = 1;
		usartTx(CONSOLE_USART,"\r\n TEMPERATURE CHANNEL ..........OK");		
	}
	else
	{
		usartTx(CONSOLE_USART,"\r\n TEMPERATURE CHANNEL ..........FAIL");		
	}
	DO_voltage = selfClean.rawADC.data[0][0] * 2;
	usartTx(CONSOLE_USART,"\r\n DO channel voltage: %.2f V ",(DO_voltage/1000));		
	if(DO_voltage >= 500 && DO_voltage <= 1000)
	{
		sDo_I2c_stat.DO_I2C_bits.DO_b = 1;
		usartTx(CONSOLE_USART,"\r\n DO CHANNEL ..........OK");		
	}
	else {
		usartTx(CONSOLE_USART,"\r\n DO CHANNEL ..........FAIL");		
	}
	capSense_stat = vConfig_PCA9536();
	if(capSense_stat != HAL_OK)
	{
		usartTx(CONSOLE_USART,"\r\n CAPACITOR SENSOR COMMUNICATION FAIL\r\n");		
  }
	if(sDo_I2c_stat.DO_PCB_status == 0x0F)
	{
		usartTx(CONSOLE_USART,"\r\nDO I2C PCB ........... OK\r\n");		
	}
	else {
		usartTx(CONSOLE_USART,"\r\nDO I2C PCB ........... FAIL");		
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
		  PG_Cntrltest.self_test_PGcntrl.GPS_comm_b = 1;
			usartTx(CONSOLE_USART,"\r\n GPS............OK");
	}
	else {
		usartTx(CONSOLE_USART,"\r\n GPS............FAIL");
	}
}

void vSendBLECommands(const sBLE_NMEA_CMDS_t *cmndsBuff,uint8_t ui8NoOfCmnds){
	sBLECmndBuffer = cmndsBuff;
	usartTx(CONSOLE_USART,(const char *)"\r\nBLE Cmd.........%s",(uint8_t *)sBLECmndBuffer->CmndString);
	HAL_UART_Transmit(&huart3,(uint8_t *)sBLECmndBuffer->CmndString,strlen(sBLECmndBuffer->CmndString),1000);
}

void vBLE_Init(){

	HAL_UART_Receive_IT(&huart3,sBLErxBuff.ui8DataBuff,MAX_BLE_RX_BUFFER_SIZE);
	HAL_Delay(1000);
	
	HAL_GPIO_WritePin(GPS_PWR_EN_GPIO_Port, GPS_PWR_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BT_NRST_GPIO_Port, BT_NRST_Pin, GPIO_PIN_SET);
	
	usartTx(CONSOLE_USART,"\r\nBLE ON and waiting for 5 sec");
  HAL_Delay(BLE_ON_TIME * 1000); 	
	
	vSendBLECommands(sBLEInitCmnds,sizeof(sBLEInitCmnds)/sizeof(sBLE_NMEA_CMDS_t));
	
	HAL_Delay(CMD_RESP_WAIT_TIME * 1000); 
	sBLErxBuff.ui8DataBuff[0]=0x0A;
	if(strstr((const char *)sBLErxBuff.ui8DataBuff,sBLEInitCmnds->ResponseString)){ 
		PG_Cntrltest.self_test_PGcntrl.BLE_comm_b = 1;
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
	usartTx(CONSOLE_USART,"\r\n Initialising RTC");
	PLATFORM_RTC_Set_Alarm(2);
	HAL_Delay(5000);
	if(RTC_AlarmEvntflag == SET)
	{
		PG_Cntrltest.self_test_PGcntrl.rtc_b = 1;
		usartTx(CONSOLE_USART,"\r\n RTC............OK");
	}
	else {
		usartTx(CONSOLE_USART,"\r\n RTC............FAIL");
	}
}
void Check_contr_PCB()
{
	usartTx(CONSOLE_USART,"\r\n\r\n\r\n\r\n\r\n CONTROLLER PCB TESTING STARTED \r\n\r\n");
	/* Test Flash chip */
	FlashTest();
	/* LCD Display */
	LCDTest();
	/* Test LoRa chip */
	LoRaTest();
	vGPS_Init(); 
	
	#ifdef PG2_5
	vBLE_Init();
	#endif
	
	HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_FWD_PIN,GPIO_PIN_RESET);
	HAL_Delay(2);
	if(!HAL_GPIO_ReadPin(POSITION_MOTOR_PORT,POSITION_MOTOR_REV_PIN))
	{
		PG_Cntrltest.self_test_PGcntrl.pos_motor_for_b = 1;
		usartTx(CONSOLE_USART,"\r\n Position_motor forward and reverse pins Logic low OK");
	}
	else
	{
		usartTx(CONSOLE_USART,"\r\n Position_motor forward and reverse pins Logic low FAIL");
	}
	HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_FWD_PIN,GPIO_PIN_SET);
	HAL_Delay(2);
	if(HAL_GPIO_ReadPin(POSITION_MOTOR_PORT,POSITION_MOTOR_REV_PIN))
	{
		PG_Cntrltest.self_test_PGcntrl.pos_motor_rev_b = 1;
		usartTx(CONSOLE_USART,"\r\n Position_motor forward and reverse pins Logic high OK");
	}
	else
	{
		usartTx(CONSOLE_USART,"\r\n Position_motor forward and reverse pins Logic high FAIL");
	}
	HAL_GPIO_WritePin(ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT, ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN,GPIO_PIN_RESET);
	HAL_Delay(2);
	if(!HAL_GPIO_ReadPin(WATER_JET_MOTOR_PORT, WATER_JET_MOTOR_PIN))
	{
		PG_Cntrltest.self_test_PGcntrl.water_jet_b = 1;
		usartTx(CONSOLE_USART,"\r\n Water jet pins and sensor power pins Logic low OK");
	}
	else
	{
		usartTx(CONSOLE_USART,"\r\n Water jet pins and sensor power pins Logic low FAIL");
	}
	HAL_GPIO_WritePin(ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT, ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN,GPIO_PIN_SET);
	HAL_Delay(2);
	if(HAL_GPIO_ReadPin(WATER_JET_MOTOR_PORT, WATER_JET_MOTOR_PIN)){
		PG_Cntrltest.self_test_PGcntrl.sensor_pwr_en_b = 1;
		usartTx(CONSOLE_USART,"\r\n Water jet pins and sensor power pins Logic high OK");
	}
	else {
		usartTx(CONSOLE_USART,"\r\n Water jet pins and sensor power pins Logic high FAIL");
	}
	HAL_GPIO_WritePin(OBJECT_DETECT_BUTTON_PORT, OBJECT_DETECT_BUTTON_PIN,GPIO_PIN_RESET);
	HAL_Delay(2);
	if(!HAL_GPIO_ReadPin(INDSENSE_PORT, INDSENSE_PIN))
	{
		PG_Cntrltest.self_test_PGcntrl.detect_swt_b = 1;
		usartTx(CONSOLE_USART,"\r\n Detect switch and Ind sense pins Logic low OK");
	}
	else
	{
		usartTx(CONSOLE_USART,"\r\n Detect switch and Ind sense pins Logic low FAIL");
	}
	HAL_GPIO_WritePin(OBJECT_DETECT_BUTTON_PORT, OBJECT_DETECT_BUTTON_PIN,GPIO_PIN_SET);
	HAL_Delay(2);
	if(HAL_GPIO_ReadPin(INDSENSE_PORT, INDSENSE_PIN))
	{
		PG_Cntrltest.self_test_PGcntrl.ind_sense_b = 1;
		usartTx(CONSOLE_USART,"\r\n Detect switch and Ind sense pins Logic high OK");
	}
	else
	{
		usartTx(CONSOLE_USART,"\r\n Detect switch and Ind sense pins Logic high FAIL");
	}
	RTC_Chck();
	if(PG_Cntrltest.self_test == 0x7FF)
	{
		usartTx(CONSOLE_USART,"\r\n\r\n CONTROLLER PCB.............OK");	
	}
	else {
		usartTx(CONSOLE_USART,"\r\n\r\n CONTROLLER PCB.............FAIL");	
	}
}
void chck_ldCurr()
{
	uint8_t j=RESET,i2c_status = RESET;
	usartTx(CONSOLE_USART,"\r\n\r\n\r\n\r\n\r\n Load current testing started");
	HAL_GPIO_WritePin(ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT, ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN,GPIO_PIN_SET);
	HAL_Delay(2);
	i2c_status = vInit_ADC_channels();
	if(i2c_status == HAL_OK)
	{
		PG_Cntrltest.self_test_PGcntrl.ADC128_comm_b = 1;
		SetADC18_ConvChanls(( 1 << ADC128_LOAD_CURRENT_INDEX));
		for(j=0;j< 100;j++)
		{
			HAL_Delay(13);
			readADC128(ADC128_LOAD_CURRENT_INDEX);
			Load_curr_ui16 = ADC128_data[ADC128_LOAD_CURRENT_INDEX];
			Loadcurravg_fl = (Load_curr_ui16*ADC_REF_VOLTAGE_ADC128_MILLIS)/MAX_ADC_COUNT_VALUE;
			Loadcurravg_fl = (Loadcurravg_fl/CHRG_CURR_OP_AMP_GAIN);
			Loadcurravg_fl = (Loadcurravg_fl/CHRG_CURRENT_LOAD_RESISTOR);
			usartTx(CONSOLE_USART,"\r\n Load current:%.2f mA",Loadcurravg_fl);
		}
	}
}
void Chck_WaterjetCurr()
{
	uint16_t j=RESET,i2c_status = RESET;
	usartTx(CONSOLE_USART,"\r\n\r\n\r\n\r\n\r\n Waterjet current testing started");
	HAL_GPIO_WritePin(ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT, ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN,GPIO_PIN_SET);
	HAL_Delay(2);
	i2c_status = vInit_ADC_channels();
	if(i2c_status == HAL_OK)
	{
		SetADC18_ConvChanls(( 1 << WATER_JET_CURRENT));
		HAL_GPIO_WritePin(WATER_JET_MOTOR_PORT, WATER_JET_MOTOR_PIN,GPIO_PIN_SET);
		for(j=0;j < 300;j++)
		{
			HAL_Delay(13);
			readADC128(WATER_JET_CURRENT); 
			jetCurrent_mA = ADC128_data[WATER_JET_CURRENT];
			jetCurrent_mA = (jetCurrent_mA *ADC_REF_VOLTAGE_ADC128_MILLIS)/MAX_ADC_COUNT_VALUE;
			jetCurrent_mA = jetCurrent_mA/260;	
      usartTx(CONSOLE_USART,"\r\n Water jet motor current...........%.2f A",jetCurrent_mA);				
		}
		HAL_GPIO_WritePin(WATER_JET_MOTOR_PORT, WATER_JET_MOTOR_PIN,GPIO_PIN_RESET);
	}
}
void Chck_PosMotorCurr()
{
	uint16_t j=RESET,i2c_status = RESET;
	usartTx(CONSOLE_USART,"\r\n\r\n\r\n\r\n\r\n Position motor current testing started");
	HAL_GPIO_WritePin(ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT, ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN,GPIO_PIN_SET);
	HAL_Delay(2);
	i2c_status = vInit_ADC_channels();
	if(i2c_status == HAL_OK)
	{
		SetADC18_ConvChanls(( 1 << POSITION_MOTOR_CURRENT_INDEX));
		HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_FWD_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_REV_PIN,GPIO_PIN_RESET);
		usartTx(CONSOLE_USART,"\r\n\r\n Position motor turned ON forward direction\r\n\r\n");
		for(j=0;j< 500;j++)
		{
			readADC128(POSITION_MOTOR_CURRENT_INDEX);
			fSlfCleanMotorCurrent_mA = ADC128_data[POSITION_MOTOR_CURRENT_INDEX];	
			fSlfCleanMotorCurrent_mA = (fSlfCleanMotorCurrent_mA * ADC_REF_VOLTAGE_ADC128_MILLIS)/MAX_ADC_COUNT_VALUE;
			fSlfCleanMotorCurrent_mA = fSlfCleanMotorCurrent_mA/POS_MOTOR_CURR_OP_AMP_GAIN;
			fSlfCleanMotorCurrent_mA = fSlfCleanMotorCurrent_mA/CHRG_CURRENT_LOAD_RESISTOR;
			usartTx(CONSOLE_USART,"\r\n\r\n Position motor forward ADC Value...........%d ",ADC128_data[POSITION_MOTOR_CURRENT_INDEX]);	
			HAL_Delay(13);
		}
		HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_FWD_PIN,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_REV_PIN,GPIO_PIN_SET);
		usartTx(CONSOLE_USART,"\r\n\r\n Position motor turned ON reverse direction\r\n\r\n");
		for(j=0;j< 500;j++)
		{
			readADC128(POSITION_MOTOR_CURRENT_INDEX);
			fSlfCleanMotorCurrent_mA = ADC128_data[POSITION_MOTOR_CURRENT_INDEX];	
			fSlfCleanMotorCurrent_mA = (fSlfCleanMotorCurrent_mA *ADC_REF_VOLTAGE_ADC128_MILLIS)/MAX_ADC_COUNT_VALUE;
			fSlfCleanMotorCurrent_mA = fSlfCleanMotorCurrent_mA/POS_MOTOR_CURR_OP_AMP_GAIN;
			fSlfCleanMotorCurrent_mA = fSlfCleanMotorCurrent_mA/CHRG_CURRENT_LOAD_RESISTOR;
      usartTx(CONSOLE_USART,"\r\n Position motor reverse ADC Value...........%d ",ADC128_data[POSITION_MOTOR_CURRENT_INDEX]);				
			HAL_Delay(13);
		}
		HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_FWD_PIN,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(POSITION_MOTOR_PORT, POSITION_MOTOR_REV_PIN,GPIO_PIN_RESET);  
	}
}
void CheckADSSensor() {
	HAL_GPIO_WritePin(ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT, ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN,GPIO_PIN_SET);
	HAL_Delay(100);
	ADSwrite[0] = 0x02;
	ADSwrite[1] = 0x55;
	ADSwrite[2] = 0x00; // 10000011
	status = HAL_I2C_Master_Transmit(&hi2c1, (ADS1015_ADDRESS)<<1, ADSwrite, 3, 50);
	if(status != HAL_OK) {
		usartTx(CONSOLE_USART, "\r\n ADC128 comm fail \r\n");
		return;
	}
	else {
		i2cCommunicationError = 0;
	}
	HAL_GPIO_WritePin(ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT, ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN,GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT, ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN,GPIO_PIN_SET);
	ADSwrite[0] = 0x02;
	HAL_I2C_Master_Transmit(&hi2c1, (ADS1015_ADDRESS)<<1, ADSwrite, 1, 50);
	if(status != HAL_OK) {
		usartTx(CONSOLE_USART, "\r\n ADC128 comm fail \r\n");
		return;
	}
	else {
		i2cCommunicationError = 0;
	}
	HAL_Delay(5);
	status = HAL_I2C_Master_Receive(&hi2c1, (ADS1015_ADDRESS)<<1, ADSwrite, 2, 50);
	if(status != HAL_OK) {
		usartTx(CONSOLE_USART, "\r\n ADC128 comm fail \r\n");
		return;
	}
	else {
		i2cCommunicationError = 0;
	}
	adcOut[0][0] = (ADSwrite[0] << 8 | ADSwrite[1]);
//	usartTx(CONSOLE_USART, "\r\n Threshold read value:%x \r\n",adcOut[0][0]);
}
void PGTest()
{
/*	Chck_PosMotorCurr();
	chck_ldCurr(); 
	Chck_WaterjetCurr(); */
//  Check_encoder(); 
//	Chck_Powr_PCB(); 
//	if(HAL_GetTick() - TempLCDUpdTimout > 100)
//  {		
//		TempLCDUpdTimout = HAL_GetTick();
//		usartTx(CONSOLE_USART, "\r\n Temperature: %.2f deg\r\n",getTemperatureValueFromADS());
//		ModifdreadADSSensor();
//		usartTx(CONSOLE_USART, "\r\n Modified Temperature: %.2f deg\r\n",ModgetTemperatureValueFromADS());
//	}
	Check_DOI2C_PCB();
	HAL_Delay(50); 	  
//	Cntrl_GPIO_Config();
	Check_contr_PCB(); 
//	CheckADSSensor(); 
}
#endif
