#ifndef __PG_TEST_H__
#define __PG_TEST_H__
#include "stm32l4xx_hal.h"
#pragma anon_unions
#define TRUE 1
#define FALSE 0
#define SCL_Pin GPIO_PIN_6
#define SDA_Pin GPIO_PIN_7
#define POSITION_MOTOR_PORT GPIOC
#define POSITION_MOTOR_FWD_PIN GPIO_PIN_8
#define POSITION_MOTOR_REV_PIN GPIO_PIN_9
#define WATER_JET_MOTOR_PORT GPIOC 
#define WATER_JET_MOTOR_PIN GPIO_PIN_10
#define OBJECT_DETECT_BUTTON_PORT GPIOB
#define OBJECT_DETECT_BUTTON_PIN GPIO_PIN_0
#define TRAIL_REQUEST_PORT GPIOB
#define TRAIL_REQUEST_PIN GPIO_PIN_1	
#define ENCODER_INDUC_SENSOR_PWR_CNTRL_PIN GPIO_PIN_6
#define ENCODER_INDUC_SENSOR_PWR_CNTRL_PORT GPIOC
#define INDSENSE_PORT GPIOA
#define INDSENSE_PIN GPIO_PIN_8
#define ADS1015_ADDRESS 0x48
#define PCA9536_ADDRESS 0x82
#define PCA9536_TURN_OFF_WATER_JET_BIT 0
#define PCA9536_ADDRESS_INDEX 0x00
#define PCA9536_DATA_INDEX 0x01
#define ADC128_ADDRESS 0x3A
#define ADC128_CHANNEL_REGISTERS_OFFSET 0x20
#define ADC128_CHANNEL_OFFSET 0x02
#define NOT_READY_BIT_POSITION 0x01
#define BATTERY_VOLTAGE_INDEX 0x07
#define ADC128_SOLAR_VOLTAGE_INDEX 0x06
#define ADC128_CHARGE_CURRENT_INDEX 0x05
#define ADC128_LOAD_CURRENT_INDEX 0x04
#define POSITION_MOTOR_CURRENT_INDEX 0x03
#define WATER_JET_CURRENT 0x02
#define ADC_BAT_CNTRL_VLTG_INDEX 0x01
#define ADC_REF_VOLTAGE_ADC128_MILLIS 2560
#define ADC128_START_CHANNEL 0x01
#define ADC128_END_CHANNEL 0x07
#define ADC128_SAMPLES 0x0A
#define MSP3021_I2C_ADDRESS 0x9E
#define NUM_OF_ADC128CHANNELS     8 
#define MAX_ADC_COUNT_VALUE         4095
#define ADC_REF_VOLTAGE_MILLI_VOLTS 3300
#define POS_MOTOR_CURR_OP_AMP_GAIN 100
#define CHRG_CURR_OP_AMP_GAIN 20
#define CHRG_CURRENT_LOAD_RESISTOR 0.1
#define WATER_JET_OFFSET_COUNT 640 /* 400 mV */
#define WATER_JET_NO_LOAD_COUNT  1040 /* 530 mV */
#define POSITION_MOTOR_MIN_CURR 18
#define POSITION_MOTOR_MAX_CURR 40
#define ADC_BAT_RESISTANCE1_KOHM     10
#define ADC_BAT_RESISTANCE2_KOHM     2
#define SOLAR_PANEL_RESISTANCE1_KOHM     10
#define SOLAR_PANEL_RESISTANCE2_KOHM     1
#define MAX_CMND_STRING_LEN 55U
#define CMD_RESP_WAIT_TIME 3U  

#define GPS_ON_TIME  35U // 35 seconds
#define BLE_ON_TIME  5U // 5 seconds
#define MAX_GPS_RX_BUFFER_SIZE 100U
#define MAX_BLE_RX_BUFFER_SIZE 100U

typedef union self_test_PGcntrl_u
{
	uint16_t self_test;
	
	struct self_test_PGcntrl_t
	{
		uint16_t	pos_motor_for_b:1;
		uint16_t	pos_motor_rev_b:1;
		uint16_t	water_jet_b:1;
		uint16_t	sensor_pwr_en_b:1;
		uint16_t	detect_swt_b:1;
		uint16_t	ind_sense_b:1;
    uint16_t	ADC128_comm_b:1;
		uint16_t	LoRa_comm_b:1;
		uint16_t	GPS_comm_b:1;
		uint16_t  BLE_comm_b:1;
		uint16_t	flash_b:1;
		uint16_t	rtc_b:1;
	}self_test_PGcntrl;
	
}self_test_PG_Cntrl_t;
typedef union self_test_PGPowr_u
{
	uint8_t self_test;
	
	struct self_test_PGPowr_t
	{
		uint8_t	pos_motor_for_b:1;
		uint8_t	pos_motor_rev_b:1;
		uint8_t	water_jet_motor:1;
		uint8_t	Solar_vltg_b:1;
		uint8_t	Battery_vltg_b:1;
		uint8_t	ADCBat_vltg_b:1;
		uint8_t	LoadCurr_b:1;
	}self_test_PGcntrl;
	
}self_test_PG_Powr_t;
typedef union DO_I2C_t
{
	uint8_t DO_PCB_status;
	struct DO_I2C_bits_t
	{
		uint8_t DO_b:1;
		uint8_t temp_b:1;
		uint8_t vref_b:1;
		uint8_t capsense_b:1;
	}DO_I2C_bits;
}sDO_I2C;
	typedef enum{
		IDLE_STATE = 0x00U,
		PULLING_SENSOR,
		CLEANING,
		CALIBRATE_WAIT,
		DROPPING_SENSOR
	}eSELF_CELAN_STATES_t;

	typedef union{
			struct {
				uint16_t reserved1;
				uint16_t reserved2;
				uint16_t reserved3;
				uint16_t reserved4;
				uint16_t reserved5;
				uint16_t encoderMainWheel;
				uint16_t encoderSupportWheel;
				uint16_t reserved6;
			};
			struct {
				uint16_t data[2][4];
			};
	}sRAW_ADC_t;
	
	typedef struct {
		float height;
		float waterLevel;
		float jetAvgCurrent;
		float mainWheelHeight;
		#ifdef SLIP_RINGS
		float slipRingWheelHeight;
		#endif
	}sSENSOR_INFO_t;
		
	typedef struct {
		uint8_t sensorPosInitialized:1;
		uint8_t positionMotorDir:2;
		sRAW_ADC_t rawADC;
		sSENSOR_INFO_t sensorInfo;
		eSELF_CELAN_STATES_t state;
		float requiredHeight;
		uint32_t timeout;
		uint32_t lastTick;
	}sSELF_CLEAN_t;
typedef struct{
	char CmndString[MAX_CMND_STRING_LEN];
	char ResponseString[MAX_CMND_STRING_LEN];
	void (*vResponseCallBack)(uint32_t );
}sGPS_NMEA_CMDS_t;

typedef struct{
	char CmndString[MAX_CMND_STRING_LEN];
	char ResponseString[MAX_CMND_STRING_LEN];
	void (*vResponseCallBack)(uint32_t );
}sBLE_NMEA_CMDS_t;

typedef struct {
	uint8_t ui8DataBuff[MAX_GPS_RX_BUFFER_SIZE];
	uint16_t ui16ReadPtr;
}sGPS_RX_BUFF_t;

typedef struct {
	uint8_t ui8DataBuff[MAX_BLE_RX_BUFFER_SIZE];
	uint16_t ui16ReadPtr;
}sBLE_RX_BUFF_t;

extern void PGTest(void);
extern self_test_PG_Cntrl_t PG_Cntrltest;
extern sGPS_RX_BUFF_t sGPSrxBuff;
extern void Cntrl_GPIO_Config(void);
extern uint8_t RTC_AlarmEvntflag;
#endif

