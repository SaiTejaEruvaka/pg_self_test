/* Copyright (C) Eruvaka Technologies - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * 2018 */
 
#ifdef PONDMOTHER

#ifndef __PMTEST_H__
#define __PMTEST_H__

#include "stm32l4xx_hal.h"

void PMTest(void);

#define GPS_ON_TIME  35U // 35 seconds
#define BLE_ON_TIME  5U // 5 seconds
#define MAX_GPS_RX_BUFFER_SIZE 100U
#define MAX_BLE_RX_BUFFER_SIZE 100U

#define MAX_CMND_STRING_LEN 55U
#define CMD_RESP_WAIT_TIME 7U 

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

#endif

#endif
//
