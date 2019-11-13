/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : environment_log.c                                                                              *
 *  Abstract   : Environment logger handling driver.                                                            *
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : FEBRUARY 2019                                                                                  *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "sensors.h"
#include "adt7420.h"
#include "sht35.h"
#include "adxl345.h"
#include "lis3dshtr.h"
#include "main.h"
#include "upload.h"
#include "eeprom_image.h"
#include "internal_flash.h"

//void flash_programming( uint64_t source_address , uint32_t destination_address , uint32_t size);
/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define ENVIRONMENT_LOG_RECORDS			16

typedef struct
{
	uint32_t	date;
	uint32_t	time;
	uint16_t 	adt7420_temprature;
	uint16_t 	internal_sht35_temprature;
	uint16_t 	internal_sht35_humidity;
	uint32_t 	external_sht35_temprature;
	uint32_t 	external_sht35_humidity;
	uint32_t 	external_co2;
	int16_t  	adxl_accelerometer_axis_x;
	int16_t  	adxl_accelerometer_axis_y;
	int16_t  	adxl_accelerometer_axis_z;
	int16_t  	lis3dshtr_accelerometer_axis_x;
	int16_t  	lis3dshtr_accelerometer_axis_y;
	int16_t  	lis3dshtr_accelerometer_axis_z;
	uint16_t 	battery_bits;
	uint8_t		spare[20];
} ENVIRONMENT_LOG_RECORD;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern ENVIRONMENT_LOG_RECORD environment_log[ENVIRONMENT_LOG_RECORDS] __attribute__ ((section (".ram2"))); //SRAM2 memory only in turn off volatage is erased, between standby it is saved and not changed.
ENVIRONMENT_LOG_RECORD environment_log[ENVIRONMENT_LOG_RECORDS];
extern uint16_t environment_log_record_index __attribute__ ((section (".ram2"))); //SRAM2 memory only in turn off volatage is erased, between standby it is saved and not changed.
uint16_t environment_log_record_index;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
/*void environment_log_set_time()
{
	RTC_TimeTypeDef sTime;

	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
	    _Error_Handler(__FILE__, __LINE__);
	}
}

void environment_log_set_date()
{
	RTC_DateTypeDef sDate;

	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 0x1;
	sDate.Year = 0x0;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
	    _Error_Handler(__FILE__, __LINE__);
	}
}*/

void environment_log_reset_log()
{
	for( environment_log_record_index = 0 ; environment_log_record_index < ENVIRONMENT_LOG_RECORDS ; environment_log_record_index ++ )
		memset( &environment_log[environment_log_record_index] , 0 , sizeof(ENVIRONMENT_LOG_RECORD) );
	environment_log_record_index = 0;
}

void environment_log_update_time_and_date( uint32_t date , uint32_t time )
{
	uint16_t counter = 0;

	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	hrtc.Instance->ISR |= 0x00000080;
	while( ( ( hrtc.Instance->ISR & 0x00000040 ) == 0 ) && counter++ < 1000 );
	hrtc.Instance->TR   = time;
	hrtc.Instance->DR   = date;
	hrtc.Instance->ISR &= ~0x00000080;
    __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
}

void environment_log_update_record()
{
	environment_log[environment_log_record_index].date = hrtc.Instance->DR;
	environment_log[environment_log_record_index].time = hrtc.Instance->TR;
	environment_log[environment_log_record_index].adt7420_temprature = adt7420_get_last_temperature_bits();
	environment_log[environment_log_record_index].external_co2 = getCo2LevelFromExtBoard();
	sht35_get_last_temperature_and_humidity_bits( &environment_log[environment_log_record_index].internal_sht35_temprature , &environment_log[environment_log_record_index].internal_sht35_humidity );
	adxl345_get_accelerometer_sample(   (SAMPLE *)&environment_log[environment_log_record_index].adxl_accelerometer_axis_x );
	lis3dshtr_get_accelerometer_sample( (SAMPLE *)&environment_log[environment_log_record_index].lis3dshtr_accelerometer_axis_x );
	environment_log_record_index = ( ( environment_log_record_index + 1 ) % ENVIRONMENT_LOG_RECORDS );
}

char *environment_log_get_environment_log_pointer()
{
	return (char *)environment_log;
}
