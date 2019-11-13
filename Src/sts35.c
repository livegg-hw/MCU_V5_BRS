/*----------------------------------------------------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         											*
 *  File name  : sts35.c                                                                                        											*
 *  Abstract   : 16-Bit Digital I2C Temperature sensor driver.                                                  											*
 *	https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/0_Datasheets/Temperature/Sensirion_Temperature_Sensors_STS3x_Datasheet.pdf*
 *  Written by : Ofer Freilich                                                                                  											*
 *  Date       : MAY 2018                                                                                       											*
 *----------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "i2c.h"
#include "sensors.h"
#include "sts35.h"

/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define STS35_SLAVE_ADDRESS										0x88
#define UART_TIMEOUT                							2000

/* Single Shot Data Acquisition Mode */
#define SINGLE_SHOT_HIGH_REPEATABILITY_CLOCK_STRECH_ENABLE		(uint8_t [2]){ 0x2C , 0x06 }
#define SINGLE_SHOT_MEDIUM_REPEATABILITY_CLOCK_STRECH_ENABLE	(uint8_t [2]){ 0x2C , 0x0D }
#define SINGLE_SHOT_LOW_REPEATABILITY_CLOCK_STRECH_ENABLE		(uint8_t [2]){ 0x2C , 0x10 }
#define SINGLE_SHOT_HIGH_REPEATABILITY_CLOCK_STRECH_DISABLE		(uint8_t [2]){ 0x24 , 0x00 }
#define SINGLE_SHOT_MEDIUM_REPEATABILITY_CLOCK_STRECH_DISABLE	(uint8_t [2]){ 0x24 , 0x0B }
#define SINGLE_SHOT_LOW_REPEATABILITY_CLOCK_STRECH_DISABLE		(uint8_t [2]){ 0x24 , 0x16 }

/* Measurement Commands for Periodic Data Acquisition Mode */
#define PERIODIC_MODE_HIGH_REPEATABILITY_0_5_MPS				(uint8_t [2]){ 0x20 , 0x32 }
#define PERIODIC_MODE_MEDIUM_REPEATABILITY_0_5_MPS				(uint8_t [2]){ 0x20 , 0x24 }
#define PERIODIC_MODE_LOW_REPEATABILITY_0_5_MPS					(uint8_t [2]){ 0x20 , 0x2F }
#define PERIODIC_MODE_HIGH_REPEATABILITY_1_MPS					(uint8_t [2]){ 0x21 , 0x30 }
#define PERIODIC_MODE_MEDIUM_REPEATABILITY_1_MPS				(uint8_t [2]){ 0x21 , 0x26 }
#define PERIODIC_MODE_LOW_REPEATABILITY_1_MPS					(uint8_t [2]){ 0x21 , 0x2D }
#define PERIODIC_MODE_HIGH_REPEATABILITY_2_MPS					(uint8_t [2]){ 0x22 , 0x36 }
#define PERIODIC_MODE_MEDIUM_REPEATABILITY_2_MPS				(uint8_t [2]){ 0x22 , 0x20 }
#define PERIODIC_MODE_LOW_REPEATABILITY_2_MPS					(uint8_t [2]){ 0x22 , 0x2B }
#define PERIODIC_MODE_HIGH_REPEATABILITY_4_MPS					(uint8_t [2]){ 0x23 , 0x34 }
#define PERIODIC_MODE_MEDIUM_REPEATABILITY_4_MPS				(uint8_t [2]){ 0x23 , 0x22 }
#define PERIODIC_MODE_LOW_REPEATABILITY_4_MPS					(uint8_t [2]){ 0x23 , 0x29 }
#define PERIODIC_MODE_HIGH_REPEATABILITY_10_MPS					(uint8_t [2]){ 0x27 , 0x37 }
#define PERIODIC_MODE_MEDIUM_REPEATABILITY_10_MPS				(uint8_t [2]){ 0x27 , 0x21 }
#define PERIODIC_MODE_LOW_REPEATABILITY_10_MPS					(uint8_t [2]){ 0x27 , 0x2A }

/* General commands */
#define FETCH_DATA												(uint8_t [2]){ 0xE0 , 0x00 }
#define BREAK_COMMAND											(uint8_t [2]){ 0x30 , 0x93 }
#define SOFT_RESET												(uint8_t [2]){ 0x30 , 0xA2 }
#define GENERAL_CALL_RESET										(uint8_t [2]){ 0x00 , 0x06 }
#define HEATER_ENABLE											(uint8_t [2]){ 0x30 , 0x6D }
#define HEATER_DISABLE											(uint8_t [2]){ 0x30 , 0x66 }
#define READ_STATUS												(uint8_t [2]){ 0xF3 , 0x2D }
#define CLEAR_STATUS											(uint8_t [2]){ 0x30 , 0x41 }

typedef enum
{
	TEMPERATURE             = 0x00,
	TEMPERATURE_MSB         = 0x00,
	TEMPERATURE_LSB         = 0x01,
	STATUS                  = 0x02,
	CONFIGURATION           = 0x03,
	T_HIGH_SETPOINT         = 0x04,
	T_HIGH_SETPOINT_MSB     = 0x04,
	T_HIGH_SETPOINT_LSB     = 0x05,
	T_LOW_SETPOINT          = 0x06,
	T_LOW_SETPOINT_MSB      = 0x06,
	T_LOW_SETPOINT_LSB      = 0x07,
	T_CRITICAL_SETPOINT     = 0x08,
	T_CRITICAL_SETPOINT_MSB = 0x08,
	T_CRITICAL_SETPOINT_LSB = 0x09,
	T_HYST_SETPOINT         = 0x0A,
	ID                      = 0x0B,
	SOFTWARE_RESET          = 0x2F
} STS35_REGISTER;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
uint16_t sts35_get_one_shot_temperature()
{
	return STS35_I2C_read_word( STS35_SLAVE_ADDRESS , SINGLE_SHOT_LOW_REPEATABILITY_CLOCK_STRECH_ENABLE );
}

uint16_t sts35_get_periodic_mode_temperature()
{
	return STS35_I2C_read_word( STS35_SLAVE_ADDRESS , FETCH_DATA );
}

void sts35_start_periodic_mode_temperature_sampling(uint8_t *command)
{
	STS35_I2C_write_word( STS35_SLAVE_ADDRESS , command );
}

void sts35_stop_periodic_mode_temperature_sampling()
{
	STS35_I2C_write_word( STS35_SLAVE_ADDRESS , BREAK_COMMAND );
}

void sts35_soft_reset()
{
	STS35_I2C_write_word( STS35_SLAVE_ADDRESS , SOFT_RESET );
}

void sts35_general_call_reset()
{
	STS35_I2C_write_general_call_reset( GENERAL_CALL_RESET );
}

void sts35_heater_enable()
{
	STS35_I2C_write_word( STS35_SLAVE_ADDRESS , HEATER_ENABLE );
}

void sts35_heater_disable()
{
	STS35_I2C_write_word( STS35_SLAVE_ADDRESS , HEATER_DISABLE );
}

void sts35_clear_status()
{
	STS35_I2C_write_word( STS35_SLAVE_ADDRESS , CLEAR_STATUS );
}

uint16_t sts35_get_status()
{
	return STS35_I2C_read_word( STS35_SLAVE_ADDRESS , READ_STATUS );
}
uint16_t raw_temperature;
float temperature;
void sts35_initialization()
{
	sts35_clear_status();
	while(1)
	{
	raw_temperature = sts35_get_one_shot_temperature();
	temperature = sts35_get_temperature_in_celsius(raw_temperature);
	}
}

float sts35_get_temperature_in_celsius(int16_t raw_temperature)
{
	return -45.0 + 175 * ( raw_temperature / (float)0xFFFF );
}

void sts35_get_sample(void)
{
	int16_t temperature = sts35_get_one_shot_temperature();
    HAL_UART_Transmit( &huart1 , (uint8_t *)&temperature , 2 , UART_TIMEOUT );
}

