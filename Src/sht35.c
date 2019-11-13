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
#include "sht35.h"
#include "command.h"

/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define SHT35_SLAVE_ADDRESS										0x88

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
#define ART_COMMAND												(uint8_t [2]){ 0x2B , 0x32 }

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
} SHT35_REGISTER;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
static int16_t last_temperature_bits , last_humidity_bits;
static uint16_t temperature , humidity ;
static uint8_t	initialized = 0;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void SHT35_I2C_write_word( uint16_t DevAddress , uint8_t *command )
{
	I2C2_write_byte( DevAddress , command[0] , command[1] );
}

int16_t sht35_get_one_shot_temperature()
{
	return last_temperature_bits = (int16_t)SENSIRION_I2C_read_word( SHT35_SLAVE_ADDRESS , SINGLE_SHOT_LOW_REPEATABILITY_CLOCK_STRECH_ENABLE );
}

int16_t sht35_get_periodic_mode_temperature()
{
	return last_temperature_bits = SENSIRION_I2C_read_word( SHT35_SLAVE_ADDRESS , FETCH_DATA );
}

void sht35_start_periodic_mode_temperature_sampling(uint8_t *command)
{
	SHT35_I2C_write_word( SHT35_SLAVE_ADDRESS , command );
}

void sht35_stop_periodic_mode_temperature_sampling()
{
	SHT35_I2C_write_word( SHT35_SLAVE_ADDRESS , BREAK_COMMAND );
}

void sht35_soft_reset()
{
	SHT35_I2C_write_word( SHT35_SLAVE_ADDRESS , SOFT_RESET );
}

void sht35_general_call_reset()
{
	SENSIRION_I2C_write_general_call_reset( GENERAL_CALL_RESET );
}

void sht35_heater_enable()
{
	SHT35_I2C_write_word( SHT35_SLAVE_ADDRESS , HEATER_ENABLE );
}

void sht35_heater_disable()
{
	SHT35_I2C_write_word( SHT35_SLAVE_ADDRESS , HEATER_DISABLE );
}

void sht35_clear_status()
{
	SHT35_I2C_write_word( SHT35_SLAVE_ADDRESS , CLEAR_STATUS );
}

uint16_t sht35_get_status()
{
	return SENSIRION_I2C_read_word( SHT35_SLAVE_ADDRESS , READ_STATUS );
}

void sht35_initialization()
{
	sht35_clear_status();
}

float sht35_get_temperature_in_celsius(int16_t raw_temperature)
{
	return -45.0 + 175 * ( raw_temperature / (float)0xFFFF );
}

float sht35_get_temperature_in_fahrenheit(int16_t raw_temperature)
{
	return -49.0 + 315 * ( raw_temperature / (float)0xFFFF );
}

uint8_t sht35_get_humidity(int16_t raw_humidity)
{
	return 100 * ( raw_humidity / (float)0xFFFF );
}

float sht35_get_temperature()
{
	if (initialized == 0)
	{
		sht35_get_temperature_and_humidity_bits( &temperature , &humidity );
	}
	return sht35_get_temperature_in_celsius(last_temperature_bits);
}

void sht35_get_temperature_sample(void)
{
	int16_t temperature = sht35_get_one_shot_temperature();
	send_message_to_pc( (char *)&temperature , 2 );
}

void sht35_get_humidity_sample(void)
{
	int16_t temperature = sht35_get_one_shot_temperature();
	send_message_to_pc( (char *)&temperature , 2 );
}

void sht35_get_temperature_and_humidity_bits( uint16_t *temperature , uint16_t *humidity )
{
	uint8_t samples[6];
	HAL_Delay(1300);
	SENSIRION_I2C_read_array( SHT35_SLAVE_ADDRESS , SINGLE_SHOT_LOW_REPEATABILITY_CLOCK_STRECH_ENABLE , samples , 6 );
	last_temperature_bits = *temperature = (  samples[1] | ( samples[0] << 8 ) );
	last_humidity_bits    = *humidity =    (  samples[4] | ( samples[3] << 8 ) );
	initialized = 1;
}

void sht35_get_last_temperature_and_humidity_bits( uint16_t *temperature , uint16_t *humidity )
{
	*temperature = last_temperature_bits;
	*humidity    = last_humidity_bits;
}

void sht35_get_sample(void)
{
	static char sht_samples_string[6];
	static uint8_t samples[6];
	SENSIRION_I2C_read_array( SHT35_SLAVE_ADDRESS , SINGLE_SHOT_LOW_REPEATABILITY_CLOCK_STRECH_ENABLE , samples , 6 );
	sprintf( sht_samples_string , "%c%c%c%c" , samples[1] , samples[0] , samples[4] , samples[3] );
	send_message_to_pc( (char *)&sht_samples_string , strlen((const char *)sht_samples_string) );
}

