/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : adt7420.c                                                                                      *
 *  Abstract   : 16-Bit Digital I2C Temperature sensor driver.                                                  *
 *				 http://www.analog.com/media/en/technical-documentation/data-sheets/ADT7420.pdf                 *
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : APRIL 2018                                                                                     *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "i2c.h"
#include "sensors.h"
#include "adt7420.h"
#include "command.h"

/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define ADT7420_SLAVE_ADDRESS			0x96

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
} ADT7420_REGISTER;

typedef enum
{
	FAULT_QUEUE_1         = 0x00,
	FAULT_QUEUE_2         = 0x01,
	FAULT_QUEUE_3         = 0x02,
	FAULT_QUEUE_4 	      = 0x03,
	CT_PIN_ACTIVE_LOW     = 0x00,
	CT_PIN_ACTIVE_HIGH    = 0x04,
	INT_PIN_ACTIVE_LOW    = 0x00,
	INT_PIN_ACTIVE_HIGH   = 0x08,
	INTERRUPT_MODE		  = 0x00,
	COMPARATOR_MODE       = 0x10,
	CONTINUOUS_CONVERSION = 0x00,
	ONE_SHOT_CONVERSION   = 0x20,
	SPS_MODE_CONVERSION   = 0x40,
	SHUTDOWN_CONVERSION   = 0x60,
	RESOLUTION_13_BITS    = 0x00,
	RESOLUTION_16_BITS    = 0x80
} CONFIGURATION_REGISTER;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
static volatile int16_t last_temperature_bits;
static uint8_t initialized = 0;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void adt7420_set_configuration()
{
	I2C2_write_byte( ADT7420_SLAVE_ADDRESS , CONFIGURATION , FAULT_QUEUE_4 | CT_PIN_ACTIVE_LOW | INT_PIN_ACTIVE_LOW | INTERRUPT_MODE | ONE_SHOT_CONVERSION | RESOLUTION_16_BITS );
}

int16_t adt7420_get_temperature_bits()
{
	uint16_t temperature = __REV16( I2C2_read_word( ADT7420_SLAVE_ADDRESS , TEMPERATURE ) );
	initialized = 1;
	return last_temperature_bits = ( ( temperature & 0x8000 ) ? ( temperature - 65536 ) : temperature );
}

int16_t adt7420_get_one_shot_temperature()
{
	adt7420_set_configuration();
	HAL_Delay(1000);
	adt7420_get_temperature_bits();
	return adt7420_get_temperature_bits();
}

int16_t adt7420_get_last_temperature_bits()
{
	return last_temperature_bits;
}

float adt7420_get_temperature()
{
	static char adt7420_string[100];
	if (initialized == 0)
	{
		adt7420_set_configuration();
		adt7420_get_temperature_bits();
		sprintf( adt7420_string , "ADT was not Initialized !! ATD7420 - BITS=%d TEMPERATURE=%.2f\r\n" , last_temperature_bits , last_temperature_bits / 128.0 );
		send_message_to_pc( (char *)adt7420_string , strlen(adt7420_string) );
	}
	else
	{
		sprintf( adt7420_string , "ADT --WAS-- Initialized !! ATD7420 - BITS=%d TEMPERATURE=%.2f\r\n" , last_temperature_bits , last_temperature_bits / 128.0 );
		send_message_to_pc( (char *)adt7420_string , strlen(adt7420_string) );
	}
	return ( last_temperature_bits / 128.0 );
}

void adt7420_initialization()
{
	//int8_t id = I2C2_read_byte( ADT7420_SLAVE_ADDRESS , ID );
	adt7420_set_configuration();
}

void get_adt7420_sample(void)
{
	static char adt7420_string[100];
	int16_t temperature = adt7420_get_one_shot_temperature();
	sprintf( adt7420_string ,"   adt7420 captured !\r\n");
	send_message_to_pc( (char *)&temperature , 2 );
	send_message_to_pc( (char *)adt7420_string , strlen(adt7420_string) );

}

void adt7420_print_sample()
{
	static char adt7420_string[100];
	adt7420_get_one_shot_temperature();
	sprintf( adt7420_string , "ATD7420 - BITS=%d TEMPERATURE=%.2f\r\n" , last_temperature_bits , last_temperature_bits / 128.0 );
	send_message_to_pc( (char *)adt7420_string , strlen(adt7420_string) );

}

