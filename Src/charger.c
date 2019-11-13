/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : charger.c                                                                                      *
 *  Abstract   : Charge handling driver.                                                                        *
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : JUNE 2018                                                                                      *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "command.h"
#include "leds.h"
#include "battery.h"
#include "charger.h"
#include "photo_diode.h"
#include "schedule.h"
#include "user_interface.h"

/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define CHARGER_ENABLE_1_LOW  			HAL_GPIO_WritePin( CHARGE_EN1_GPIO_Port , CHARGE_EN1_Pin , GPIO_PIN_RESET );
#define CHARGER_ENABLE_1_HIGH  			HAL_GPIO_WritePin( CHARGE_EN1_GPIO_Port , CHARGE_EN1_Pin , GPIO_PIN_SET   );
#define CHARGER_ENABLE_1_STATE 			( HAL_GPIO_ReadPin( CHARGE_EN1_GPIO_Port , CHARGE_EN1_Pin ) == GPIO_PIN_SET )
#define CHARGER_ENABLE_2_LOW  			HAL_GPIO_WritePin( CHARGE_EN2_GPIO_Port , CHARGE_EN2_Pin , GPIO_PIN_RESET );
#define CHARGER_ENABLE_2_HIGH  			HAL_GPIO_WritePin( CHARGE_EN2_GPIO_Port , CHARGE_EN2_Pin , GPIO_PIN_SET   );
#define CHARGER_ENABLE_2_STATE 			( HAL_GPIO_ReadPin( CHARGE_EN2_GPIO_Port , CHARGE_EN2_Pin ) == GPIO_PIN_SET )
#define CHARGER_CONTROL_LOW  			HAL_GPIO_WritePin( CHARGE_CONTROL_GPIO_Port , CHARGE_CONTROL_Pin , GPIO_PIN_RESET );
#define CHARGER_CONTROL_HIGH  			HAL_GPIO_WritePin( CHARGE_CONTROL_GPIO_Port , CHARGE_CONTROL_Pin , GPIO_PIN_SET   );
#define CHARGER_CONTROL_SET(gpio_state) HAL_GPIO_WritePin( CHARGE_CONTROL_GPIO_Port , CHARGE_CONTROL_Pin , gpio_state );
#define CHARGER_CONTROL_STATE 			( HAL_GPIO_ReadPin( CHARGE_CONTROL_GPIO_Port , CHARGE_CONTROL_Pin ) == GPIO_PIN_SET )
#define BATTERY_CHARGING	  			( HAL_GPIO_ReadPin( EXTI4___CHARGE_STATUS_GPIO_Port , EXTI4___CHARGE_STATUS_Pin ) == GPIO_PIN_RESET )
#define CHARGE_PGOOD   		  			( HAL_GPIO_ReadPin( CHARGE_PGOODn_GPIO_Port , CHARGE_PGOODn_Pin ) == GPIO_PIN_RESET )

#define OVER_SAMPLING					5

typedef struct
{
	uint8_t battery_is_charging		 	: 1;
	uint8_t valid_input_power_source 	: 1;
	uint8_t charging_setting			: 2;
	uint8_t charging_control			: 1;
} CHARGE_STATUS_STRUCT;

typedef union
{
	uint8_t					charge_struct_byte;
	CHARGE_STATUS_STRUCT 	charge_struct_bits;
} CHARGE_STATUS;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc3;
extern volatile bool photo_diode_sampling_ready;
ADC_ChannelConfTypeDef   sConfig_adc_charger;
uint16_t charger_adc_samples[1 << OVER_SAMPLING];

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void charger_set_setting(CHARGE_RATE charger_seting)
{
	switch(charger_seting)
	{
		case CHARGE_RATE_100MA:		CHARGER_ENABLE_1_HIGH;
									CHARGER_ENABLE_2_LOW;
									break;

		case CHARGE_RATE_500MA:		CHARGER_ENABLE_1_LOW;
									CHARGER_ENABLE_2_LOW;
									break;

		case CHARGE_RATE_1500MA:	CHARGER_ENABLE_1_HIGH;
									CHARGER_ENABLE_2_HIGH;
									break;

		case CHARGE_RATE_STANDBY:	CHARGER_ENABLE_1_LOW;
									CHARGER_ENABLE_2_HIGH;
									break;
	}
}

void charger_get_setting()
{
	CHARGE_STATUS charger_status = { 0x00 };
	char charge_status_string[] = "0\n";

	if( BATTERY_CHARGING == true ) {
		charger_status.charge_struct_bits.battery_is_charging = true;
	}
	if( CHARGE_PGOOD == true )
		charger_status.charge_struct_bits.valid_input_power_source = true;
	if( CHARGER_ENABLE_1_STATE == false )
		charger_status.charge_struct_bits.charging_setting |= 0x01;
	if( CHARGER_ENABLE_2_STATE == true )
		charger_status.charge_struct_bits.charging_setting |= 0x02;
	if( CHARGER_CONTROL_STATE == true )
		charger_status.charge_struct_bits.charging_control = true;

	LogPrintfWrapper("%d\n", charger_status.charge_struct_byte );
}

void charger_set_charge_control(char gpio_state)
{
	CHARGER_CONTROL_SET( atoi(&gpio_state) );
}

void charger_iset_configuration()
{
	sConfig_adc_charger.Channel      = ADC_CHANNEL_13;              /* Sampled channel number */
	sConfig_adc_charger.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
	sConfig_adc_charger.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
	sConfig_adc_charger.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
	sConfig_adc_charger.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
	sConfig_adc_charger.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig_adc_charger) != HAL_OK)
	{
	    Error_Handler();
	}
}

uint32_t charger_calculate_sample_avarage()
{
	uint32_t samples_sum = 0;

	for( uint8_t sample_index = 0 ; sample_index < ( 0x0001 << OVER_SAMPLING ) ; sample_index++ )
		samples_sum += charger_adc_samples[sample_index];
	return( samples_sum >> OVER_SAMPLING );
}

uint16_t charger_get_sampling_adc_bits()
{
 uint32_t counter = 0;
	charger_iset_configuration();
	photo_diode_sampling_ready = false;
	sConfig_adc_charger.Channel = ADC_CHANNEL_9;
	
	if (HAL_OK != HAL_ADC_Start_DMA( &hadc3 , (uint32_t *)&charger_adc_samples , 0x0001 << OVER_SAMPLING ))
	{
	    LogPrintfWrapperC(FONT_LIGHT_RED, "charger_get_sampling_adc_bits DMA fail to start\r\n");
	    return 0;
	}
	
	while( photo_diode_sampling_ready == false ) {
	    HAL_Delay(1);
	    if (++counter > 3000) {
	        LogPrintfWrapperC(FONT_LIGHT_RED, "charger_get_sampling_adc_bits DMA error\r\n");
	        break;
	    }
	}
	HAL_ADC_Stop_DMA(&hadc3);
	return charger_calculate_sample_avarage();
}

float charger_calculate_sampling_voltage(uint16_t adc_bits)
{
	return( 5.0 * adc_bits / (float)0x0FFF );
}

void charger_get_iset_analog_measurement()
{
  uint16_t charger_sampling_adc_bits = charger_get_sampling_adc_bits();
	LogPrintfWrapper("%d %d\n" , (uint16_t)( charger_calculate_sampling_voltage(charger_sampling_adc_bits) * 100 ) , charger_sampling_adc_bits);
}

bool charger_is_during_charging()
{
	return( ( CHARGE_PGOOD || BATTERY_CHARGING ) ? true : false );
}

void __attribute__((optimize("O0"))) charger_handle_charging()
{
	volatile CHARGE_STATUS previous_charger_status = { 0xFF } , charger_status = { 0x00 };
	//uint16_t maximum_debouncing_counter = 0 , debouncing_counter = 0;
    //char charger_string[30];

	if( CHARGE_PGOOD == true )
	{
		leds_illuminate( INDICATION_LED , TURN_OFF_LED );
		charger_set_setting(CHARGE_RATE_1500MA);
		while(1)
		{
		    user_interface_recieved_polling();
			charger_status.charge_struct_byte = 0x00;
			if( BATTERY_CHARGING == true ) {
				charger_status.charge_struct_bits.battery_is_charging = true;
			}
				
			if( CHARGE_PGOOD == true ) {
				charger_status.charge_struct_bits.valid_input_power_source = true;
			}
				
			if( charger_status.charge_struct_byte != previous_charger_status.charge_struct_byte )
			{
				previous_charger_status.charge_struct_byte = charger_status.charge_struct_byte;
				switch(charger_status.charge_struct_byte)
				{	// Disconnecting charger - Turn off device.
					case 0x00: 	
					    schedule_session_anomaly(CHARGER_REMOVED);
						break;

					// This state should never happen - 3Hz blinking just in case.
					case 0x01: 	
					    leds_illuminate_blinking( CHARGE_LED , RED , BLINKING_RATE_3_HZ , 50 );
						break;

					// Charging completed - 5Hz blinking rate.
					case 0x02: 	
					    leds_illuminate_blinking( CHARGE_LED , battery_get_charging_led_color() , BLINKING_RATE_5_HZ , 50 );
						break;

					// During charging - 1HZ blinking rate.
					case 0x03:	
					    leds_illuminate_blinking( CHARGE_LED , battery_get_charging_led_color() , BLINKING_RATE_1_HZ , 50 );
						break;

					default:   	
					    break;
				}
			    //sprintf( charger_string ,"st=%d\n" , (int)charger_status.charge_struct_byte );
				//send_message_to_pc( charger_string , strlen(charger_string) );
			    //HAL_UART_Transmit( &huart1 , (uint8_t *)charger_string , strlen(charger_string)  , 2000 );
			}
			//previous_charger_status.charge_struct_byte = charger_status.charge_struct_byte;
		}
	}
}
