/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : leds.c                                                                                         *
 *  Abstract   : Leds handling driver.                                                                          *
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : JUNE 2018                                                                                      *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "leds.h"
#include "battery.h"
#include "command.h"
#include "photo_diode.h"
#include "schedule.h"

/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define OVER_SAMPLING						5
#define BATTERY_ADC_SAMPLES_BUFFER_SIZE		64
#define MINIMUM_VOLTAGE_FOR_GREEN_LED       4.0
#define MINIMUM_VOLTAGE_FOR_ORANGE_LED      3.6
#define MINIMUM_VOLTAGE					    3.4
#define V_REFERENCE							5.0
#define MAXIMUM_ADC_VALUE					( (float)0x0FFF )

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc2;
ADC_HandleTypeDef * battery_hadc = &hadc2;
DMA_HandleTypeDef * battery_hdma_adc = &hdma_adc2;
volatile bool battery_sampling_ready;
extern volatile bool photo_diode_sampling_ready;
ADC_ChannelConfTypeDef   sConfig_adc_battery;
uint16_t battery_adc_samples[1 << OVER_SAMPLING];
//uint16_t *battery_adc_samples;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void battery_initialization()
{
	sConfig_adc_battery.Channel      = ADC_CHANNEL_1;       		/* Sampled channel number */
	sConfig_adc_battery.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
	sConfig_adc_battery.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
	sConfig_adc_battery.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
	sConfig_adc_battery.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
	sConfig_adc_battery.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
	if (HAL_ADC_ConfigChannel( battery_hadc , &sConfig_adc_battery ) != HAL_OK)
	{
	    Error_Handler();
	}
}

uint32_t battery_calculate_sample_avarage()
{
	uint32_t samples_sum = 0;

	for( uint8_t sample_index = 0 ; sample_index < ( 0x0001 << OVER_SAMPLING ) ; sample_index++ )
		samples_sum += battery_adc_samples[sample_index];
	return( samples_sum >> OVER_SAMPLING );
}

uint16_t battery_get_sampling_adc_bits()
{
	//uint16_t battery_adc_sample;

	//battery_adc_samples = malloc( 4 * ( 0x0001 << OVER_SAMPLING ) );
	battery_initialization();
	battery_sampling_ready = false;
	sConfig_adc_battery.Channel = ADC_CHANNEL_1;
	HAL_ADC_Start_DMA( battery_hadc , (uint32_t *)&battery_adc_samples , 0x0001 << OVER_SAMPLING );
	while( battery_sampling_ready == false );
	HAL_ADC_Stop_DMA(battery_hadc);
	return battery_calculate_sample_avarage();
	//battery_adc_sample = battery_calculate_sample_avarage();
	//free(battery_adc_samples);
	//return battery_adc_sample;
}

float battery_get_sampling_voltage()
{
	return( V_REFERENCE * battery_get_sampling_adc_bits() / MAXIMUM_ADC_VALUE );
}

float battery_calculate_sampling_voltage(uint16_t adc_bits)
{
	return( V_REFERENCE * adc_bits / MAXIMUM_ADC_VALUE );
}

COLOR battery_get_charging_led_color()
{
	float battery_voltage = battery_get_sampling_voltage();
	if( battery_voltage >= MINIMUM_VOLTAGE_FOR_GREEN_LED  )
		return GREEN;
	if( battery_voltage >= MINIMUM_VOLTAGE_FOR_ORANGE_LED )
		return ORANGE;
	return RED;
}

void battery_update_charge_led_status()
{
	leds_illuminate( CHARGE_LED , battery_get_charging_led_color() );
}

void battery_get_battery_measurement()
{
 uint16_t battery_sampling_adc_bits = battery_get_sampling_adc_bits();
    LogPrintfWrapperC(FONT_LIGHT_CYAN, "%d %d\n" , (uint16_t)( battery_calculate_sampling_voltage(battery_sampling_adc_bits) * 100 ) , battery_sampling_adc_bits );
}

void battery_check_low_battery()
{
	if( battery_get_sampling_voltage() < MINIMUM_VOLTAGE ) {
	
		schedule_session_anomaly(LOW_BATTERY);
	}
}

/**
* @brief This function handles DMA2 channel4 global interrupt.
*/
void DMA2_Channel4_IRQHandler(void)
{
	if( __HAL_DMA_GET_FLAG(&hdma_adc2, DMA_FLAG_TC4) == DMA_FLAG_TC4 )
	{
		  HAL_DMA_IRQHandler(&hdma_adc2);
		  battery_sampling_ready = true;
	}
	else
		__HAL_DMA_CLEAR_FLAG(&hdma_adc2, DMA_FLAG_HT1);
}

