/*----------------------------------------------------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         											*
 *  File name  : photo_diode.c                                                                                        										*
 *  Abstract   : Photo diode leds and detectors module.                                                  													*
 *  Written by : Ofer Freilich                                                                                  											*
 *  Date       : MAY 2018                                                                                       											*
 *----------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "stm32l4xx_hal.h"
#include "i2c.h"
#include "macronix_flash.h"
#include "photo_diode.h"
#include "command.h"
#include "leds.h"
#include "battery.h"
#include "schedule.h"
#include "charger.h"
#include "tray_id.h"
#include "sensors.h"
#include "lis3dshtr.h"
#include <math.h>

/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define HARDWARE_REV_2
#define ADC_V_REF									3.3f
#define ADC_SIZE_BITS								12
#define ADC_SAMPLES_BUFFER_SIZE						512
#define LEDS_SAMPLES_SIZE							( NUMBER_OF_PHOTO_DIODES * sizeof(uint16_t) )
#define PHOTO_DIODE_SELECT( gpio , pin_number )    	HAL_GPIO_WritePin( gpio , pin_number , GPIO_PIN_SET   )
#define PHOTO_DIODE_DESELECT( gpio , pin_number )   HAL_GPIO_WritePin( gpio , pin_number , GPIO_PIN_RESET )
#define ENABLE_LEDS									HAL_GPIO_WritePin( GPIOG , VBAT_SW_EN_Pin , GPIO_PIN_SET   );
#define DISABLE_LEDS								HAL_GPIO_WritePin( GPIOG , VBAT_SW_EN_Pin , GPIO_PIN_RESET );
#define ROUND(x)									((x) + 0.5f)
#define IABS(a)										(((a) ^ ((a) < 0 ? -1 : 0)) - ((a) < 0 ? -1 : 0))
#define MIN(a,b)									( ( a < b ) ? a : b )
#define IRRELEVANT									0
#define REG_MAX(numBits)							((1 << (numBits)) -1)
#define SECTOR_64KB_SIZE							0x10000
#define SAMPLE_BYTES_SIZE							2
#define SAMPLE_SIZE_PER_LED							2
#define SAW_TOOTH_MAXIMUM_DAC_VALUE_TRANSFER		400
#define SAW_TOOTH_MAXIMUM_DAC_VALUE_REFLECTION		4
#define MINIMUM_EGGS_ON_TRAY						3
#define BOARD_TYPE_BIT								0x10
#define REFLECTION_DUMMY_ADC_SAMPLE					365
//#define OVERSAMPLING_FACTOR							7

typedef enum
{
	GENDER_COMM_MODE	= 0xED,
	GENDER_TEST_MODE 	= 0xEE,
	GENDER_MODE    		= 0xEF,
	TEST_COMM_MODE 		= 0xFD,
	TEST_UNIT_MODE 		= 0xFE,
	REGULAR_MODE   		= 0xFF
} MODE;

typedef struct
{
	GPIO_TypeDef	*gpio;
	uint16_t	  	pin_number;
	void			*adc;
	uint32_t		adc_channel_number;
	void			*adc_instance;
} PHOTO_DIODE_STRUCT;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern DAC_HandleTypeDef hdac1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern IWDG_HandleTypeDef hiwdg; //@Eliwatchdog
uint32_t sample_index = 0;
uint16_t adc_samples[ADC_SAMPLES_BUFFER_SIZE];
//uint16_t *adc_samples;
bool start_next_sampling = true;
volatile bool photo_diode_sampling_ready = true;
uint32_t ms_timer = 0;
DETECTOR_TYPE working_point_leds_dac_value[NUMBER_OF_PHOTO_DIODES] = { { 0 , 0 } , { 1 , 1 } , { 2 , 2 } , { 3 , 3 } , { 4 , 4 } , { 5 , 5 } , { 6 , 6 } , { 7 , 7 } , { 8 , 8 } , { 9 ,9 } , { 10 , 10 } , { 11 , 11 } , { 12 , 12 } , { 13 , 13 } , { 14 , 14 } , { 15 , 15 } };
const PHOTO_DIODE_STRUCT  photo_diode_routing_table[NUMBER_OF_PHOTO_DIODES] =
{
/* PHOTO_DIODE_0  */ { SELECT1_1_GPIO_Port , SELECT1_1_Pin , &hadc1 , ADC_CHANNEL_16 , ADC1 } ,
/* PHOTO_DIODE_1  */ { SELECT1_2_GPIO_Port , SELECT1_2_Pin , &hadc1 , ADC_CHANNEL_13 , ADC1 } ,
/* PHOTO_DIODE_2  */ { SELECT1_3_GPIO_Port , SELECT1_3_Pin , &hadc1 , ADC_CHANNEL_14 , ADC1 } ,
/* PHOTO_DIODE_3  */ { SELECT1_4_GPIO_Port , SELECT1_4_Pin , &hadc1 , ADC_CHANNEL_15 , ADC1 } ,
/* PHOTO_DIODE_4  */ { SELECT2_1_GPIO_Port , SELECT2_1_Pin , &hadc1 , ADC_CHANNEL_2  , ADC1 } ,
/* PHOTO_DIODE_5  */ { SELECT2_2_GPIO_Port , SELECT2_2_Pin , &hadc3 , ADC_CHANNEL_10 , ADC3 } ,
/* PHOTO_DIODE_6  */ { SELECT2_3_GPIO_Port , SELECT2_3_Pin , &hadc3 , ADC_CHANNEL_12 , ADC3 } ,
/* PHOTO_DIODE_7  */ { SELECT2_4_GPIO_Port , SELECT2_4_Pin , &hadc3 , ADC_CHANNEL_11 , ADC3 } ,
/* PHOTO_DIODE_8  */ { SELECT3_1_GPIO_Port , SELECT3_1_Pin , &hadc1 , ADC_CHANNEL_7  , ADC1 } ,
/* PHOTO_DIODE_9  */ { SELECT3_2_GPIO_Port , SELECT3_2_Pin , &hadc1 , ADC_CHANNEL_6  , ADC1 } ,
/* PHOTO_DIODE_10 */ { SELECT3_3_GPIO_Port , SELECT3_3_Pin , &hadc1 , ADC_CHANNEL_4  , ADC1 } ,
/* PHOTO_DIODE_11 */ { SELECT3_4_GPIO_Port , SELECT3_4_Pin , &hadc1 , ADC_CHANNEL_3  , ADC1 } ,
/* PHOTO_DIODE_12 */ { SELECT4_1_GPIO_Port , SELECT4_1_Pin , &hadc1 , ADC_CHANNEL_11 , ADC1 } ,
/* PHOTO_DIODE_13 */ { SELECT4_2_GPIO_Port , SELECT4_2_Pin , &hadc1 , ADC_CHANNEL_8  , ADC1 } ,
/* PHOTO_DIODE_14 */ { SELECT4_3_GPIO_Port , SELECT4_3_Pin , &hadc1 , ADC_CHANNEL_10 , ADC1 } ,
/* PHOTO_DIODE_15 */ { SELECT4_4_GPIO_Port , SELECT4_4_Pin , &hadc1 , ADC_CHANNEL_12 , ADC1 }
};

extern SAMPLING_PARAMETERS sampling_parameters __attribute__ ((section (".ram2")));//RAM2 is retained in standby mode
SAMPLING_PARAMETERS sampling_parameters;
SAMPLING_PARAMETERS sampling_parameters_default =
{
/* number_of_photo_diodes                     */ NUMBER_OF_PHOTO_DIODES,
/* photo_diodes_order[NUMBER_OF_PHOTO_DIODES] */ { PHOTO_DIODE_0 , PHOTO_DIODE_0 , PHOTO_DIODE_1 , PHOTO_DIODE_1 , PHOTO_DIODE_2 , PHOTO_DIODE_2 , PHOTO_DIODE_3 , PHOTO_DIODE_3 , PHOTO_DIODE_4 , PHOTO_DIODE_4 , PHOTO_DIODE_5 , PHOTO_DIODE_5 , PHOTO_DIODE_6 , PHOTO_DIODE_6 , PHOTO_DIODE_7 , PHOTO_DIODE_7 , PHOTO_DIODE_8 , PHOTO_DIODE_8 , PHOTO_DIODE_9 , PHOTO_DIODE_9 , PHOTO_DIODE_10 , PHOTO_DIODE_10 , PHOTO_DIODE_11 , PHOTO_DIODE_11 , PHOTO_DIODE_12 , PHOTO_DIODE_12 , PHOTO_DIODE_13 , PHOTO_DIODE_13 , PHOTO_DIODE_14 , PHOTO_DIODE_14 , PHOTO_DIODE_15 , PHOTO_DIODE_15 },
/* working_point                              */ 16000,
/* delay_before_dark_sample_us                */ 5,
/* delay_before_light_sample_us               */ 25,
/* oversampling_factor                        */ 5,
/* measurement_size_bits                      */ 16,
/* sample_interval_us                         */ 6000,
/* samples_per_package                        */ 32,
/* number_of_packages                         */ 100,//0x0400,
/* mode                                       */ GENDER_TEST_MODE,
/* maximum_temperature_in_celsios             */ 194,
/* minimum_temperature_in_celsios             */ 0,
/* WAKE_UP_REASON                             */ NORMAL_WAKE_UP,
/* sleep_interval_1_full_sample               */ 10000,
/* sleep_interval_2_snooze                    */ 30000,
/* sleep_interval_3_environment_send          */ 30000,
/* sleep_interval_4_alert                     */ 30000,
/* sleep_interval_5_environment_log           */ 30000,
/* sleep_interval_6_blink                     */ 10,
/* maximum_dac_value	                      */ 2500
};
#if 0
SAMPLING_PARAMETERS sampling_parameters_MIN =
{
/* number_of_photo_diodes                     */ PHOTO_DIODE_0,
/* photo_diodes_order[NUMBER_OF_PHOTO_DIODES] */ { PHOTO_DIODE_0 , PHOTO_DIODE_0 , PHOTO_DIODE_1 , PHOTO_DIODE_1 , PHOTO_DIODE_2 , PHOTO_DIODE_2 , PHOTO_DIODE_3 , PHOTO_DIODE_3 , PHOTO_DIODE_4 , PHOTO_DIODE_4 , PHOTO_DIODE_5 , PHOTO_DIODE_5 , PHOTO_DIODE_6 , PHOTO_DIODE_6 , PHOTO_DIODE_7 , PHOTO_DIODE_7 , PHOTO_DIODE_8 , PHOTO_DIODE_8 , PHOTO_DIODE_9 , PHOTO_DIODE_9 , PHOTO_DIODE_10 , PHOTO_DIODE_10 , PHOTO_DIODE_11 , PHOTO_DIODE_11 , PHOTO_DIODE_12 , PHOTO_DIODE_12 , PHOTO_DIODE_13 , PHOTO_DIODE_13 , PHOTO_DIODE_14 , PHOTO_DIODE_14 , PHOTO_DIODE_15 , PHOTO_DIODE_15 },
/* working_point                              */ 0,
/* delay_before_dark_sample_us                */ 1,
/* delay_before_light_sample_us               */ 1,
/* oversampling_factor                        */ 0,
/* measurement_size_bits                      */ 0,
/* sample_interval_us                         */ 500,
/* samples_per_package                        */ 1,
/* number_of_packages                         */ 1,//0x0400,
/* mode                                       */ FIRST_MODE,
/* maximum_temperature_in_celsios             */ 0,
/* minimum_temperature_in_celsios             */ 0,
/* WAKE_UP_REASON                             */ NORMAL_WAKE_UP,
/* sleep_interval_1_full_sample               */ 0,
/* sleep_interval_2_snooze                    */ 0,
/* sleep_interval_3_environment_send          */ 0,
/* sleep_interval_4_alert                     */ 0,
/* sleep_interval_5_environment_log           */ 0,
/* sleep_interval_6_blink                     */ 0,
/* maximum_dac_value	                      */ 0
};
SAMPLING_PARAMETERS sampling_parameters_MAX =
{
/* number_of_photo_diodes                     */ 64,
/* photo_diodes_order[NUMBER_OF_PHOTO_DIODES] */ { PHOTO_DIODE_0 , PHOTO_DIODE_0 , PHOTO_DIODE_1 , PHOTO_DIODE_1 , PHOTO_DIODE_2 , PHOTO_DIODE_2 , PHOTO_DIODE_3 , PHOTO_DIODE_3 , PHOTO_DIODE_4 , PHOTO_DIODE_4 , PHOTO_DIODE_5 , PHOTO_DIODE_5 , PHOTO_DIODE_6 , PHOTO_DIODE_6 , PHOTO_DIODE_7 , PHOTO_DIODE_7 , PHOTO_DIODE_8 , PHOTO_DIODE_8 , PHOTO_DIODE_9 , PHOTO_DIODE_9 , PHOTO_DIODE_10 , PHOTO_DIODE_10 , PHOTO_DIODE_11 , PHOTO_DIODE_11 , PHOTO_DIODE_12 , PHOTO_DIODE_12 , PHOTO_DIODE_13 , PHOTO_DIODE_13 , PHOTO_DIODE_14 , PHOTO_DIODE_14 , PHOTO_DIODE_15 , PHOTO_DIODE_15 },
/* working_point                              */ 32000,
/* delay_before_dark_sample_us                */ 255,
/* delay_before_light_sample_us               */ 255,
/* oversampling_factor                        */ 50,
/* measurement_size_bits                      */ 49,
/* sample_interval_us                         */ 65535,
/* samples_per_package                        */ 1000,
/* number_of_packages                         */ 1000,//0x0400,
/* mode                                       */ REGULAR_MODE,
/* maximum_temperature_in_celsios             */ 250,
/* minimum_temperature_in_celsios             */ 250,
/* WAKE_UP_REASON                             */ FROM_BLINK,
/* sleep_interval_1_full_sample               */ 65535,
/* sleep_interval_2_snooze                    */ 65535,
/* sleep_interval_3_environment_send          */ 65535,
/* sleep_interval_4_alert                     */ 65535,
/* sleep_interval_5_environment_log           */ 65535,
/* sleep_interval_6_blink                     */ 100,
/* maximum_dac_value	                      */ 4096
};
#endif

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void photo_diode_initialization()
{
	GPIOC->MODER &= 0xFFFF7FFF;
	GPIOC->MODER |= 0x01 << 14;
    macronix_flash_enter_4byte_address_mode();
    if( tray_id_get_tray_type() == REGULAR_TRAY )
    {
        sampling_parameters.samples_per_package    = 32;
        sampling_parameters.number_of_photo_diodes = 16;
        sampling_parameters.mode				  |= BOARD_TYPE_BIT;
    }
    else
    {
        sampling_parameters.samples_per_package    = 16;
        sampling_parameters.number_of_photo_diodes = 32;
        sampling_parameters.mode				  &= ~BOARD_TYPE_BIT;
    }
    photo_diode_start_sampling(sampling_parameters.mode);
}

void photo_diode_set_defaults()
{
	memcpy( &sampling_parameters , &sampling_parameters_default , sizeof(SAMPLING_PARAMETERS) );
}

void delay_us(uint16_t delay_time)
{
	//HAL_GPIO_WritePin( GPIOC , GPIO_PIN_7 , GPIO_PIN_SET );
	if( delay_time > 1000 )
		HAL_Delay(delay_time / 1000 );
	else
	{
		uint16_t timeout = ( (39 * delay_time) );
		TIM6->CNT = 0;
		__HAL_TIM_ENABLE(&htim6);
		while( TIM6->CNT < timeout );
		__HAL_TIM_DISABLE(&htim6);
		//__HAL_TIM_SET_PRESCALER(&htim6,  (39 * delay_time) >> 2 );
	}
	//HAL_GPIO_WritePin( GPIOC , GPIO_PIN_7 , GPIO_PIN_RESET );
}

void photo_diode_activate( PHOTO_DIODE photo_diode , uint16_t photo_diode_level)
{
	PHOTO_DIODE_SELECT( photo_diode_routing_table[photo_diode].gpio , photo_diode_routing_table[photo_diode].pin_number );
	for( uint8_t precentages = 0 ; precentages <= 100 ; precentages += 25 )
	{
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)( precentages * photo_diode_level ) / 100 );
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	}
}

void photo_diode_deactivate(PHOTO_DIODE photo_diode)
{
	PHOTO_DIODE_DESELECT( photo_diode_routing_table[photo_diode].gpio , photo_diode_routing_table[photo_diode].pin_number );
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0 );
	HAL_DAC_Stop( &hdac1 , DAC_CHANNEL_1 );
}

int32_t photo_diode_bits_shifting(int32_t value_to_shift)
{
	int32_t samples_sum_shift = ADC_SIZE_BITS + sampling_parameters.oversampling_factor - sampling_parameters.measurement_size_bits;

	if( samples_sum_shift >= 0 )
		return( value_to_shift >> samples_sum_shift );
	else
		return( value_to_shift << -samples_sum_shift );
}

int32_t photo_diode_bits_back_shifting(int32_t value_to_shift)
{
	int32_t samples_sum_shift = ADC_SIZE_BITS + sampling_parameters.oversampling_factor - sampling_parameters.measurement_size_bits;

	if( samples_sum_shift >= 0 )
		return( value_to_shift << samples_sum_shift );
	else
		return( value_to_shift >> -samples_sum_shift );
}

/*uint32_t photo_diode_calculate_sample_avarage(uint8_t oversampling_factor)
{
	uint32_t samples_sum = 0;

    for( uint16_t sample_index = 0 ; sample_index < ( 0x0001 << oversampling_factor ) ; sample_index++ )
		samples_sum += adc_samples[sample_index];
    return photo_diode_bits_shifting(samples_sum);
}*/

uint32_t photo_diode_calculate_samples_sum(uint8_t oversampling_factor)
{
	uint32_t samples_sum = 0;
	uint16_t maximum_sample = 0 , minimum_sample = 4096;

    for( uint16_t sample_index = 0 ; sample_index < ( 0x0001 << oversampling_factor ) + 2 ; sample_index++ )
    {
    	if( adc_samples[sample_index] > maximum_sample )
    		maximum_sample       = adc_samples[sample_index];
    	if( adc_samples[sample_index] < minimum_sample )
    		minimum_sample       = adc_samples[sample_index];
    }
    for( uint16_t sample_index = 0 ; sample_index < ( 0x0001 << oversampling_factor ) + 2 ; sample_index++ )
    		samples_sum += adc_samples[sample_index];
    samples_sum -= ( maximum_sample + minimum_sample);
    return samples_sum;
}

uint16_t photo_diode_get_clean_sample( int32_t in_light_sampling , int32_t in_dark_sampling )
{
	int32_t clean_sample = in_light_sampling - in_dark_sampling;

	clean_sample = ( clean_sample > 0 ) ? clean_sample : 0;
	if(0)//( clean_sample < ( 100 << sampling_parameters.oversampling_factor ) || clean_sample > ( 3800 << sampling_parameters.oversampling_factor ) )       //defective LED
		return 1;
	else
		return photo_diode_bits_shifting(clean_sample);
}

uint16_t photo_diode_get_clean_sample_for_test( int32_t in_light_sampling , int32_t in_dark_sampling )
{
	int32_t clean_sample = in_light_sampling - in_dark_sampling;
	clean_sample = ( clean_sample > 0 ) ? clean_sample : 0;
	return photo_diode_bits_shifting(clean_sample);
}

uint32_t __attribute__((optimize("O0"))) photo_diode_get_led_sampling( PHOTO_DIODE photo_diode_led , PHOTO_DIODE photo_diode_detector , uint16_t dac_level , uint8_t oversampling_factor , uint16_t led_on_time )
{
	uint16_t counter = 0;

	photo_diode_sampling_ready = false;
	// set adc channel
	//LL_ADC_REG_SetSequencerRanks( (ADC_TypeDef *)photo_diode_routing_table[photo_diode].adc_instance , ADC_REGULAR_RANK_1, photo_diode_routing_table[photo_diode].adc_channel_number);
	LL_ADC_REG_SetSequencerRanks( ( (ADC_HandleTypeDef *)photo_diode_routing_table[photo_diode_detector].adc == &hadc1 ) ? ADC1 : ADC3 , ADC_REGULAR_RANK_1, photo_diode_routing_table[photo_diode_detector].adc_channel_number);
	if( dac_level > 0 )
		photo_diode_activate( photo_diode_led , dac_level );
	delay_us(led_on_time);
	HAL_GPIO_WritePin( GPIOC , GPIO_PIN_7 , GPIO_PIN_SET );
	HAL_ADC_Start_DMA( photo_diode_routing_table[photo_diode_detector].adc , (uint32_t *)&adc_samples , ( 0x0001 << oversampling_factor ) + 2 );
	while( photo_diode_sampling_ready == false && counter++ < 4000 );
	HAL_ADC_Stop_DMA(photo_diode_routing_table[photo_diode_detector].adc);
	HAL_GPIO_WritePin( GPIOC , GPIO_PIN_7 , GPIO_PIN_RESET );
	if( dac_level > 0 )
		photo_diode_deactivate(photo_diode_led);
	if(counter >= 4000)
		return 0;
	return photo_diode_calculate_samples_sum(oversampling_factor);
}

void __attribute__((optimize("O0"))) photo_diode_leds_sampling(MODE mode)
{
	#define MAXIMUM_CHANGE_X 700
	#define MAXIMUM_CHANGE_Y 700
	#define MAXIMUM_CHANGE_Z 700

	static int32_t in_dark_sampling , in_light_transferring_sampling , in_light_reflection_sampling;// , clean_sample;
	static uint32_t sampling_num;//@EliWatchdog
	uint8_t  samples_size =  ( ( tray_id_get_tray_type() == REGULAR_TRAY ) ? 32 : 64 );
	volatile uint8_t  eggs_number  =  ( ( tray_id_get_tray_type() == REGULAR_TRAY ) ? 16 :  8 );
	uint32_t flash_address = 0;
	static uint16_t clean_samples[2 * NUMBER_OF_PHOTO_DIODES];
    static SAMPLE axes_before_leds_sampling , axes_after_leds_sampling;
	//const uint8_t  index_table[8] = { 0 , 1 , 2 , 3 , 6 , 7 , 10 , 11 };
//	const uint8_t  transferring_index_table[16] = { 0 , 2 , 4 , 2 , 8 , 6 , 12 , 10 };
	//const uint8_t  reflection_index_table[16] = { 0 , 1 , 5 , 3 , 9 , 7 , 13 , 11 };
	#ifdef HARDWARE_REV_2
		const uint8_t  transferring_index_table[8] = { 0 , 2 , 4 , 6 , 8 , 10 , 12 , 14 };
		const uint8_t  reflection_index_table[8]   = { 0 , 0 , 1 , 3 , 5 , 7  ,  9 , 11 };
	#else
		const uint8_t  transferring_index_table[8] = { 0 , 1 , 4 , 5 , 8 , 9 , 12 , 13 };
		const uint8_t  reflection_index_table[8]   = { 0 , 0 , 2 , 3 , 6 , 7 , 10 , 11 };
	#endif
	volatile uint16_t dac_level = 0;

    lis3dshtr_get_accelerometer_sample(&axes_before_leds_sampling);
	__HAL_TIM_ENABLE(&htim7);
	__HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
	//adc_samples = malloc( ( 0x0001 << sampling_parameters.oversampling_factor ) * sizeof(uint16_t) );
	ms_timer = 0;
	TIM7->CNT = 0;
	start_next_sampling = true;
	sampling_num = sampling_parameters.samples_per_package * sampling_parameters.number_of_packages;//@EliWatchdog
	//for( uint32_t sample_index = 0 ; sample_index < sampling_parameters.samples_per_package * sampling_parameters.number_of_packages && charger_is_during_charging() == false ; sample_index++ )
	for( uint32_t sample_index = 0 ; (sample_index < sampling_num) && (charger_is_during_charging() == false) ; sample_index++ )
	{
		while(start_next_sampling == false);
		start_next_sampling = false;
		//HAL_GPIO_TogglePin( GPIOC , GPIO_PIN_7 );
		for( PHOTO_DIODE photo_diode = PHOTO_DIODE_0 ; photo_diode < eggs_number ; photo_diode++ )
		{
			switch(mode)
			{
				case GENDER_TEST_MODE:	in_dark_sampling               = photo_diode_get_led_sampling( photo_diode * 2     , transferring_index_table[photo_diode] , 0 					                                       , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
										in_light_transferring_sampling = photo_diode_get_led_sampling( photo_diode * 2     , transferring_index_table[photo_diode] , ( dac_level % SAW_TOOTH_MAXIMUM_DAC_VALUE_TRANSFER ) + 50 , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
										//in_dark_sampling               = photo_diode_get_led_sampling( PHOTO_DIODE_6     , photo_diode * 2 , 0 					                                       , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
										//in_light_transferring_sampling = photo_diode_get_led_sampling( PHOTO_DIODE_6     , photo_diode * 2 , ( dac_level % SAW_TOOTH_MAXIMUM_DAC_VALUE_TRANSFER ) + 50 , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
										clean_samples[photo_diode * 4    ] = photo_diode_get_clean_sample_for_test( in_light_transferring_sampling , in_dark_sampling );
										delay_us(300);
										#ifdef REFLECTION_ACTIVATED
											in_dark_sampling               = photo_diode_get_led_sampling( photo_diode * 2 , reflection_index_table[photo_diode]  , 0 					                                       , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
											in_light_reflection_sampling   = photo_diode_get_led_sampling( photo_diode * 2 , reflection_index_table[photo_diode]  , ( dac_level % SAW_TOOTH_MAXIMUM_DAC_VALUE_REFLECTION )    , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
											clean_samples[photo_diode * 4 + 1] = photo_diode_get_clean_sample_for_test( in_light_reflection_sampling   , in_dark_sampling );
											delay_us(300);
										#else
											clean_samples[photo_diode * 4 + 1] = REFLECTION_DUMMY_ADC_SAMPLE;
										#endif
										in_dark_sampling               = photo_diode_get_led_sampling( photo_diode * 2 + 1 , transferring_index_table[photo_diode] , 0 					                                       , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
										in_light_transferring_sampling = photo_diode_get_led_sampling( photo_diode * 2 + 1 , transferring_index_table[photo_diode] , ( dac_level % SAW_TOOTH_MAXIMUM_DAC_VALUE_TRANSFER ) + 50 , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
										//in_dark_sampling               = photo_diode_get_led_sampling( PHOTO_DIODE_6 , photo_diode * 2 + 1 , 0 					                                       , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
										//in_light_transferring_sampling = photo_diode_get_led_sampling( PHOTO_DIODE_6 , photo_diode * 2 + 1 , ( dac_level % SAW_TOOTH_MAXIMUM_DAC_VALUE_TRANSFER ) + 50 , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
										clean_samples[photo_diode * 4 + 2] = photo_diode_get_clean_sample_for_test( in_light_transferring_sampling , in_dark_sampling );
										#ifdef REFLECTION_ACTIVATED
											delay_us(300);
											in_dark_sampling               = photo_diode_get_led_sampling( photo_diode * 2 + 1 , reflection_index_table[photo_diode] , 0 					                                       , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
											in_light_reflection_sampling   = photo_diode_get_led_sampling( photo_diode * 2 + 1 , reflection_index_table[photo_diode] , ( dac_level % SAW_TOOTH_MAXIMUM_DAC_VALUE_REFLECTION )    , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
											clean_samples[photo_diode * 4 + 3] = photo_diode_get_clean_sample_for_test( in_light_reflection_sampling   , in_dark_sampling );
										#else
											clean_samples[photo_diode * 4 + 3] = REFLECTION_DUMMY_ADC_SAMPLE;
										#endif
										if( photo_diode <= PHOTO_DIODE_1 )
										{
											clean_samples[photo_diode * 4 + 1] = 3;
											clean_samples[photo_diode * 4 + 3] = 4;
										}
										break;

				case GENDER_MODE:		in_dark_sampling               = photo_diode_get_led_sampling( photo_diode * 2     , transferring_index_table[photo_diode] , 0 										                       , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
										in_light_transferring_sampling = photo_diode_get_led_sampling( photo_diode * 2     , transferring_index_table[photo_diode] , working_point_leds_dac_value[photo_diode * 2    ].transferring_detector , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
										//clean_samples[photo_diode * 4    ] = in_light_transferring_sampling;
										clean_samples[photo_diode * 4    ] = photo_diode_get_clean_sample( in_light_transferring_sampling , in_dark_sampling );
										delay_us(300);
										#ifdef REFLECTION_ACTIVATED
											in_dark_sampling               = photo_diode_get_led_sampling( photo_diode * 2 , reflection_index_table[photo_diode] , 0 										                       , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
											in_light_reflection_sampling   = photo_diode_get_led_sampling( photo_diode * 2 , reflection_index_table[photo_diode] , working_point_leds_dac_value[photo_diode * 2    ].reflection_detector   , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
											//clean_samples[photo_diode * 4 + 1] = in_light_reflection_sampling;
											clean_samples[photo_diode * 4 + 1] = photo_diode_get_clean_sample( in_light_reflection_sampling   , in_dark_sampling );
											delay_us(300);
										#else
											clean_samples[photo_diode * 4 + 1] = REFLECTION_DUMMY_ADC_SAMPLE;
										#endif
										in_dark_sampling               = photo_diode_get_led_sampling( photo_diode * 2 + 1 , transferring_index_table[photo_diode] , 0 										                       , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
										in_light_transferring_sampling = photo_diode_get_led_sampling( photo_diode * 2 + 1 , transferring_index_table[photo_diode] , working_point_leds_dac_value[photo_diode * 2 + 1].transferring_detector , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
										//clean_samples[photo_diode * 4 + 2] = in_light_transferring_sampling;
										clean_samples[photo_diode * 4 + 2] = photo_diode_get_clean_sample( in_light_transferring_sampling , in_dark_sampling );
										#ifdef REFLECTION_ACTIVATED
											delay_us(300);
											in_dark_sampling               = photo_diode_get_led_sampling( photo_diode * 2 + 1 , reflection_index_table[photo_diode] , 0 										                       , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
											in_light_reflection_sampling   = photo_diode_get_led_sampling( photo_diode * 2 + 1 , reflection_index_table[photo_diode] , working_point_leds_dac_value[photo_diode * 2 + 1].reflection_detector   , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
											//clean_samples[photo_diode * 4 + 3] = in_light_reflection_sampling;
											clean_samples[photo_diode * 4 + 3] = photo_diode_get_clean_sample( in_light_reflection_sampling   , in_dark_sampling );
										#else
											clean_samples[photo_diode * 4 + 3] = REFLECTION_DUMMY_ADC_SAMPLE;
										#endif
										break;

				case GENDER_COMM_MODE:	clean_samples[photo_diode * 4    ] = ( photo_diode + ( photo_diode << 8 )     );
										clean_samples[photo_diode * 4 + 1] = ( photo_diode + ( photo_diode << 8 ) + 1 );
										clean_samples[photo_diode * 4 + 2] = ( photo_diode + ( photo_diode << 8 ) + 2 );
										clean_samples[photo_diode * 4 + 3] = ( photo_diode + ( photo_diode << 8 ) + 3 );
										break;

				case TEST_COMM_MODE:	clean_samples[photo_diode] = ( photo_diode + ( photo_diode << 8 ) );
										break;

				case TEST_UNIT_MODE: 	in_light_transferring_sampling = photo_diode_get_led_sampling( photo_diode , photo_diode , dac_level % SAW_TOOTH_MAXIMUM_DAC_VALUE_TRANSFER , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
										in_dark_sampling               = photo_diode_get_led_sampling( photo_diode , photo_diode , 0 									   			, sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
										clean_samples[photo_diode]     = photo_diode_get_clean_sample_for_test( in_light_transferring_sampling , in_dark_sampling );
										break;

				case REGULAR_MODE:		in_light_transferring_sampling = photo_diode_get_led_sampling( photo_diode , photo_diode , working_point_leds_dac_value[photo_diode].transferring_detector , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
										in_dark_sampling  			   = photo_diode_get_led_sampling( photo_diode , photo_diode , 0 										                       , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
										clean_samples[photo_diode]     = photo_diode_get_clean_sample( in_light_transferring_sampling , in_dark_sampling );
										break;
			}
		}
		dac_level++;
		macronix_flash_write_data_to_flash( flash_address , (uint8_t *)clean_samples , samples_size );
		flash_address += samples_size;
	}
	__HAL_TIM_DISABLE(&htim7);
	__HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
    lis3dshtr_get_accelerometer_sample(&axes_after_leds_sampling);
	if( fabs( axes_after_leds_sampling.x_axis - axes_before_leds_sampling.x_axis ) > MAXIMUM_CHANGE_X || fabs( axes_after_leds_sampling.y_axis - axes_before_leds_sampling.y_axis ) > MAXIMUM_CHANGE_Y || fabs( axes_after_leds_sampling.z_axis - axes_before_leds_sampling.z_axis ) > MAXIMUM_CHANGE_Z )
		schedule_session_anomaly(TRAY_IN_MOTION);
}

/**
* @brief This function handles DMA2 channel3 global interrupt.
*/
void DMA2_Channel3_IRQHandler(void)
{
	if( __HAL_DMA_GET_FLAG(&hdma_adc1, DMA_FLAG_TC3) == DMA_FLAG_TC3 )
	{
		  HAL_DMA_IRQHandler(&hdma_adc1);
		  photo_diode_sampling_ready = true;
	}
	else
		__HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_HT1);
}

/**
* @brief This function handles DMA2 channel5 global interrupt.
*/
void DMA2_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel5_IRQn 0 */
	if( __HAL_DMA_GET_FLAG(&hdma_adc3, DMA_FLAG_TC5) == DMA_FLAG_TC5 )
	{
		  HAL_DMA_IRQHandler(&hdma_adc3);
		  photo_diode_sampling_ready = true;
	}
	else
		__HAL_DMA_CLEAR_FLAG(&hdma_adc3, DMA_FLAG_HT3);
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim7);
  //HAL_GPIO_TogglePin( GPIOC , GPIO_PIN_7 );
  //__HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
  if( ++ms_timer >= ( sampling_parameters.sample_interval_us / 1000 ) )
  {
	  ms_timer = 0;
	  start_next_sampling = true;
	  TIM7->CNT = 0;
  }
}

void __attribute__((optimize("O0"))) photo_diode_find_multi_working_point(MODE mode)
{
	volatile uint16_t low_dac_value = 0 , high_dac_value = MIN( sampling_parameters.maximum_dac_value + 1 , 4000 );
	volatile int32_t in_dark_sampling = 0 , in_light_sampling = 0 , clean_sample = 0;
	volatile uint16_t dac_value = 0;
	const uint16_t adc_detector_tolerance = 320;
	uint16_t working_point = sampling_parameters.working_point;
	static char working_point_string[200];
	PHOTO_DIODE photo_diode_detector;
	//uint8_t  transferring_index_table[16] = { 0 , 0 , 1 , 1 , 4 , 4 , 5 , 5 , 8 , 8 , 9 , 9 , 12 , 12 , 13 , 13 };
//	uint8_t  transferring_index_table[16] = { 0 , 0 , 1 , 1 , 4 , 4 , 2 , 2 , 8 , 8 , 6 , 6 , 12 , 12 , 10 , 10 };
#ifdef HARDWARE_REV_2
	uint8_t  transferring_index_table[16] = { 0 , 0 , 2 , 2 , 4 , 4 , 6 , 6 , 8 , 8 , 10 , 10 , 12 , 12 , 14 , 14 };
#else
	uint8_t  transferring_index_table[16] = { 0 , 0 , 1 , 1 , 4 , 4 , 5 , 5 , 8 , 8 , 9 , 9 , 12 , 12 , 13 , 13 };
#endif

	for( PHOTO_DIODE photo_diode = PHOTO_DIODE_0 ; photo_diode < NUMBER_OF_PHOTO_DIODES ; photo_diode++ )
	{
	    if( tray_id_get_tray_type() == REGULAR_TRAY )
			photo_diode_detector = photo_diode;
		else
			photo_diode_detector = transferring_index_table[photo_diode];
		low_dac_value = 0;
		high_dac_value = sampling_parameters.maximum_dac_value + 1;
		do
		{
			dac_value = ( ( low_dac_value + high_dac_value ) / 2 );
			in_dark_sampling  = photo_diode_get_led_sampling( photo_diode , photo_diode_detector , 0 	     , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_dark_sample_us  );
			in_light_sampling = photo_diode_get_led_sampling( photo_diode , photo_diode_detector , dac_value , sampling_parameters.oversampling_factor , sampling_parameters.delay_before_light_sample_us );
			clean_sample = photo_diode_get_clean_sample( in_light_sampling , in_dark_sampling );
			clean_sample = clean_sample > 0 ? clean_sample : 0;
			working_point_leds_dac_value[photo_diode].transferring_detector = dac_value;
			if( IABS( working_point - clean_sample ) < adc_detector_tolerance )
				break;
			if( low_dac_value == high_dac_value )
				break;
			if( clean_sample < working_point )
				low_dac_value = dac_value;
			else
				high_dac_value = dac_value;
			//delay_us(300);
			delay_us(1000);
			sprintf( working_point_string , "LED%2d-DET%2d: offset=%4d workingPoint=%d in_dark_sampling=%d in_light_sampling=%d clean_sample=%d clean_sample(BEFORE)=%d\r\n" , photo_diode + 1 , photo_diode_detector + 1 , dac_value , working_point , (int)(in_dark_sampling) , (int)(in_light_sampling) , (int)clean_sample , (int)(in_light_sampling - in_dark_sampling) );
			send_message_to_pc( (char *)working_point_string , strlen(working_point_string) );
		} while( dac_value > 1 && dac_value < sampling_parameters.maximum_dac_value );
		sprintf( working_point_string , "LED%2d-DET%2d: offset=%4d workingPoint=%d in_dark_sampling=%d in_light_sampling=%d clean_sample=%d clean_sample(BEFORE)=%d\r\n" , photo_diode + 1 , photo_diode_detector + 1 , dac_value , working_point , (int)(in_dark_sampling) , (int)(in_light_sampling) , (int)clean_sample , (int)(in_light_sampling - in_dark_sampling) );
		send_message_to_pc( (char *)working_point_string , strlen(working_point_string) );
	}
}

void photo_diode_start_sampling(uint8_t mode)
{
	//macronix_flash_read_data_from_flash( 0 , (uint8_t *)aaa , 20000 );
	uint32_t sampling_num;//@EliWatchdog
	sampling_num = sampling_parameters.number_of_packages * sampling_parameters.samples_per_package * sampling_parameters.number_of_photo_diodes * SAMPLE_BYTES_SIZE ;
	//for( uint32_t flash_address = 0 ; flash_address < sampling_parameters.number_of_packages * sampling_parameters.samples_per_package * sampling_parameters.number_of_photo_diodes * SAMPLE_BYTES_SIZE ; flash_address += SECTOR_64KB_SIZE )
	for( uint32_t flash_address = 0 ; flash_address < sampling_num; flash_address += SECTOR_64KB_SIZE )
		macronix_flash_block_64k_erase(flash_address);
	leds_illuminate( INDICATION_LED , GREEN );
	leds_illuminate( CHARGE_LED     , battery_get_charging_led_color() );
	ENABLE_LEDS;
	photo_diode_find_multi_working_point(mode);
	photo_diode_leds_sampling(mode);
	DISABLE_LEDS;
}

void photo_diode_set_wake_up_reason(WAKE_UP_REASON wake_up_reason)
{
	sampling_parameters.wake_up_reason = wake_up_reason;
}

void fill_samples_parameters(uint8_t *sampling_parameters_pointer)
{
	memcpy( sampling_parameters_pointer , &sampling_parameters , sizeof(SAMPLING_PARAMETERS) );
}

void photo_diode_set_sampling_parameters(uint8_t *new_sampling_parameters)
{
	memcpy( (uint8_t *)&sampling_parameters , new_sampling_parameters , sizeof(SAMPLING_PARAMETERS) );
}

uint16_t photo_diode_get_sampling_data_payload_size()
{
	return ( sampling_parameters.samples_per_package * sampling_parameters.number_of_photo_diodes * 2 );
}

void photo_diode_get_sampling_parameters()
{
	LogPrintfWrapper("%d %d %d\n" , sampling_parameters.number_of_photo_diodes , sampling_parameters.samples_per_package , sampling_parameters.number_of_packages);
}

void photo_diode_copy_working_points_dac_value(DETECTOR_TYPE *working_point_dac_value_pointer)
{
	for( PHOTO_DIODE photo_diode = PHOTO_DIODE_0 ; photo_diode < NUMBER_OF_PHOTO_DIODES ; photo_diode++ )
	{
		working_point_dac_value_pointer[photo_diode].transferring_detector = working_point_leds_dac_value[photo_diode].transferring_detector;
		working_point_dac_value_pointer[photo_diode].reflection_detector   = working_point_leds_dac_value[photo_diode].reflection_detector;
	}
}

void photo_diode_get_samples(uint32_t packet_number)
{
	static uint8_t packet_samples_string[2048];
	uint16_t packet_size = 2 * sampling_parameters.number_of_photo_diodes * sampling_parameters.samples_per_package;
	if (packet_size > 2048) {
	    packet_size = 2048;
	}
	macronix_flash_read_data_from_flash( packet_number * packet_size , (uint8_t *)&packet_samples_string , packet_size );
	send_message_to_pc( (char *)packet_samples_string , packet_size );
}

uint8_t photo_diode_eggs_on_tray_counting()
{
	uint8_t  eggs_counter = 0;
	int32_t  sample;
	uint16_t working_point = 30;
	static char eggs_on_tray_string[200];

	ENABLE_LEDS;
	for( PHOTO_DIODE photo_diode = PHOTO_DIODE_0 ; photo_diode < NUMBER_OF_PHOTO_DIODES ; photo_diode++ )
	{
		sample = photo_diode_get_led_sampling( photo_diode , photo_diode , working_point , sampling_parameters.oversampling_factor , 50 ) >> sampling_parameters.oversampling_factor;
		if( sample < 4070 )
			eggs_counter++;
		sprintf( eggs_on_tray_string , "PHOTO DIODE %d : sample = %d\r\n" , photo_diode + 1 , (int)sample );
		send_message_to_pc( (char *)eggs_on_tray_string , strlen(eggs_on_tray_string) );
	}
	DISABLE_LEDS;
	sprintf( eggs_on_tray_string , "THE TRAY HAS %d EGG%c ON IT\r\n" , eggs_counter , eggs_counter == 1 ? ' ' : 'S' );
	send_message_to_pc( (char *)eggs_on_tray_string , strlen(eggs_on_tray_string) );
	return eggs_counter;
}

bool photo_diode_is_tray_loaded()
{
	return( ( photo_diode_eggs_on_tray_counting() >= MINIMUM_EGGS_ON_TRAY ) ? true : false );
}

void photo_diode_check_tray_loading_status()
{
	if( photo_diode_is_tray_loaded() == false )
		schedule_session_anomaly(EMPTY_TRAY);
}

bool photo_diode_check_temperature_boundaries(float temperature)
{
	if( temperature <= ( sampling_parameters.maximum_temperature_in_celsios / 5.0 ) && temperature >= ( sampling_parameters.minimum_temperature_in_celsios / 5.0 ) )
	{
		return true;
	}
	else
	{
		return false;
	}
}
