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

/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define PULSE_PERIOD_VALUE	4716

typedef enum
{
	RED_LED,
	GREEN_LED,

	LEDS_PER_FUNCTION
} LED_COLOR;

typedef struct
{
	GPIO_TypeDef	*gpio;
	uint16_t	  	pin_number;
} GPIO_PIN;
/*
typedef struct
{
	GPIO_PIN red_led;
	GPIO_PIN green_led;
	uint8_t  alternate_function_red_led;
	uint8_t  alternate_function_green_led;
} LEDS;
*/

typedef struct
{
	GPIO_PIN 	gpio;
	uint8_t  	alternate_function;
	TIM_TypeDef *timer;
	uint32_t    channel;
} LEDS_PINOUT;

typedef struct
{
	LEDS_PINOUT red_led;
	LEDS_PINOUT green_led;
} LEDS;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#if OLD_BOARD
//#if 1
const LEDS leds[NUMBER_OF_LEDS] =
{//                              { { {               PORT               ,             GPIO             } , ALTERNATE FUN , TIMER,     CHANNEL   } } ,
/* CHARGE_LED     - RED LED   */ { { { TIM3_CH1___CHARGE_LED_R_GPIO_Port , TIM3_CH1___CHARGE_LED_R_Pin } , GPIO_AF2_TIM3 , TIM3 , TIM_CHANNEL_1 } ,
/* 				  - GREEN LED */   { { TIM3_CH2___CHARGE_LED_G_GPIO_Port , TIM3_CH2___CHARGE_LED_G_Pin } , GPIO_AF2_TIM3 , TIM3 , TIM_CHANNEL_2 } } ,
/* INDICATION_LED - RED LED   */ { { { TIM1_CH1___IND_LED1_GPIO_Port     , TIM1_CH1___IND_LED1_Pin     } , GPIO_AF1_TIM1 , TIM1 , TIM_CHANNEL_1 } ,
/*                - GREEN LED */   { { TIM8_CH4___IND_LED2_GPIO_Port     , TIM8_CH4___IND_LED2_Pin     } , GPIO_AF3_TIM8 , TIM8 , TIM_CHANNEL_4 } }
};
#else
const LEDS leds[NUMBER_OF_LEDS] =
{//                              { { {               PORT               ,             GPIO             } , ALTERNATE FUN , TIMER,     CHANNEL   } } ,
/* CHARGE_LED     - RED LED   */ { { { TIM3_CH1___CHARGE_LED_R_GPIO_Port , TIM3_CH1___CHARGE_LED_R_Pin } , GPIO_AF2_TIM3 , TIM3 , TIM_CHANNEL_1 } ,
/* 				  - GREEN LED */   { { TIM3_CH2___CHARGE_LED_G_GPIO_Port , TIM3_CH2___CHARGE_LED_G_Pin } , GPIO_AF2_TIM3 , TIM3 , TIM_CHANNEL_2 } } ,
/* INDICATION_LED - RED LED   */ { { { TIM1_CH1___IND_LED1_GPIO_Port     , TIM1_CH1___IND_LED1_Pin     } , GPIO_AF1_TIM1 , TIM1 , TIM_CHANNEL_1 } ,
/*                - GREEN LED */   { { TIM1_CH2___IND_LED2_GPIO_Port     , TIM1_CH2___IND_LED2_Pin     } , GPIO_AF1_TIM1 , TIM1 , TIM_CHANNEL_2 } }
};
#endif

LED_CONFIGURATION last_led_configuration[NUMBER_OF_LEDS] =
{
/* CHARGE_LED     */ { TURN_OFF_LED , NO_BLINKING } ,
/* INDICATION_LED */ { TURN_OFF_LED , NO_BLINKING }
};

bool leds_update_configuragion_enable = true;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void leds_configure_led_state( const LEDS_PINOUT *colored_led , GPIO_PinState led_state )
{
	static GPIO_InitTypeDef GPIO_InitStruct;

	HAL_GPIO_WritePin( colored_led->gpio.gpio , colored_led->gpio.pin_number , led_state );
	GPIO_InitStruct.Pin = colored_led->gpio.pin_number;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init( colored_led->gpio.gpio , &GPIO_InitStruct );
}

void __attribute__((optimize("O0"))) leds_configure_blinking_led( const LEDS_PINOUT *colored_led , uint16_t blinking_rate , uint8_t duty_cycle )
{
	static GPIO_InitTypeDef   GPIO_InitStruct;
    static TIM_HandleTypeDef  TimHandle;   /* Timer handler declaration */
    static TIM_OC_InitTypeDef sConfig;     /* Timer Output Compare Configuration Structure declaration */

    GPIO_InitStruct.Pin = colored_led->gpio.pin_number;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = colored_led->alternate_function;
    HAL_GPIO_Init( colored_led->gpio.gpio , &GPIO_InitStruct );

    TimHandle.Instance = colored_led->timer;
    TimHandle.Instance->CR1 &= ~(TIM_CR1_UDIS);
    TimHandle.Init.Prescaler         = 1000000;//uhPrescalerValue;
    TimHandle.Init.Period            = PULSE_PERIOD_VALUE / blinking_rate;
    TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV4;
    TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    TimHandle.Init.RepetitionCounter = 0;
    TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&TimHandle);

    /*##-2- Configure the PWM channels #########################################*/
    /* Configuration for channel 3 */
    sConfig.OCMode       = TIM_OCMODE_PWM1;
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfig.OCFastMode   = TIM_OCFAST_ENABLE;//TIM_OCFAST_DISABLE;
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

    /* Set the pulse value for channel 1 */
    sConfig.Pulse = (uint32_t)( PULSE_PERIOD_VALUE / ( blinking_rate * 2 ) );
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, colored_led->channel);

    /*##-1- Start PWM signals generation #######################################*/
    /* Start channel 3 */
    HAL_TIM_PWM_Start(&TimHandle, colored_led->channel);
}

void leds_update_last_configuration( LED led , COLOR color , uint16_t blinking_rate )
{
	if( leds_update_configuragion_enable == true )
	{
		last_led_configuration[led].color         = color;
		last_led_configuration[led].blinking_rate = blinking_rate;
	}
}

void leds_illuminate( LED led , COLOR color )
{
	switch(color)
	{
		case TURN_OFF_LED:  leds_configure_led_state( &leds[led].red_led   , GPIO_PIN_RESET );
							leds_configure_led_state( &leds[led].green_led , GPIO_PIN_RESET );
							break;

		case RED:			leds_configure_led_state( &leds[led].red_led   , GPIO_PIN_SET   );
							leds_configure_led_state( &leds[led].green_led , GPIO_PIN_RESET );
							break;

		case ORANGE:		leds_configure_led_state( &leds[led].red_led   , GPIO_PIN_SET   );
							leds_configure_led_state( &leds[led].green_led , GPIO_PIN_SET   );
							break;

		case GREEN:			leds_configure_led_state( &leds[led].red_led   , GPIO_PIN_RESET );
							leds_configure_led_state( &leds[led].green_led , GPIO_PIN_SET   );
							break;
	}
	leds_update_last_configuration( led , color , NO_BLINKING );
}

void leds_illuminate_blinking( LED led , COLOR color , uint16_t blinking_rate , uint8_t duty_cycle )
{
	switch(color)
	{
		case TURN_OFF_LED:  
		    leds_configure_led_state(    &leds[led].red_led   , GPIO_PIN_RESET );
			leds_configure_led_state(    &leds[led].green_led , GPIO_PIN_RESET );
			break;

		case RED:			
		    leds_configure_blinking_led( &leds[led].red_led   , blinking_rate , duty_cycle);
			leds_configure_led_state(    &leds[led].green_led , GPIO_PIN_RESET );
			break;

		case ORANGE:		
		    leds_configure_blinking_led( &leds[led].red_led   , blinking_rate , duty_cycle );
			leds_configure_blinking_led( &leds[led].green_led , blinking_rate , duty_cycle );
			break;

		case GREEN:			
		    leds_configure_led_state(    &leds[led].red_led   , GPIO_PIN_RESET );
		    leds_configure_blinking_led( &leds[led].green_led , blinking_rate , duty_cycle );
			break;
	}
	leds_update_last_configuration( led , color , blinking_rate );
}

void leds_set_configuration_update_flag(bool enable_status)
{
	leds_update_configuragion_enable = enable_status;
}

void leds_illuminate_blinking_single_channel( LED led , COLOR color , uint16_t blinking_rate , uint8_t duty_cycle )
{
	if( color == RED ) {
		leds_configure_blinking_led( &leds[led].red_led   , blinking_rate , duty_cycle);
	} else {
		leds_configure_blinking_led( &leds[led].green_led , blinking_rate , duty_cycle );
	}
	leds_update_last_configuration( led , color , blinking_rate );
}

void led_charge_configuration(LED_CONFIGURATION led_configuration)
{
	leds_illuminate_blinking( CHARGE_LED     , led_configuration.color , led_configuration.blinking_rate , 50 );
}

void led_indication_configuration(LED_CONFIGURATION led_configuration)
{
	leds_illuminate_blinking( INDICATION_LED , led_configuration.color , led_configuration.blinking_rate , 50 );
}

void leds_restoring_status()
{
	if( last_led_configuration[CHARGE_LED].blinking_rate == NO_BLINKING ) {
		leds_illuminate( CHARGE_LED , last_led_configuration[CHARGE_LED].color );
	}
	else
	{
		leds_illuminate_blinking( CHARGE_LED     , last_led_configuration[CHARGE_LED].color     , last_led_configuration[CHARGE_LED].blinking_rate     , 50 );
	}
	if( last_led_configuration[INDICATION_LED].blinking_rate == NO_BLINKING )
	{
		leds_illuminate( INDICATION_LED , last_led_configuration[INDICATION_LED].color );
	}
	else
	{
		leds_illuminate_blinking( INDICATION_LED , last_led_configuration[INDICATION_LED].color , last_led_configuration[INDICATION_LED].blinking_rate , 50 );
	}
}

