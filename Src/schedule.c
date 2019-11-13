/*----------------------------------------------------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         											*
 *  File name  : schedule.c                                                                                        											*
 *  Abstract   : All activation handling - turning on/off, blinking etc.                                          											*
 *  Written by : Ofer Freilich                                                                                  											*
 *  Date       : JULY 2018                                                                                       											*
 *----------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "photo_diode.h"
#include "leds.h"
#include "battery.h"
#include "command.h"
#include "schedule.h"
#include "charger.h"
#include "environment_log.h"

/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define PERIPH_ENABLE							HAL_GPIO_WritePin(PERIPH_EN_GPIO_Port, PERIPH_EN_Pin, GPIO_PIN_SET  );
#define PERIPH_DISABLE							HAL_GPIO_WritePin(PERIPH_EN_GPIO_Port, PERIPH_EN_Pin, GPIO_PIN_RESET);
#define WIFI_DISABLE    						HAL_GPIO_WritePin(GPIOE, WIFI_EN_Pin, GPIO_PIN_RESET);
#define WIFI_WAKE_PIN_HIGH	  					HAL_GPIO_WritePin( GPIOE, GPIO_PIN_2 , GPIO_PIN_SET   );
#define WIFI_WAKE_PIN_LOW	        			HAL_GPIO_WritePin( GPIOE, GPIO_PIN_2 , GPIO_PIN_RESET );
#define DISABLE_SW			        			HAL_GPIO_WritePin( VBAT_SW_EN_GPIO_Port, VBAT_SW_EN_Pin , GPIO_PIN_RESET );
#define SW_INST_OP_HIGH							( HAL_GPIO_ReadPin( SYS_WKUP1___SW_INST_OP_GPIO_Port , SYS_WKUP1___SW_INST_OP_Pin ) == GPIO_PIN_SET )
#define MAXIMUM_EMPTY_TRAY_FAILURE_TRIES		5

typedef enum
{
	RTC_WAKEUP,
	NRST_PIN_RESET,
	LOW_POWER_RESET,
	SOFTWARE_RESET,
	BROWN_OUT_RESET,
	FIREWALL_RESET,
	REMOVE_RESET,
	OPTION_BYTE_LOADER_RESET
} RESET_REASON;

//typedef enum
//{
//	ALL_COUNTERS,
//	WAKE_UP_COUNTER,
//	SNOOZE_COUNTER,
//	ENVIRONMENT_SEND_COUNTER,
//	ALERT_SEND_INTERVAL_COUNTER,
//	ENVIRONMENT_LOG_COUNTER
//} COUNTER;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern SAMPLING_PARAMETERS sampling_parameters;
extern uint16_t wake_up_counter __attribute__ ((section (".ram2")));
uint16_t wake_up_counter; // variables in ram2 cannot be initialized.
extern uint16_t snooze_counter __attribute__ ((section (".ram2")));
uint16_t snooze_counter; // variables in ram2 cannot be initialized.
extern uint16_t environment_send_counter __attribute__ ((section (".ram2")));
uint16_t environment_send_counter; // variables in ram2 cannot be initialized.
extern uint16_t alert_send_interval_counter __attribute__ ((section (".ram2")));
uint16_t alert_send_interval_counter; // variables in ram2 cannot be initialized.
extern uint16_t environment_log_counter __attribute__ ((section (".ram2")));
uint16_t environment_log_counter; // variables in ram2 cannot be initialized.
extern uint16_t session_anomaly_counter __attribute__ ((section (".ram2")));
uint16_t session_anomaly_counter; // variables in ram2 cannot be initialized.
char *anomaly_reason_string[] =
{
/* WIFI_CONNECTION_FAIL */ "WIFI CONNECTION FAILED" ,
/* EMPTY_TRAY           */ "EMPTY TRAY" ,
/* TRAY_IN_MOTION       */ "TRAY IN MOTION" ,
/* TOO_SHORT_IGNITION   */ "TOO SHORT IGNITION - WILL SHUT OFF" ,
/* LOW_BATTERY			*/ "LOW BATTERY - WILL SHUT OFF" ,
/* CHARGER_REMOVED		*/ "CHARGER REMOVED - WILL SHUT OFF" ,
/* SUCCESSFUL_SHUT_DOWN */ "SUCCESSFUL SHUT DOWN"
};

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
uint16_t reset_reason = 0x0000;
void schedule_system_turn_on()
{
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) == true )
		reset_reason |= 0x01;
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) == true )
		reset_reason |= 0x02;
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) == true )
		reset_reason |= 0x04;
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) == true )
		reset_reason |= 0x08;
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_FWRST) == true )
		reset_reason |= 0x10;
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_RMVF) == true )
		reset_reason |= 0x20;
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) == true )
		reset_reason |= 0x40;
	if( __HAL_PWR_GET_FLAG(PWR_FLAG_SB) == true )
	{
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
		reset_reason |= 0x80;
	}
	if( PWR->SR1 & 0x00000100 )
	{
		PWR->SCR = 0x100;
		reset_reason |= 0x100;
	}
	__HAL_RCC_CLEAR_RESET_FLAGS();
	return;
}

bool schedule_wake_up_from_reset()
{
	//bool wake_up_from_reset = false;
	#warning "open back"
//    if (true == __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
//    {
//        return false;
//    }
	if( (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) == true) || (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) == true) )
	{
		return true;
	}
	//__HAL_RCC_CLEAR_RESET_FLAGS();
	//return wake_up_from_reset;
	return false;
}

bool schedule_wake_up_from_standby()
{
	/* Check and handle if the system was resumed from StandBy mode */
	#if 0
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
	{
	  /* Clear Standby flag */
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	}
	#endif
	if( PWR->SR1 & 0x00000100 )
	{
		PWR->SCR = 0x100;
		return true;
	}
	return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/// 									schedule_goto_sleep										  ///
/////////////////////////////////////////////////////////////////////////////////////////////////////
/// The sleep time is calculated like this:														  ///
/// BASIC_UNIT = RTC_WAKEUPCLOCK_RTCCLK_DIV / LSE_clock											  ///
/// BASIC_UNIT =              16            /   32767 											  ///
/// BASIC_UNIT = 1 / 2048 = 0.000488 seconds													  ///
/// SLEEP_TIME = BASIC_UNIT * WakeUpCounter = 0.000488 * F000 = 30 seconds						  ///
/////////////////////////////////////////////////////////////////////////////////////////////////////
void schedule_goto_sleep()
{
    LogPrintfWrapper("Go to sleep\r\n");

	float    time_base = ( 16 / 32767.0 );
	float sleep_time_register_f = ( (float)sampling_parameters.sleep_interval_6_blink / time_base );
    uint32_t sleep_time_register = (uint32_t) sleep_time_register_f;

	if (sleep_time_register > 0xFFFF) {
	    sleep_time_register = 0xffff;
	}
    LogPrintfWrapper("time_base:%f, Sleep intr blink:%d, sleep time reg:%u\r\n", time_base, sampling_parameters.sleep_interval_6_blink, sleep_time_register);

	/* Keep TURN_ON pin (PF11) in high by pull-up */
	HAL_PWREx_EnablePullUpPullDownConfig();
	HAL_PWREx_EnableGPIOPullUp( PWR_GPIO_F , PWR_GPIO_BIT_11 );

	/* Disable all used wakeup sources*/
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

	/* Clear all related wakeup flags */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	HAL_RTCEx_SetWakeUpTimer_IT( &hrtc, sleep_time_register , RTC_WAKEUPCLOCK_RTCCLK_DIV16);

	HAL_PWREx_EnableSRAM2ContentRetention();

	PERIPH_DISABLE;
	WIFI_DISABLE;
	WIFI_WAKE_PIN_LOW;
	DISABLE_SW;
	HAL_PWREx_EnableGPIOPullDown( (uint32_t)PERIPH_EN_GPIO_Port, PERIPH_EN_Pin);
	HAL_PWREx_EnableGPIOPullDown( (uint32_t)WIFI_EN_GPIO_Port  , WIFI_EN_Pin);
	HAL_PWREx_EnableGPIOPullDown( (uint32_t)WIFI_WAKE_GPIO_Port, (uint32_t)WIFI_WAKE_GPIO_Port);
	HAL_PWREx_EnableGPIOPullDown( (uint32_t)VBAT_SW_EN_GPIO_Port, (uint32_t)VBAT_SW_EN_Pin);
	/* Enter the Standby mode */
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
	__HAL_RCC_GPIOE_CLK_DISABLE();
	__HAL_RCC_GPIOF_CLK_DISABLE();
	__HAL_RCC_GPIOG_CLK_DISABLE();
	__HAL_RCC_ADC_CLK_DISABLE();
	__HAL_RCC_TIM1_CLK_DISABLE();
	__HAL_RCC_TIM2_CLK_DISABLE();
	__HAL_RCC_TIM3_CLK_DISABLE();
	__HAL_RCC_TIM4_CLK_DISABLE();
	__HAL_RCC_TIM5_CLK_DISABLE();
	__HAL_RCC_TIM6_CLK_DISABLE();
	__HAL_RCC_TIM7_CLK_DISABLE();
	__HAL_RCC_TIM8_CLK_DISABLE();
	__HAL_RCC_I2C1_CLK_DISABLE();
	__HAL_RCC_I2C2_CLK_DISABLE();
	__HAL_RCC_I2C3_CLK_DISABLE();
	__HAL_RCC_I2C4_CLK_DISABLE();
	__HAL_RCC_USART1_CLK_DISABLE();
	HAL_PWR_EnterSTANDBYMode();
}


void schedule_reset_counters(COUNTER counter)
{
	switch(counter)
	{
		case ALL_COUNTERS:
		case WAKE_UP_COUNTER:				
		    wake_up_counter = 0;
		case SNOOZE_COUNTER:				
		    snooze_counter = 0;
		case ENVIRONMENT_SEND_COUNTER:		
		    environment_send_counter = 0;
		case ALERT_SEND_INTERVAL_COUNTER:	
		    alert_send_interval_counter = 0;
		case ENVIRONMENT_LOG_COUNTER:		
		    environment_log_counter = 0;
	}
}
void schedule_initialize_counters()
{
	schedule_reset_counters(ALL_COUNTERS);
	session_anomaly_counter = 0;
}

WAKE_UP_REASON schedule_timing_conditions()
{
	static char reset_reason_string[300] = "";
	uint16_t wake_up_counter_time_unit = sampling_parameters.sleep_interval_6_blink;
	
	LogPrintfWrapperC(FONT_WHITE, "*******************************\r\n");

	if( __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) == true ) {
		LogPrintfWrapperC(FONT_WHITE, "RESET REASON: PINRST\r\n");
	}
	
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) == true ) {
		LogPrintfWrapperC(FONT_WHITE, "RESET REASON: SFTRST\r\n");
	}
	
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_FWRST) == true ) {
		strcat( reset_reason_string , "RESET REASON: FWRST\r\n");
    }
    
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) == true ) {
		strcat( reset_reason_string , "RESET REASON: OBLRST\r\n");
	}
	
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) == true ) {
		strcat( reset_reason_string , "RESET REASON: BORRST\r\n");
	}
	
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) == true ) {
		strcat( reset_reason_string , "RESET REASON: IWDGRST\r\n");
	}
	
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) == true ) {
		strcat( reset_reason_string , "RESET REASON: WWDGRST\r\n");
	}
	
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) == true ) {
		strcat( reset_reason_string , "RESET REASON: LPWRRST\r\n");

    }
	
	LogPrintfWrapperC(FONT_WHITE, "%s\r\n",reset_reason_string);	

	battery_check_low_battery();
	if( schedule_wake_up_from_reset() == true )
	{
    	send_message_to_pc("schedule_wake_up_from_reset = true\r\n", sizeof("schedule_wake_up_from_reset = true\r\n"));
		__HAL_RCC_CLEAR_RESET_FLAGS();
		PWR->SCR = 0x100;
		//schedule_wait_for_reed_switch();
		photo_diode_set_defaults();
		schedule_initialize_counters();
		environment_log_reset_log();
	}
	else
	{
	send_message_to_pc("schedule_wake_up_from_reset = false\r\n", sizeof("schedule_wake_up_from_reset = false\r\n"));
	#warning "check closed line"
	//	PWR->SCR |= PWR_SCR_CSBF; //We clear "get into Standby" flag
		battery_update_charge_led_status();
		HAL_Delay(100);
		leds_illuminate( CHARGE_LED , TURN_OFF_LED );
		wake_up_counter++;
		snooze_counter++;
		environment_send_counter++;
		alert_send_interval_counter++;
		environment_log_counter++;
		LogPrintfWrapperC(FONT_LIGHT_BLUE, "Wake up counter:\t\t%3d/%d\t\t%dsec/%dsec\r\n", 
		                                    wake_up_counter             , 
		                                    sampling_parameters.sleep_interval_1_full_sample / wake_up_counter_time_unit , 
		                                    wake_up_counter  * wake_up_counter_time_unit , 
		                                    sampling_parameters.sleep_interval_1_full_sample );
                                            LogPrintfWrapperC(FONT_LIGHT_BLUE, "Snooze counter:\t\t%3d/%d\t\t%dsec/%dsec\r\n" , 
		                                    snooze_counter, 
		                                    sampling_parameters.sleep_interval_2_snooze / wake_up_counter_time_unit , 
		                                    snooze_counter * wake_up_counter_time_unit , 
		                                    sampling_parameters.sleep_interval_2_snooze );
		LogPrintfWrapperC(FONT_LIGHT_BLUE, "Env. send counter:\t\t%3d/%d\t\t%dsec/%dsec\r\n" , 
		                                    environment_send_counter    , 
		                                    sampling_parameters.sleep_interval_3_environment_send / wake_up_counter_time_unit , 
		                                    environment_send_counter    * wake_up_counter_time_unit , 
		                                    sampling_parameters.sleep_interval_3_environment_send );
		LogPrintfWrapperC(FONT_LIGHT_BLUE, "Alert send interval:\t\t%3d/%d\t\t%dsec/%dsec\r\n" , 
		                                    alert_send_interval_counter , 
		                                    sampling_parameters.sleep_interval_4_alert            / wake_up_counter_time_unit , 
		                                    alert_send_interval_counter * wake_up_counter_time_unit , 
		                                    sampling_parameters.sleep_interval_4_alert            );
		LogPrintfWrapperC(FONT_LIGHT_BLUE, "Env. log counter:\t\t%3d/%d\t\t%dsec/%dsec\r\n" , 
		                                    environment_log_counter     , 
		                                    sampling_parameters.sleep_interval_5_environment_log  / wake_up_counter_time_unit , 
		                                    environment_log_counter     * wake_up_counter_time_unit , 
		                                    sampling_parameters.sleep_interval_5_environment_log  );


		//#warning "test function reset"
		//test_func();
		if( sampling_parameters.sleep_interval_1_full_sample / wake_up_counter_time_unit <= wake_up_counter )
		{
			schedule_reset_counters(WAKE_UP_COUNTER);
			return FROM_SLEEP;
		}
		if( sampling_parameters.sleep_interval_2_snooze / wake_up_counter_time_unit <= snooze_counter )
		{
			schedule_reset_counters(SNOOZE_COUNTER);
			return FROM_SNOOZE;
		}
		if( sampling_parameters.sleep_interval_3_environment_send / wake_up_counter_time_unit <= environment_send_counter )
		{
			schedule_reset_counters(ENVIRONMENT_SEND_COUNTER);
			return FROM_ENVIRONMENT_SEND;
		}
		if( sampling_parameters.sleep_interval_4_alert / wake_up_counter_time_unit <= alert_send_interval_counter )
		{
//			schedule_reset_counters(ALERT_SEND_INTERVAL_COUNTER);
			return FROM_ALERT_SEND_INTERVAL;
		}
		if( sampling_parameters.sleep_interval_5_environment_log / wake_up_counter_time_unit == environment_log_counter )
		{
			environment_log_update_record();
			schedule_reset_counters(ENVIRONMENT_LOG_COUNTER);
			return FROM_ENVIRONMENT_LOG;
		}
		LogPrintfWrapper( "schedule_timing_conditions:Go to sleep\r\n");
		schedule_goto_sleep();
	}
	return NORMAL_WAKE_UP;
}

void schedule_reset_session_anomaly_counter()
{
	session_anomaly_counter = 0;
}


void schedule_turn_off_power()
{
	leds_illuminate_blinking( INDICATION_LED , RED , BLINKING_RATE_5_HZ , 50 );
	leds_illuminate_blinking( CHARGE_LED     , RED , BLINKING_RATE_5_HZ , 50 );
	HAL_Delay(2000);
	leds_illuminate( INDICATION_LED , TURN_OFF_LED );
	leds_illuminate( CHARGE_LED     , TURN_OFF_LED );
	HAL_Delay(10000);
	HAL_PWREx_DisablePullUpPullDownConfig();
	HAL_PWREx_DisableGPIOPullUp( PWR_GPIO_F , PWR_GPIO_BIT_11 );
	TURN_OFF_POWER;
}

void schedule_session_anomaly(ANOMALY_REASON anomaly_reason)
{
	static uint8_t session_anomaly_string[100];
	uint8_t sleep_time_in_minutes_after_session_anomaly[] = { 2 , 4 , 8 , 16 , 32 , 60 , 120 };
	//uint8_t sleep_time_in_minutes_after_session_anomaly[] = { 1 , 2 , 3 , 4 };
	leds_illuminate( INDICATION_LED , RED );
	if( session_anomaly_counter < sizeof(sleep_time_in_minutes_after_session_anomaly) ) {
		sampling_parameters.sleep_interval_1_full_sample = 60 * sleep_time_in_minutes_after_session_anomaly[session_anomaly_counter++];
	} else {
		sampling_parameters.sleep_interval_1_full_sample = 60 * sleep_time_in_minutes_after_session_anomaly[sizeof(sleep_time_in_minutes_after_session_anomaly) - 1];
    }
    LogPrintfWrapperC(FONT_LIGHT_YELLOW, "Sleep Interval=%d\r\n", sampling_parameters.sleep_interval_1_full_sample);
    LogPrintfWrapperC(FONT_LIGHT_YELLOW, "Anormaly Reason=%s\r\n", anomaly_reason_string[anomaly_reason] );
    
	switch(anomaly_reason)
	{
		case TRAY_IN_MOTION:
		case WIFI_CONNECTION_FAIL:	schedule_reset_counters(ALL_COUNTERS);
									schedule_goto_sleep();
									break;

		case EMPTY_TRAY:
		case TOO_SHORT_IGNITION:
		case LOW_BATTERY:
		case CHARGER_REMOVED:
		case SUCCESSFUL_SHUT_DOWN:  schedule_turn_off_power();
									break;
	}
}

void schdule_enable_shut_off_by_magnet()
{
	//return ; //@EliWatchdog
	GPIO_InitTypeDef GPIO_InitStruct;

	/*Configure GPIO pin : SYS_WKUP1___SW_INST_OP_Pin */
	GPIO_InitStruct.Pin = SYS_WKUP1___SW_INST_OP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(SYS_WKUP1___SW_INST_OP_GPIO_Port, &GPIO_InitStruct);
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 2);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void schedule_wait_for_reed_switch()
{
	uint16_t time_for_ignitation = 750; // in miliseconds
	COLOR    color = TURN_OFF_LED;
	uint16_t reed_switch_counter = 0 , maximum_waiting_for_reed_switch_counter = 0;

	if( schedule_wake_up_from_reset() == true )
	{
		while(1)
		{
			if( reed_switch_counter % 25 == 0 )
			{
				leds_illuminate( INDICATION_LED , color );
				leds_illuminate( CHARGE_LED     , color );
				if( color != TURN_OFF_LED )
					color = TURN_OFF_LED;
				else
				{
					if( reed_switch_counter < time_for_ignitation / 3 )
						color = RED;
					else
					{
						if( reed_switch_counter < 2 * time_for_ignitation / 3 )
							color = ORANGE;
						else
							color = GREEN;
					}
				}
			}
			if( charger_is_during_charging() == true )
				return;
			if( SW_INST_OP_HIGH == true )
			{
				if( ++reed_switch_counter > ( time_for_ignitation / 2 ) )
				{
					schdule_enable_shut_off_by_magnet();
					return;
				}
			}
			else
				reed_switch_counter = 0;
			if( ++maximum_waiting_for_reed_switch_counter > time_for_ignitation )
			{
				if( SW_INST_OP_HIGH == true )
					maximum_waiting_for_reed_switch_counter = 0;
				else
					schedule_session_anomaly(TOO_SHORT_IGNITION);
			}
			HAL_Delay(1);
		}
	}
}

void __attribute__((optimize("O0"))) EXTI0_IRQHandler(void)
{
	uint16_t time_for_ignitation = 750; // in miliseconds
	COLOR    color = TURN_OFF_LED;
	uint16_t reed_switch_counter = 0 , maximum_waiting_for_reed_switch_counter = 0;

	leds_set_configuration_update_flag(false);
	while(1)
	{
		if( reed_switch_counter % 25 == 0 )
		{
			leds_illuminate( INDICATION_LED , color );
			leds_illuminate( CHARGE_LED     , color );
			if( color != TURN_OFF_LED )
				color = TURN_OFF_LED;
			else
			{
				if( reed_switch_counter < time_for_ignitation / 2 )
					color = ORANGE;
				else
					color = RED;
			}
		}
		if( charger_is_during_charging() == true )
			return;
		if( SW_INST_OP_HIGH == true )
		{
			if( ++reed_switch_counter > ( time_for_ignitation / 2 ) )
			{
				schedule_session_anomaly(SUCCESSFUL_SHUT_DOWN);
				return;
			}
		}
		else
			reed_switch_counter = 0;
		if( ++maximum_waiting_for_reed_switch_counter > time_for_ignitation )
		{
			if( SW_INST_OP_HIGH == true )
				maximum_waiting_for_reed_switch_counter = 0;
			else
			{
				leds_restoring_status();
				break;
			}
		}
		for( uint16_t delay_index = 0 ; delay_index < 20000 ; delay_index++ );
	}
	leds_set_configuration_update_flag(true);
	NVIC_ClearPendingIRQ(EXTI0_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
}

