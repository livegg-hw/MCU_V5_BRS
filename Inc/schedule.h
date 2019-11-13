/*
 * schedule.h
 *
 *  Created on: 9 αιεμι 2018
 *      Author: oferf
 */

#ifndef SCHEDULE_H_
#define SCHEDULE_H_

#define TURN_ON_POWER		HAL_GPIO_WritePin( S_H_D_N_GPIO_Port , S_H_D_N_Pin , GPIO_PIN_SET   )
#define TURN_OFF_POWER		HAL_GPIO_WritePin( S_H_D_N_GPIO_Port , S_H_D_N_Pin , GPIO_PIN_RESET )

typedef enum
{
	WIFI_CONNECTION_FAIL,
	EMPTY_TRAY,
	TRAY_IN_MOTION,
	TOO_SHORT_IGNITION,
	LOW_BATTERY,
	CHARGER_REMOVED,
	SUCCESSFUL_SHUT_DOWN
} ANOMALY_REASON;

typedef enum
{
	ALL_COUNTERS,
	WAKE_UP_COUNTER,
	SNOOZE_COUNTER,
	ENVIRONMENT_SEND_COUNTER,
	ALERT_SEND_INTERVAL_COUNTER,
	ENVIRONMENT_LOG_COUNTER
} COUNTER;

WAKE_UP_REASON schedule_timing_conditions();
void schedule_goto_sleep();
void schedule_session_anomaly(ANOMALY_REASON anomaly_reason);
void schedule_reset_session_anomaly_counter();
void schedule_wait_for_reed_switch();
void schedule_initialize_counters();
void schedule_reset_counters(COUNTER counter);

#endif /* SCHEDULE_H_ */
