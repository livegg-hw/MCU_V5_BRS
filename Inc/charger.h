/*
 * charger.h
 *
 *  Created on: 18 αιεπι 2018
 *      Author: oferf
 */

#ifndef CHARGER_H_
#define CHARGER_H_

typedef enum
{
	CHARGE_RATE_100MA   = '0',
	CHARGE_RATE_500MA   = '1',
	CHARGE_RATE_1500MA  = '2',
	CHARGE_RATE_STANDBY = '3'
} CHARGE_RATE;

void charger_set_setting(CHARGE_RATE charger_seting);
void charger_get_setting();
void charger_set_charge_control(char gpio_state);
void charger_get_iset_analog_measurement();
void charger_handle_charging();
bool charger_is_during_charging();

#endif /* CHARGER_H_ */
