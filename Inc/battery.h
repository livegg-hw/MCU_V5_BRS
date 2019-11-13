/*
 * battery.h
 *
 *  Created on: 17 αιεπι 2018
 *      Author: oferf
 */

#ifndef BATTERY_H_
#define BATTERY_H_

float battery_get_sampling_voltage();
void battery_update_charge_led_status();
uint16_t battery_get_sampling_adc_bits();
void battery_get_battery_measurement();
COLOR battery_get_charging_led_color();
void battery_check_low_battery();

#endif /* BATTERY_H_ */
