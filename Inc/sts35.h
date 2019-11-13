/*
 * sts35.h
 *
 *  Created on: 6 במאי 2018
 *      Author: oferf
 */

#ifndef STS35_H_
#define STS35_H_

uint16_t sts35_get_one_shot_temperature();
uint16_t sts35_get_periodic_mode_temperature();
void sts35_start_periodic_mode_temperature_sampling(uint8_t *command);
void sts35_stop_periodic_mode_temperature_sampling();
void sts35_soft_reset();
void sts35_general_call_reset();
void sts35_heater_enable();
void sts35_heater_disable();
void sts35_clear_status();
uint16_t sts35_get_status();
void sts35_initialization();
float sts35_get_temperature_in_celsius(int16_t raw_temperature);
void sts35_get_sample(void);

#endif /* STS35_H_ */
