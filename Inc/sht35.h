/*
 * sht35.h
 *
 *  Created on: 9 במאי 2018
 *      Author: oferf
 */

#ifndef SHT35_H_
#define SHT35_H_

int16_t sht35_get_one_shot_temperature();
int16_t sht35_get_periodic_mode_temperature();
void sht35_start_periodic_mode_temperature_sampling(uint8_t *command);
void sht35_stop_periodic_mode_temperature_sampling();
void sht35_soft_reset();
void sht35_general_call_reset();
void sht35_heater_enable();
void sht35_heater_disable();
void sht35_clear_status();
uint16_t sht35_get_status();
void sht35_initialization();
float sht35_get_temperature_in_celsius(int16_t raw_temperature);
float sht35_get_temperature_in_fahrenheit(int16_t raw_temperature);
uint8_t sht35_get_humidity(int16_t raw_humidity);
void sht35_get_sample(void);
void sht35_get_temperature_sample(void);
void sht35_get_humidity_sample(void);
void sht35_get_temperature_and_humidity_bits( uint16_t *temperature , uint16_t *humidity );
void sht35_get_last_temperature_and_humidity_bits( uint16_t *temperature , uint16_t *humidity );
float sht35_get_temperature();

#endif /* SHT35_H_ */
