/*
 * adt7420.h
 *
 *  Created on: 6 במאי 2018
 *      Author: oferf
 */

#ifndef ADT7420_H_
#define ADT7420_H_

void adt7420_initialization();
void get_adt7420_sample(void);
int16_t adt7420_get_one_shot_temperature();
void adt7420_print_sample();
int16_t adt7420_get_last_temperature_bits();
float adt7420_get_temperature();

#endif /* ADT7420_H_ */
