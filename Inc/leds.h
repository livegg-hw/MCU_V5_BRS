/*
 * leds.h
 *
 *  Created on: 16 αιεπι 2018
 *      Author: oferf
 */

#ifndef LEDS_H_
#define LEDS_H_

typedef enum
{
	CHARGE_LED,
	INDICATION_LED,

	NUMBER_OF_LEDS
} LED;

typedef enum
{
	TURN_OFF_LED,
	RED,
	ORANGE,
	GREEN
} COLOR;

typedef enum
{
	NO_BLINKING,
	BLINKING_RATE_1_HZ,
	BLINKING_RATE_2_HZ,
	BLINKING_RATE_3_HZ,
	BLINKING_RATE_4_HZ,
	BLINKING_RATE_5_HZ
} BLINKING_RATE;

typedef struct
{
	COLOR   color;
	uint8_t blinking_rate;
} LED_CONFIGURATION;

void led_charge_configuration(LED_CONFIGURATION led_configuration);
void led_indication_configuration(LED_CONFIGURATION led_configuration);
void leds_illuminate( LED led , COLOR color );
void leds_illuminate_blinking( LED led , COLOR color , uint16_t blinking_rate , uint8_t duty_cycle );
void leds_illuminate_blinking_single_channel( LED led , COLOR color , uint16_t blinking_rate , uint8_t duty_cycle );
void leds_set_configuration_update_flag(bool enable_status);
void leds_restoring_status();


#endif /* LEDS_H_ */
