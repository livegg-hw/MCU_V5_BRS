/*
 * photo_diode.h
 *
 *  Created on: 15 במאי 2018
 *      Author: oferf
 */

#ifndef PHOTO_DIODE_H_
#define PHOTO_DIODE_H_

typedef enum
{
	PHOTO_DIODE_0,
	PHOTO_DIODE_1,
	PHOTO_DIODE_2,
	PHOTO_DIODE_3,
	PHOTO_DIODE_4,
	PHOTO_DIODE_5,
	PHOTO_DIODE_6,
	PHOTO_DIODE_7,
	PHOTO_DIODE_8,
	PHOTO_DIODE_9,
	PHOTO_DIODE_10,
	PHOTO_DIODE_11,
	PHOTO_DIODE_12,
	PHOTO_DIODE_13,
	PHOTO_DIODE_14,
	PHOTO_DIODE_15,

	NUMBER_OF_PHOTO_DIODES
} PHOTO_DIODE;

typedef struct
{
	uint16_t transferring_detector;
	uint16_t reflection_detector;
} DETECTOR_TYPE;

typedef enum
{
	NORMAL_WAKE_UP				= 0x00,
	FROM_SLEEP  				= 0x01,
	FROM_SNOOZE 				= 0x02,
	FROM_ENVIRONMENT_SEND 		= 0x03,
	FROM_ALERT_SEND_INTERVAL 	= 0x04,
	FROM_ENVIRONMENT_LOG		= 0x05,
	FROM_BLINK  				= 0x06
} WAKE_UP_REASON;

typedef struct  __attribute__((packed))
{
	uint8_t  number_of_photo_diodes;
	uint8_t  photo_diodes_order[2 * NUMBER_OF_PHOTO_DIODES];
	uint16_t working_point;
	uint8_t  delay_before_dark_sample_us;
	uint8_t  delay_before_light_sample_us;
	uint8_t  oversampling_factor;
	uint8_t  measurement_size_bits;
	uint16_t sample_interval_us;
	uint16_t samples_per_package;
	uint16_t number_of_packages;
	uint8_t  mode;
	uint8_t  maximum_temperature_in_celsios;
	uint8_t  minimum_temperature_in_celsios;
	WAKE_UP_REASON  wake_up_reason;
	uint16_t sleep_interval_1_full_sample;
	uint16_t sleep_interval_2_snooze;
	uint16_t sleep_interval_3_environment_send;
	uint16_t sleep_interval_4_alert;
	uint16_t sleep_interval_5_environment_log;
	uint16_t sleep_interval_6_blink;
	uint16_t maximum_dac_value;
} SAMPLING_PARAMETERS;

void photo_diode_initialization();
void photo_diode_start_all_leds_sampling();
void send_sampling_parameters();
void photo_diode_set_sampling_parameters(uint8_t *new_sampling_parameters);
void fill_samples_parameters(uint8_t *sampling_parameters_pointer);
void delay_us(uint16_t delay_time);
uint16_t photo_diode_get_sampling_data_payload_size();
void photo_diode_start_sampling(uint8_t  mode);
void photo_diode_get_sampling_parameters();
void photo_diode_get_samples(uint32_t packet_number);
void photo_diode_copy_working_points_dac_value(DETECTOR_TYPE *working_point_dac_value_pointer);
void photo_diode_set_defaults();
unsigned char photo_diode_Sanity_minimu_maximum_validation();
void photo_diode_set_wake_up_reason(WAKE_UP_REASON wake_up_reason);
bool photo_diode_check_temperature_boundaries(float temperature);
bool photo_diode_is_tray_loaded();
void photo_diode_check_tray_loading_status();

#endif /* PHOTO_DIODE_H_ */
