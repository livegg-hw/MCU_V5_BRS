/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : client.c                                                                                  		*
 *  Abstract   : Wifi protocol of the client implemention.                                        				*
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : APRIL 2018                                                                                     *
 *--------------------------------------------------------------------------------------------------------------*/

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "eeprom_image.h"
#include "atwinc1500.h"
#include "photo_diode.h"
#include "sensors.h"
#include "adt7420.h"
#include "adxl345.h"
#include "sht35.h"
#include "lis3dshtr.h"
#include "macronix_flash.h"
#include "schedule.h"
#include "leds.h"
#include "battery.h"
#include "upload.h"
#include "command.h"
#include "tray_id.h"
#include "environment_log.h"

/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define HEADER_SIGNATURE   							"ABC0"
#define PARTIAL_HEADER_SIGNATURE   					"ABC"
#define FOOTER_SIGNATURE   							"0CBA"
#define PARTIAL_FOOTER_SIGNATURE   					"CBA"
#define PREAMBLE									"$*&?"
#define POSTAMBLE									"!*#@"
#define BUILD_DATE_TIME 							__DATE__ "@@" __TIME__
#define COMMAND_STRING_SIZE 						2
#define PAYLOAD_LENGTH_SIZE							sizeof(uint32_t)
#define PACKET_CHECKSUM_SIZE						sizeof(uint8_t)
#define BLOCK_NUMBER_SIZE							sizeof(uint16_t)

typedef union
{
	int 	firmware_version;
	uint8_t firmware_version_array[4];
} FIRMWARE_VERSION;

typedef union
{
	char     preample_array[4];
	uint32_t preamble_word;
} PREAMBLE_UNION;

typedef union
{
	char     postample_array[4];
	uint32_t postamble_word;
} POSTAMBLE_UNION;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} NO_PAYLOAD_FRAME;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint16_t		payload[2];
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} GET_DATA_PAYLOAD_FRAME;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint8_t			payload[sizeof(SAMPLING_PARAMETERS)];
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} SETUP_UNIT_FRAME;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint32_t		payload[2];
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} ENVIRONMENT_LOG_FRAME;

typedef struct  __attribute__((packed))
{
	uint16_t 		adt7420_temprature;
	uint16_t 		internal_sht35_temprature;
	uint16_t 		internal_sht35_humidity;
	uint32_t 		external_sht35_temprature;
	uint32_t 		external_sht35_humidity;
	uint32_t 		external_co2;
	int16_t  		adxl_accelerometer_axis_x;
	int16_t  		adxl_accelerometer_axis_y;
	int16_t  		adxl_accelerometer_axis_z;
	int16_t  		lis3dshtr_accelerometer_axis_x;
	int16_t  		lis3dshtr_accelerometer_axis_y;
	int16_t  		lis3dshtr_accelerometer_axis_z;
	uint16_t 		battery_bits;
	DETECTOR_TYPE 	working_point_leds_dac_value[16];
	FIRMWARE_VERSION  boot_firmware_version;
	FIRMWARE_VERSION  application_firmware_version;
	char     		build_date_and_time[21];
	char			tray_id[12];
	char			chip_id[12];
} READABLE_ONLY_PARAMETERS;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint8_t			payload[sizeof(READABLE_ONLY_PARAMETERS)];
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} READABLE_ONLY_PARAMETERS_REQUEST_RESPONSE_FRAME;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint8_t			payload[sizeof(READABLE_ONLY_PARAMETERS)]; // having maximum payload of "P1" and "P2" commands
	//uint8_t			payload[sizeof(SAMPLING_PARAMETERS)];
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} SAMPLING_PARAMETERS_REQUEST_RESPONSE_FRAME;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint8_t			payload[sizeof(READABLE_ONLY_PARAMETERS)];
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} RESPONSE_FRAME;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint8_t			payload[1024];
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} ENVIRONENT_LOG_RESPONSE_FRAME;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint8_t			payload[13];
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} FIRMWARE_FRAME;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint32_t		payload[8];
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} FIRMWARE_RESPONSE_FRAME;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint16_t		block_number;
	uint8_t			payload[UPLOADING_BLOCK_SIZE];
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} UPDATE_FRAME;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint16_t		payload;
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} UPDATE_RESPONSE_FRAME;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint8_t			payload[2];
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} COPY_FRAME;

typedef struct  __attribute__((packed))
{
	PREAMBLE_UNION 	preamble;
	char 		    command[COMMAND_STRING_SIZE];
	uint32_t  		length;
	uint8_t			payload[2];
	uint8_t  		checksum;
	POSTAMBLE_UNION	postamble;
} COPY_RESPONSE_FRAME;

typedef struct  __attribute__((packed))
{
	char  command[COMMAND_STRING_SIZE];
	char response[COMMAND_STRING_SIZE];
} COMMANDS;

typedef enum
{
	PARAMETERS_1,
	PARAMETERS_2,
	GET_DATA,
	SETUP_UNIT,
	ENVIRONMENT_LOG,
	GO_TO_SLEEP,
	FIRMWARE,
	UPDATE,
	COPY,

	NUMBER_OF_CLIENT_COMMANDS
} CLIENT_COMMAND;

COMMANDS client_command_string[NUMBER_OF_CLIENT_COMMANDS] =
{
/* PARAMETERS_1 	*/ { "P1" , "1P" } ,
/* PARAMETERS_2 	*/ { "P2" , "2P" } ,
/* GET_DATA     	*/ { "GD" , "DG" } ,
/* SETUP_UNIT   	*/ { "SU" , "US" } ,
/* ENVIRONMENT_LOG  */ { "LG" , "GL" } ,
/* GO_TO_SLEEP  	*/ { "!C" , "C!" } ,
/* FIRMWARE			*/ { "FW" , "WF" } ,
/* UPDATE			*/ { "UD" , "DU" } ,
/* COPY				*/ { "CP" , "PC" } ,
};

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
char firmware_version[] = "05.02.01.13";
extern RTC_HandleTypeDef hrtc;
extern volatile bool wifi_sending_completed;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
uint8_t calculate_checksum( uint8_t *data , uint16_t size)
{
	uint8_t checksum = 0;
	for( uint16_t data_index = 0 ; data_index < size ; data_index++ )
		checksum += data[data_index];
	return checksum;
}

void send_wifi_message( uint8_t *message , uint16_t message_length )
{
	atwinc1500_send_wifi_message( message, message_length );
}

void envlope_payload( RESPONSE_FRAME *response_frame , uint8_t *payload , uint16_t message_length , char *command )
{
	memcpy( &( response_frame)->preamble  , PREAMBLE  , sizeof( PREAMBLE_UNION) );
	memcpy( &( response_frame)->postamble , POSTAMBLE , sizeof(POSTAMBLE_UNION) );
	memcpy( &( response_frame)->command   , command   , COMMAND_STRING_SIZE );
	#warning "no defence on memcpy"
	memcpy( &( response_frame)->payload   , payload   , message_length );
	#warning "no defence on memcpy"
	( response_frame)->length   = message_length;
	( response_frame)->checksum = calculate_checksum( (uint8_t *)response_frame , sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE + PAYLOAD_LENGTH_SIZE + message_length );
}

void envlope_empty_payload( NO_PAYLOAD_FRAME *response_frame , char *command )
{
	memcpy( &( response_frame)->preamble  , PREAMBLE  , sizeof( PREAMBLE_UNION) );
	memcpy( &( response_frame)->postamble , POSTAMBLE , sizeof(POSTAMBLE_UNION) );
	memcpy( &( response_frame)->command   , command   , COMMAND_STRING_SIZE );
	( response_frame)->length   = 0;
	( response_frame)->checksum = calculate_checksum( (uint8_t *)response_frame , sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE + PAYLOAD_LENGTH_SIZE );
}

int get_firmware_version()
{
	static FIRMWARE_VERSION application_firmware_version;
	static int				 firmaware_version_parts[4];
	sscanf( firmware_version , "%d.%d.%d.%d" , &firmaware_version_parts[3] , &firmaware_version_parts[2] , &firmaware_version_parts[1] , &firmaware_version_parts[0] );
	for( uint8_t firmware_version_part_index = 0 ; firmware_version_part_index < 4 ; firmware_version_part_index++ ) 
	{
		application_firmware_version.firmware_version_array[firmware_version_part_index] = firmaware_version_parts[firmware_version_part_index];
	}
	return application_firmware_version.firmware_version;
}

void fill_readable_only_parameters(READABLE_ONLY_PARAMETERS *payload)
{
	#define UNIQUE_DEVICE_ID		0x1FFF7590

    payload->external_co2 = getCo2LevelFromExtBoard();

	payload->adt7420_temprature = adt7420_get_last_temperature_bits();
	sht35_get_last_temperature_and_humidity_bits( &payload->internal_sht35_temprature , &payload->internal_sht35_humidity );
	adxl345_get_accelerometer_sample(   (SAMPLE *)&payload->adxl_accelerometer_axis_x );
	lis3dshtr_get_accelerometer_sample( (SAMPLE *)&payload->lis3dshtr_accelerometer_axis_x );
	payload->battery_bits = battery_get_sampling_adc_bits();
	photo_diode_copy_working_points_dac_value(&payload->working_point_leds_dac_value[0]);
    hrtc.Instance = RTC;
	payload->boot_firmware_version.firmware_version = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	payload->application_firmware_version.firmware_version = get_firmware_version();
	#warning "Add test lenght here"
	memcpy( &payload->build_date_and_time , BUILD_DATE_TIME , sizeof(BUILD_DATE_TIME) );
	tray_id_copy_tray_id(&payload->tray_id[0]);
	memcpy( &payload->chip_id , (char *)UNIQUE_DEVICE_ID , sizeof(payload->chip_id) );
}

void send_readable_only_parameters(char *command)
{
	static READABLE_ONLY_PARAMETERS_REQUEST_RESPONSE_FRAME response_frame;

	fill_readable_only_parameters( (READABLE_ONLY_PARAMETERS *)&response_frame.payload );
	envlope_payload( (RESPONSE_FRAME *)&response_frame , &response_frame.payload[0] , sizeof(response_frame.payload) , command );
	send_wifi_message( (uint8_t *)&response_frame , sizeof(response_frame) );
}

void send_sampling_parameters(char *command)
{
	static SAMPLING_PARAMETERS_REQUEST_RESPONSE_FRAME response_frame;

	fill_samples_parameters( (uint8_t *)&response_frame.payload );
	envlope_payload( (RESPONSE_FRAME *)&response_frame , &response_frame.payload[0] , sizeof(response_frame.payload) , command );
	send_wifi_message( (uint8_t *)&response_frame , sizeof(response_frame) );
}

//extern uint16_t aaa[40*32*32];
void __attribute__((optimize("O0"))) send_sampling_data( char *command , uint16_t first_samples_packet_number  , uint16_t last_samples_packet_number )
{
  #define MESSAGE_BUFFER_SIZE   (1024*18) 
  static uint8_t message_buffer[MESSAGE_BUFFER_SIZE];
  static uint8_t printBuffer[200];
	uint16_t payload_size = photo_diode_get_sampling_data_payload_size();
	
	#warning "print payload size"
	uint16_t message_size = sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE + PAYLOAD_LENGTH_SIZE + payload_size + BLOCK_NUMBER_SIZE + PACKET_CHECKSUM_SIZE + sizeof(POSTAMBLE_UNION);
	
	if (message_size > MESSAGE_BUFFER_SIZE)
	{   
	        LogPrintfWrapperC(FONT_LIGHT_RED, "send_sampling_data: payload size %u > max %u\r\n",message_size, MESSAGE_BUFFER_SIZE);
	        message_size = MESSAGE_BUFFER_SIZE-1;
	        payload_size = message_size - (sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE + PAYLOAD_LENGTH_SIZE + BLOCK_NUMBER_SIZE + PACKET_CHECKSUM_SIZE + sizeof(POSTAMBLE_UNION) );
	}

	uint8_t *sample_data_response_frame = message_buffer; // malloc(message_size);
	memcpy( &sample_data_response_frame[0]  					 , PREAMBLE , sizeof(PREAMBLE_UNION) );
	memcpy( &sample_data_response_frame[sizeof(PREAMBLE_UNION)]  , command  , COMMAND_STRING_SIZE    );
	*( (uint32_t *)&sample_data_response_frame[sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE] ) = payload_size + BLOCK_NUMBER_SIZE;
	for( uint16_t samples_packet_index = first_samples_packet_number ; samples_packet_index <= last_samples_packet_number ; samples_packet_index++ )
	{
		//memcpy( (uint8_t *)&sample_data_response_frame[sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE + 4] , &aaa[samples_packet_number * 32] , payload_size );
		macronix_flash_read_data_from_flash( samples_packet_index * payload_size , (uint8_t *)&sample_data_response_frame[sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE + PAYLOAD_LENGTH_SIZE + BLOCK_NUMBER_SIZE] , payload_size );
		*( (uint16_t *)&sample_data_response_frame[sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE + PAYLOAD_LENGTH_SIZE] ) = samples_packet_index ;
		sample_data_response_frame[sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE + PAYLOAD_LENGTH_SIZE + BLOCK_NUMBER_SIZE + payload_size] = calculate_checksum( sample_data_response_frame , sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE + PAYLOAD_LENGTH_SIZE + BLOCK_NUMBER_SIZE + payload_size );
		memcpy( &sample_data_response_frame[sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE + PAYLOAD_LENGTH_SIZE + BLOCK_NUMBER_SIZE + payload_size + PACKET_CHECKSUM_SIZE] , POSTAMBLE , sizeof(POSTAMBLE_UNION) );
		send_wifi_message( sample_data_response_frame , message_size );
	}
	//free(sample_data_response_frame);
}

void send_setup_unit_data( char *command , uint8_t *sampling_parameters_pointer )
{
  static NO_PAYLOAD_FRAME response_frame;
	memcpy( &response_frame.preamble  , PREAMBLE  , sizeof( PREAMBLE_UNION) );
	memcpy( &response_frame.postamble , POSTAMBLE , sizeof(POSTAMBLE_UNION) );
	memcpy( &response_frame.command   , command   , COMMAND_STRING_SIZE );
	response_frame.length   = 0;
	response_frame.checksum = calculate_checksum( (uint8_t *)&response_frame , sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE + 4 );
	photo_diode_set_sampling_parameters(sampling_parameters_pointer);
	send_wifi_message( (uint8_t *)&response_frame , sizeof(response_frame) );
}

void send_environment_log_response( char *command )
{
// #warning "move to static - big size can overload stack"
static ENVIRONENT_LOG_RESPONSE_FRAME environment_log_response_frame;

	memcpy( &environment_log_response_frame.preamble  , PREAMBLE  , sizeof( PREAMBLE_UNION) );
	memcpy( &environment_log_response_frame.postamble , POSTAMBLE , sizeof(POSTAMBLE_UNION) );
	memcpy( &environment_log_response_frame.command   , command   , COMMAND_STRING_SIZE );

	environment_log_response_frame.length = sizeof(environment_log_response_frame.payload);

	memcpy( &environment_log_response_frame.payload   , environment_log_get_environment_log_pointer() , sizeof(environment_log_response_frame.payload) );

	environment_log_response_frame.checksum = calculate_checksum( (uint8_t *)&environment_log_response_frame , sizeof(ENVIRONENT_LOG_RESPONSE_FRAME) - sizeof(environment_log_response_frame.checksum) - sizeof(POSTAMBLE_UNION) );

	send_wifi_message( (uint8_t *)&environment_log_response_frame , sizeof(environment_log_response_frame) );
}

void __attribute__((optimize("O0"))) send_go_to_sleep_response(char *command)
{
	static NO_PAYLOAD_FRAME response_frame;
	envlope_empty_payload( (NO_PAYLOAD_FRAME *)&response_frame , command );
	wifi_sending_completed = false;
	send_wifi_message( (uint8_t *)&response_frame , sizeof(response_frame) );
	HAL_Delay(1);
	schedule_reset_session_anomaly_counter();
	schedule_goto_sleep();
}

void  send_firmware_response(char *command)
{
   
	static FIRMWARE_RESPONSE_FRAME firmware_response_frame;
	memcpy( &firmware_response_frame.preamble  , PREAMBLE  , sizeof( PREAMBLE_UNION) );
	memcpy( &firmware_response_frame.postamble , POSTAMBLE , sizeof(POSTAMBLE_UNION) );
	memcpy( &firmware_response_frame.command   , command   , COMMAND_STRING_SIZE );
	firmware_response_frame.length   = sizeof(firmware_response_frame.payload);
	for( IMAGE image_index = IMAGE_APPLICATION ; image_index < NUMBER_OF_IMAGES ; image_index++ )
	{
		firmware_response_frame.payload[ 2 * image_index     ] = eeprom_image_get_image_version(image_index);
		firmware_response_frame.payload[ 2 * image_index + 1 ] = eeprom_image_get_crc(image_index);
	}
	firmware_response_frame.checksum = calculate_checksum( (uint8_t *)&firmware_response_frame , sizeof(PREAMBLE_UNION) + COMMAND_STRING_SIZE + sizeof(firmware_response_frame.length) + sizeof(firmware_response_frame.payload) );
	send_wifi_message( (uint8_t *)&firmware_response_frame , sizeof(firmware_response_frame) );
}

void  send_update_response( char *command , uint16_t block_number )
{
	static UPDATE_RESPONSE_FRAME update_response_frame;
	memcpy( &update_response_frame.preamble  , PREAMBLE  , sizeof( PREAMBLE_UNION) );
	memcpy( &update_response_frame.postamble , POSTAMBLE , sizeof(POSTAMBLE_UNION) );
	memcpy( &update_response_frame.command   , command   , COMMAND_STRING_SIZE );
	update_response_frame.length   = sizeof(update_response_frame.payload);
	update_response_frame.payload  = block_number;
	update_response_frame.checksum = calculate_checksum( (uint8_t *)&update_response_frame , sizeof(UPDATE_RESPONSE_FRAME) - sizeof(update_response_frame.checksum) - sizeof(POSTAMBLE_UNION) );
	send_wifi_message( (uint8_t *)&update_response_frame , sizeof(update_response_frame) );
}

void  send_copy_response( char *command , uint8_t source_image , uint8_t destination_image )
{
	static COPY_RESPONSE_FRAME copy_response_frame;
	memcpy( &copy_response_frame.preamble  , PREAMBLE  , sizeof( PREAMBLE_UNION) );
	memcpy( &copy_response_frame.postamble , POSTAMBLE , sizeof(POSTAMBLE_UNION) );
	memcpy( &copy_response_frame.command   , command   , COMMAND_STRING_SIZE );
	copy_response_frame.length   = 2;
	copy_response_frame.payload[0] = source_image;
	copy_response_frame.payload[1] = destination_image;
	copy_response_frame.checksum = calculate_checksum( (uint8_t *)&copy_response_frame , sizeof(COPY_RESPONSE_FRAME) - sizeof(copy_response_frame.checksum) - sizeof(POSTAMBLE_UNION) );
	send_wifi_message( (uint8_t *)&copy_response_frame , sizeof(copy_response_frame) );
}
char updating_array[200];
void client_receive_protocol( uint8_t *data , uint16_t size )
{
	CLIENT_COMMAND		 	client_command;
	NO_PAYLOAD_FRAME     	*no_payload_frame     	= (NO_PAYLOAD_FRAME       *)data;
	GET_DATA_PAYLOAD_FRAME 	*get_data_payload_frame = (GET_DATA_PAYLOAD_FRAME *)data;
	SETUP_UNIT_FRAME	 	*setup_unit_frame     	= (SETUP_UNIT_FRAME       *)data;
	ENVIRONMENT_LOG_FRAME	*environment_log_frame  = (ENVIRONMENT_LOG_FRAME  *)data;
	FIRMWARE_FRAME		 	*pirmware_frame       	= (FIRMWARE_FRAME         *)data;
	UPDATE_FRAME		 	*update_frame		   	= (UPDATE_FRAME		      *)data;
	COPY_FRAME				*copy_frame				= (COPY_FRAME		      *)data;

	for( client_command = 0 ; client_command < NUMBER_OF_CLIENT_COMMANDS ; client_command++ )
	{
		if( strncmp( no_payload_frame->command , client_command_string[client_command].command , 2 ) == 0 )
			break;
	}
	#warning "Add ()"
	switch(client_command)
	{
		case PARAMETERS_1:		if( (sizeof(NO_PAYLOAD_FRAME) == size) && (strncmp( no_payload_frame->preamble.preample_array , PREAMBLE , 4 ) == 0) &&	(strncmp( no_payload_frame->postamble.postample_array , POSTAMBLE , 4 ) == 0) )
								{
									send_readable_only_parameters(client_command_string[client_command].response);
									atwinc1500_reset_connection_attempt_failed_timer();
								}
								break;

		case PARAMETERS_2:		if( sizeof(NO_PAYLOAD_FRAME) == size && strncmp( no_payload_frame->preamble.preample_array , PREAMBLE , 4 ) == 0 &&	strncmp( no_payload_frame->postamble.postample_array , POSTAMBLE , 4 ) == 0 )
								{
									send_sampling_parameters(client_command_string[client_command].response);
									atwinc1500_reset_connection_attempt_failed_timer();
								}
								break;

		case GET_DATA:			if( sizeof(GET_DATA_PAYLOAD_FRAME) == size && strncmp( get_data_payload_frame->preamble.preample_array , PREAMBLE , 4 ) == 0 &&	strncmp( get_data_payload_frame->postamble.postample_array , POSTAMBLE , 4 ) == 0 )
								{
									send_sampling_data( client_command_string[client_command].response , get_data_payload_frame->payload[0] , get_data_payload_frame->payload[1] );
									atwinc1500_reset_connection_attempt_failed_timer();
								}
								break;

		case SETUP_UNIT:		if( sizeof(SETUP_UNIT_FRAME) == size && strncmp( setup_unit_frame->preamble.preample_array , PREAMBLE , 4 ) == 0 &&	strncmp( setup_unit_frame->postamble.postample_array , POSTAMBLE , 4 ) == 0 )
								{
									send_setup_unit_data( client_command_string[client_command].response , setup_unit_frame->payload );
									atwinc1500_reset_connection_attempt_failed_timer();
								}
								break;

		case ENVIRONMENT_LOG:	if( sizeof(ENVIRONMENT_LOG_FRAME) == size  && strncmp( environment_log_frame->preamble.preample_array , PREAMBLE , 4 ) == 0 &&	strncmp( environment_log_frame->postamble.postample_array , POSTAMBLE , 4 ) == 0 && calculate_checksum( (uint8_t *)environment_log_frame , sizeof(ENVIRONMENT_LOG_FRAME) - sizeof(environment_log_frame->checksum) - sizeof(POSTAMBLE_UNION) ) == environment_log_frame->checksum )
								{
									environment_log_update_time_and_date( environment_log_frame->payload[0] , environment_log_frame->payload[1] );
									send_environment_log_response( client_command_string[client_command].response );
									atwinc1500_reset_connection_attempt_failed_timer();
								}
								break;

		case GO_TO_SLEEP:		if( sizeof(NO_PAYLOAD_FRAME) == size && strncmp( no_payload_frame->preamble.preample_array , PREAMBLE , 4 ) == 0 &&	strncmp( no_payload_frame->postamble.postample_array , POSTAMBLE , 4 ) == 0 )
								{
									send_go_to_sleep_response(client_command_string[client_command].response);
									atwinc1500_reset_connection_attempt_failed_timer();
								}
								break;

		case FIRMWARE:			if( sizeof(FIRMWARE_FRAME) == size && strncmp( pirmware_frame->preamble.preample_array , PREAMBLE , 4 ) == 0 &&	strncmp( pirmware_frame->postamble.postample_array , POSTAMBLE , 4 ) == 0 )
								{
									upload_set_upload_parameters(pirmware_frame->payload);
									send_firmware_response(client_command_string[client_command].response);
									atwinc1500_reset_connection_attempt_failed_timer();
								}
								break;

		case UPDATE:			sprintf( updating_array , "RECEIEVE BLOCK NUMBER: %d  PREAMBLE=%s POSTAMBLE=%s sizeof(UPDATE_FRAME)=%d size=%d\r\n" , (int)update_frame->block_number , PREAMBLE , POSTAMBLE , sizeof(UPDATE_FRAME) , size );
								send_message_to_pc( updating_array , strlen((char *)updating_array) );
								if( sizeof(UPDATE_FRAME) == size && strncmp( update_frame->preamble.preample_array , PREAMBLE , 4 ) == 0 &&	strncmp( update_frame->postamble.postample_array , POSTAMBLE , 4 ) == 0 )
								{
									sprintf( updating_array , "RECEIEVE BLOCK NUMBER: %d -  HEADER OK\r\n" , (int)update_frame->block_number );
									send_message_to_pc( updating_array , strlen((char *)updating_array) );
									if( calculate_checksum( (uint8_t *)update_frame , sizeof(UPDATE_FRAME) - sizeof(update_frame->checksum) - sizeof(POSTAMBLE_UNION) ) == update_frame->checksum )
									{
										upload_program_block( update_frame->block_number , update_frame->payload , update_frame->length );
										send_update_response( client_command_string[client_command].response , update_frame->block_number );
										atwinc1500_reset_connection_attempt_failed_timer();
										sprintf( updating_array , "SEND BLOCK NUMBER %d RESPONSE\r\n" , (int)update_frame->block_number );
										send_message_to_pc( updating_array , strlen((char *)updating_array) );
									}
								}
								break;

		case COPY:				if( sizeof(COPY_FRAME) == size && strncmp( copy_frame->preamble.preample_array , PREAMBLE , 4 ) == 0 &&	strncmp( copy_frame->postamble.postample_array , POSTAMBLE , 4 ) == 0 )
								{
									if( calculate_checksum( (uint8_t *)copy_frame , sizeof(COPY_FRAME) - sizeof(copy_frame->checksum) - sizeof(POSTAMBLE_UNION) ) == copy_frame->checksum )
									{
										send_copy_response( client_command_string[client_command].response , copy_frame->payload[0] , copy_frame->payload[1] );
										atwinc1500_reset_connection_attempt_failed_timer();
										upload_copy_blocks( copy_frame->payload[0] , copy_frame->payload[1] );
									}
								}
								break;

		case NUMBER_OF_CLIENT_COMMANDS:
								// meaning undefined command.
								break;
	}
}


uint16_t port_number_getting();

void printStartUpData(void)
{
    //char firmware_version[] = "05.03.01.01";
    LogPrintfWrapperC(FONT_GREEN, "Version:%s, Port Number: %d\r\n", firmware_version,port_number_getting());
    
}
