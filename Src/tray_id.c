/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : port_number.c                                                                               	*
 *  Abstract   : This module handles the port number of the wifi.                                               *
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : SEPTEMBER 2018                                                                                 *
 *--------------------------------------------------------------------------------------------------------------*/

#include <string.h>
#include "stm32l4xx_hal.h"
#include "atwinc1500.h"
#include "command.h"
#include "tray_id.h"

/*-------------------------------------------------------------------------------------------*/
/*----------------------------- DEFINITIONS AND ENUMARTIONS ---------------------------------*/
/*-------------------------------------------------------------------------------------------*/
//#define IS_FLASH_OTP_ADDRESS(ADDRESS)   (((ADDRESS) >= 0x1FFF7000) && ((ADDRESS) <= 0x1FFF73FF))
#define FLASH_OTP_ADDRESS_START			0x1FFF7000
#define FLASH_OTP_ADDRESS_STOP			0x1FFF7400
#define TRAY_ID_SIGNATURE				0xAAAA
#define TRAY_ID_RECORDS					( ( FLASH_OTP_ADDRESS_STOP - FLASH_OTP_ADDRESS_START ) / sizeof(TRAY_ID) )

typedef struct
{
	uint16_t port_number;
	uint16_t tray_id_signature;
	char     tray_id_upper[4];
} TRAY_ID_UPPER_STRUCT;

typedef union
{
	TRAY_ID_UPPER_STRUCT	tray_id_upper_struct;
	uint64_t			 	tray_id_upper_word;
} TRAY_ID_SEGMENT_1;

typedef union
{
	uint64_t	tray_id_lower_word;
	char     	tray_id_lower_array[8];
} TRAY_ID_SEGMENT_2;

typedef union
{
	//uint64_t	tray_id_segment_3_word;
	char     	tray_id_segment_3[16];
} TRAY_ID_SEGMENT_3;

typedef struct
{
	TRAY_ID_SEGMENT_1 tray_id_segment_1;
	TRAY_ID_SEGMENT_2 tray_id_segment_2;
	TRAY_ID_SEGMENT_3 tray_id_segment_3;
} TRAY_ID;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void tray_id_otp_writting( uint32_t address , uint64_t data )
{
    HAL_FLASH_Unlock(); /* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address , data );
	HAL_FLASH_Lock();
}

uint16_t port_number_getting()
{
//#warning "Boris"
//    return (9364);
    
	TRAY_ID *flash_otp_address = (TRAY_ID *)FLASH_OTP_ADDRESS_START;

	while( IS_FLASH_OTP_ADDRESS( (uint32_t)flash_otp_address) && ( flash_otp_address + 1 )->tray_id_segment_1.tray_id_upper_struct.tray_id_signature == TRAY_ID_SIGNATURE )
	{
		if( IS_FLASH_OTP_ADDRESS( (uint32_t)( flash_otp_address + 1) ) )
			flash_otp_address++;
		else
			break;
	}
	return flash_otp_address->tray_id_segment_1.tray_id_upper_struct.port_number;
}

uint32_t tray_id_and_port_number_getting( char * tray_id , uint16_t * port_number )
{
	TRAY_ID *flash_otp_address = (TRAY_ID *)FLASH_OTP_ADDRESS_START;

	while( IS_FLASH_OTP_ADDRESS( (uint32_t)flash_otp_address) && ( flash_otp_address + 1 )->tray_id_segment_1.tray_id_upper_struct.tray_id_signature == TRAY_ID_SIGNATURE )
	{
		if( IS_FLASH_OTP_ADDRESS( (uint32_t)( flash_otp_address + 1) ) )
			flash_otp_address++;
		else
			break;
	}
	*port_number = flash_otp_address->tray_id_segment_1.tray_id_upper_struct.port_number;
//	#warning "Boris"
//	*port_number = 9364;
	memcpy( tray_id     , flash_otp_address->tray_id_segment_1.tray_id_upper_struct.tray_id_upper , 4 );
	memcpy( tray_id + 4 , flash_otp_address->tray_id_segment_2.tray_id_lower_array 				  , 8 );
	return (uint32_t)flash_otp_address;
}

uint32_t tray_id_and_port_number_setting( char * new_tray_id , uint16_t port_number )
{
	uint32_t	flash_otp_address = FLASH_OTP_ADDRESS_START;
	static TRAY_ID 	tray_id;

	tray_id.tray_id_segment_1.tray_id_upper_struct.port_number = port_number;
	tray_id.tray_id_segment_1.tray_id_upper_struct.tray_id_signature = TRAY_ID_SIGNATURE;
	memcpy( tray_id.tray_id_segment_1.tray_id_upper_struct.tray_id_upper , &new_tray_id[0] , 4 );
	memcpy( tray_id.tray_id_segment_2.tray_id_lower_array 				 , &new_tray_id[4] , 8 );
	while( IS_FLASH_OTP_ADDRESS(flash_otp_address) )
	{
		if( ( *(TRAY_ID *)flash_otp_address ).tray_id_segment_1.tray_id_upper_struct.tray_id_signature != TRAY_ID_SIGNATURE )
		{
			tray_id_otp_writting( flash_otp_address     , tray_id.tray_id_segment_1.tray_id_upper_word );
			tray_id_otp_writting( flash_otp_address + 8 , tray_id.tray_id_segment_2.tray_id_lower_word );
			return flash_otp_address;
		}
		else
		{
			if( ( (TRAY_ID *)( flash_otp_address + sizeof(TRAY_ID) ) )->tray_id_segment_1.tray_id_upper_struct.tray_id_signature != TRAY_ID_SIGNATURE )
			{
				if( ( *( (uint64_t *)( flash_otp_address      ) ) == tray_id.tray_id_segment_1.tray_id_upper_word )
				&&  ( *( (uint64_t *)( flash_otp_address + 8  ) ) == tray_id.tray_id_segment_2.tray_id_lower_word )
				&&  strncmp( (char *)( flash_otp_address + 16 ) , (char *)&tray_id.tray_id_segment_3 , 16 ) == 0 )
				//	&&  ( *( (uint64_t *)( flash_otp_address + 16 ) ) == tray_id.tray_id_segment_2.tray_id_lower_word ) )
				{
					return flash_otp_address;
				}
			}
		}
		flash_otp_address += sizeof(TRAY_ID);
	}
	return flash_otp_address;
}

uint8_t tray_id_get_record_number(uint32_t record_address)
{
	return( ( ( record_address - FLASH_OTP_ADDRESS_START ) / sizeof(TRAY_ID) ) + 1 );
}

/**
 * @brief       keeps raw level of co2 received from external board connected to uart connector
 */
uint32_t gCo2LevelRawData = 0;
/**
 * @brief       keeps temperature received from external board connected to uart connector
 */
uint32_t gTemperatureLevelRawData = 0;
/**
 * @brief       keeps humidity received from external board connected to uart connector
 */
uint32_t gHumidityLevelRawData = 0;


/**
 * @brief       handle data from received 
 *
 * @param       uint8_t [in]                           pointer to received string of data    
 *
 *
 * @retval      void
 */
void extCo2BoardMsgHandle(uint8_t *data)
{
//	/*** casting to floats ***/
//	/*** CO2 ***/
//	tempU32 = (uint32_t)((((uint32_t)buffer[0]) << 24) | (((uint32_t)buffer[1]) << 16) | (((uint32_t)buffer[3]) << 8) | ((uint32_t)buffer[4]));
//	*co2Concentration = *(float*)&tempU32;
//	sprintf(scd30_samples_string+6  , "%c%c%c%c ", buffer[0],buffer[1],buffer[3],buffer[4]);
//
//	/*** Temperature ***/
//	tempU32 = (uint32_t)((((uint32_t)buffer[6]) << 24) | (((uint32_t)buffer[7]) << 16) | (((uint32_t)buffer[9]) << 8) | ((uint32_t)buffer[10]));
//	*temperature      = *(float*)&tempU32;
//	sprintf(scd30_samples_string+11  , "%c%c%c%c ", buffer[6],buffer[7],buffer[9],buffer[10]);
//
//	/*** Humidity ***/
//	tempU32 = (uint32_t)((((uint32_t)buffer[12])<< 24) | (((uint32_t)buffer[13])<< 16) | (((uint32_t)buffer[15])<< 8) | ((uint32_t)buffer[16]));
//	*humidity         = *(float*)&tempU32;
//	sprintf(scd30_samples_string+16  , "%c%c%c%c%c", buffer[12],buffer[13],buffer[15],buffer[16],0x0A);//0x0A for \n
//
//	send_message_to_pc( (char *)&scd30_samples_string , strlen((const char *)scd30_samples_string) );

  // collect 4 bytes of co2 data
  gCo2LevelRawData = (uint32_t)((((uint32_t)data[0]) << 24) | (((uint32_t)data[1]) << 16) | (((uint32_t)data[3]) << 8) | ((uint32_t)data[4]));
//  // collect 4 bytes of temperature
//  gTemperatureLevelRawData = (uint32_t)((((uint32_t)data[6]) << 24) | (((uint32_t)data[7]) << 16) | (((uint32_t)data[9]) << 8) | ((uint32_t)data[10]));
//  // collect 4 bytes of humidity
//  gHumidityLevelRawData = (uint32_t)((((uint32_t)data[12])<< 24) | (((uint32_t)data[13])<< 16) | (((uint32_t)data[15])<< 8) | ((uint32_t)data[16]));
  LogPrintfWrapperC(FONT_LIGHT_BLUE, "Ext board data: co2 raw level: %3.3f\r\n", *(float*)&gCo2LevelRawData);
}



/**
 * @brief       returnes received from ext. boardco2 level 
 *
 * @param       none
 *
 * @retval      uint32_t co2 level
 */
uint32_t getCo2LevelFromExtBoard()
{
    return (gCo2LevelRawData);
}


void set_tray_id(uint8_t *data)
{
	static char tray_id_string[30];
	int port_number;
	uint32_t tray_id_record_address;

	sscanf( (char *)data , "%s%d" , (char *)&tray_id_string , &port_number );
	set_wifi_port_number(port_number);
	tray_id_record_address = tray_id_and_port_number_setting( tray_id_string , port_number );
	sprintf( tray_id_string , "%d %d\n" , tray_id_get_record_number(tray_id_record_address) , TRAY_ID_RECORDS );
	send_message_to_pc( tray_id_string , strlen(tray_id_string) );
}

void get_tray_id()
{
	 static char  		tray_id[13];
	 uint16_t 	port_number;
	 static char       tray_response[80];
	 uint32_t 	tray_id_record_address;

	 tray_id[12] = '\0';
	 tray_id_record_address = tray_id_and_port_number_getting( &tray_id[0] , &port_number );
	 sprintf( tray_response , "%s%d %d %d %d\n" , tray_id , port_number , tray_id_get_record_number(tray_id_record_address) , TRAY_ID_RECORDS , (int)atwinc1500_get_ip_number() );
	 send_message_to_pc( tray_response , strlen(tray_response) );
}

TRAY_TYPE tray_id_get_tray_type()
{
	 static char  		tray_id[13];
	 uint16_t 	port_number;
	 tray_id_and_port_number_getting( &tray_id[0] , &port_number );
	 return( ( tray_id[6] == 'M' ) ? GENDER_TRAY : REGULAR_TRAY );
	 //return( GENDER_TRAY );
}

void tray_id_copy_tray_id(char *tray_id_pointer)
{
	 static char  		tray_id[13];
	 uint16_t 	port_number;
	 tray_id_and_port_number_getting( &tray_id[0] , &port_number );
	 
	 memcpy( tray_id_pointer , tray_id , 12 );
}
