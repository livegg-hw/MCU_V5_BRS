/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : download.c                                                                               		*
 *  Abstract   : This module handles software upgrade.                                               			*
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : APRIL 2018                                                                                     *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "stm32l4xx_hal.h"
#include "internal_flash.h"
#include "eeprom_image.h"
#include "upload.h"
#include "command.h"

/*-------------------------------------------------------------------------------------------*/
/*----------------------------- DEFINITIONS AND ENUMARTIONS ---------------------------------*/
/*-------------------------------------------------------------------------------------------*/

typedef  void (*pFunction)(void);
pFunction Jump_To_Application = 0;
pFunction Jump_To_Boot = 0;
extern void FLASH_PageErase(uint32_t Page, uint32_t Banks);

typedef struct
{
	VERSION 	version;
	uint32_t	size;
	uint32_t	crc;
	IMAGE		upload_image;
} UPLOAD_PARAMETERS;

typedef struct
{
	PAGE_NUMBER first_page;
	PAGE_NUMBER	last_page;
} ERASE_PAGES;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern CRC_HandleTypeDef hcrc;
UPLOAD_PARAMETERS upload_parameters = { { { 5 , 6 , 0 , 0 } } , 0 , IMAGE_A , 0 };
uint32_t images_starting_address[NUMBER_OF_IMAGES] =
{
/* APPLICATION	   */ APPLICATION_STARTING_ADDRESS,
/* IMAGE_A		   */ VERSION_A_STARTING_ADDRESS,
/* IMAGE_B		   */ VERSION_B_STARTING_ADDRESS,
/* DEFAULT_VERSION */ DEFAULT_VERSION_STARTING_ADDRESS
};

ERASE_PAGES page_number[NUMBER_OF_IMAGES] =
{
/* APPLICATION	   */ { APPLICATION_FIRST_PAGE 		, APPLICATION_LAST_PAGE 	} ,
/* IMAGE_A		   */ { VERSION_A_FIRST_PAGE   		, VERSION_A_LAST_PAGE		} ,
/* IMAGE_B		   */ { VERSION_B_FIRST_PAGE  		, VERSION_B_LAST_PAGE		} ,
/* DEFAULT_VERSION */ { DEFAULT_VERSION_FIRST_PAGE	, DEFAULT_VERSION_LAST_PAGE }
};

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void jump_to_application()
{
     volatile unsigned long JumpAddress;

    __set_MSP(*(volatile unsigned long*) (APPLICATION_STARTING_ADDRESS));
    JumpAddress = *( uint32_t*) (APPLICATION_STARTING_ADDRESS+4);
    Jump_To_Application = (pFunction) JumpAddress;
    Jump_To_Application();
}

void jump_to_boot()
{
     volatile unsigned long JumpAddress;

    __set_MSP(*(volatile unsigned long*) (BOOTLOADER_STARTING_ADDRESS));
    JumpAddress = *( uint32_t*) (BOOTLOADER_STARTING_ADDRESS+4);
    Jump_To_Boot = (pFunction) JumpAddress;
    Jump_To_Boot();
}

void erase_application()
{
	for( uint16_t page_number = APPLICATION_FIRST_PAGE ; page_number <= APPLICATION_LAST_PAGE ; page_number++ )
		FLASH_PageErase( page_number , FLASH_BANK_1 );
}

void erase_version_a()
{
	for( uint16_t page_number = VERSION_A_FIRST_PAGE ; page_number <= VERSION_A_LAST_PAGE ; page_number++ )
	{
		if( page_number < 256 )
			FLASH_PageErase( page_number , FLASH_BANK_1 );
		else
			FLASH_PageErase( page_number , FLASH_BANK_2 );
	}
}

void erase_version_b()
{
	for( uint16_t page_number = VERSION_B_FIRST_PAGE ; page_number <= VERSION_B_LAST_PAGE ; page_number++ )
		FLASH_PageErase( page_number , FLASH_BANK_2 );
}

void copy_from_image_to_image( FLASH_ADDRESS source_start_address , FLASH_ADDRESS destination_start_address , uint32_t file_size )
{
	uint32_t *source_address = (uint32_t *)source_start_address;

	for( uint32_t destination_address = destination_start_address ; destination_address < file_size / 4 ; destination_address += 4 )
		HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD , destination_address , *source_address++ );
}

void copy_version_a_to_application()
{
	copy_from_image_to_image( VERSION_A_STARTING_ADDRESS , APPLICATION_STARTING_ADDRESS , eeprom_read_int(EEPROM_FILE_SIZE_IMAGE_A) );
}

void copy_version_b_to_application()
{
	copy_from_image_to_image( VERSION_B_STARTING_ADDRESS , APPLICATION_STARTING_ADDRESS , eeprom_read_int(EEPROM_FILE_SIZE_IMAGE_B) );
}

void copy_application_to_version_a()
{
	copy_from_image_to_image( APPLICATION_STARTING_ADDRESS , VERSION_A_STARTING_ADDRESS , eeprom_read_int(EEPROM_FILE_SIZE_IMAGE_A) );
}

void copy_application_to_version_b()
{
	copy_from_image_to_image( APPLICATION_STARTING_ADDRESS , VERSION_B_STARTING_ADDRESS , eeprom_read_int(EEPROM_FILE_SIZE_IMAGE_B) );
}

void flash_programming( uint32_t strat_of_source_address , uint32_t destination_address , uint32_t size)
{
	uint64_t data = 0;

	////HAL_FLASH_Unlock(); /* Unlock the Flash to enable the flash control register access *************/
    for( uint32_t source_address = strat_of_source_address ; source_address < strat_of_source_address + size ; destination_address += 8 , source_address += 8 )
	{
    	memcpy( &data , (uint8_t *)source_address , 8 );
    	HAL_FLASH_Unlock(); /* Unlock the Flash to enable the flash control register access *************/
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, destination_address, data );
		HAL_FLASH_Lock();

		//if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, destination_address, *( (uint64_t *)source_address ) ) != HAL_OK)
		/////if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, destination_address, data ) != HAL_OK)
			///break;
	}
	////HAL_FLASH_Lock();
}

void program_file_to_flash( FLASH_ADDRESS flash_address , uint8_t *file_pointer , uint32_t file_size )
{
	flash_programming( (uint32_t)file_pointer , (uint32_t)flash_address , file_size );
}
char upload_array[120];
void upload_set_upload_parameters(uint8_t *data)
{
   
	memcpy( &upload_parameters , data , sizeof(UPLOAD_PARAMETERS) );
	eeprom_image_clear_image_parameters(upload_parameters.upload_image);
	if( upload_parameters.upload_image < NUMBER_OF_IMAGES )
	    erase_flash_pages( page_number[upload_parameters.upload_image].first_page , page_number[upload_parameters.upload_image].last_page );
	sprintf( upload_array , "upload_image=%d version=%d size=%d\r\n" , upload_parameters.upload_image , (int)upload_parameters.version.version_word , (int)upload_parameters.size);
	send_message_to_pc( (char *)upload_array , strlen((char *)upload_array) );
}

//void upload_program_block( update_frame->block_number , update_frame->payload , update_frame->length , client_command - UPDATE_1 )
void upload_program_block( uint16_t block_number , uint8_t *data , uint32_t length )
{
	program_file_to_flash( images_starting_address[upload_parameters.upload_image] + block_number * UPLOADING_BLOCK_SIZE , data , UPLOADING_BLOCK_SIZE );
	// -1 for handling ( file size is exactly moltiple of UPLOADING_BLOCK_SIZE
	if( ( upload_parameters.size - 1 ) / UPLOADING_BLOCK_SIZE == block_number )
	{
		uint32_t calculated_crc = ~HAL_CRC_Calculate( &hcrc, (uint32_t *)images_starting_address[upload_parameters.upload_image] , upload_parameters.size );
		if( upload_parameters.crc == calculated_crc )
			eeprom_image_updated_programmed_image_parameters( upload_parameters.upload_image , upload_parameters.version.version_word , upload_parameters.size , upload_parameters.crc );
	}

}

void upload_copy_blocks( IMAGE source_image , IMAGE destination_image )
{
	eeprom_image_copy_image_parameters( source_image , destination_image );
}

