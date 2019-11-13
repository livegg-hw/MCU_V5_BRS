/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : eeprom_image.c                                                                                 *
 *  Abstract   : EEPROM image handling.                                                                         *
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : FEBRUARY 2018                                                                                  *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include "stm32l4xx_hal.h"
#include "internal_flash.h"
#include "eeprom_image.h"
#include "upload.h"

/*-------------------------------------------------------------------------------------------*/
/*----------------------------- DEFINITIONS AND ENUMARTIONS ---------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define SIGNATURE					 	0xAABB
#define EEPROM_IMAGE_SIZE				( sizeof(eeprom_image) - sizeof(eeprom_image.spare) )

typedef struct
{
	uint8_t source_image      : 4;
	uint8_t destination_image : 4;
} TO_COPY;

typedef struct
{
	VERSION  version;
	uint32_t file_size;
	uint32_t crc;
	uint32_t spare;
} IMAGE_DETAILS;

typedef struct
{
    uint16_t 				signature;
    IMAGE_PROGRAMMED_STATUS programming_status;
    TO_COPY	  				images_to_copy;
    IMAGE_DETAILS			image_details[NUMBER_OF_IMAGES];
    uint8_t  				spare[1980];
} EEPROM_IMAGE;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
EEPROM_IMAGE eeprom_image =
{
  /* SIGNATURE                  */ SIGNATURE,
  /* IMAGE_PROGRAMMED_STATUS    */ { { 1 , 0 , 0 , 0 , 0 } },
  /* TO_COPY    		        */ { IMAGE_A , IMAGE_A },
  /* IMAGE_DETAILS			 	*/ { { { { 5 , 0 , 0 , 0 } } , 0 , 0 , 0 } , { { { 5 , 0 , 0 , 0 } } , 0 , 0 , 0 } , { { { 5 , 0 , 0 , 0 } } , 0 , 0 , 0 } , { { { 5 , 0 , 0 , 0 } } , 0 , 0 , 0 } }
};

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void erase_flash_pages( uint32_t first_page , uint32_t last_page )
{
	FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError = 0;
    uint32_t page_index;

	HAL_FLASH_Unlock(); 						/* Unlock the Flash to enable the flash control register access *************/
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); /* Clear OPTVERR bit set on virgin samples */

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.NbPages   = 1;
	for( page_index = first_page ; page_index <= last_page ; page_index++ )
	{
		if( page_index < 256 )
			EraseInitStruct.Banks = FLASH_BANK_1;
		else
			EraseInitStruct.Banks = FLASH_BANK_2;
		EraseInitStruct.Page = page_index;
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK);
	}
	HAL_FLASH_Lock();
}

void store_eeprom_image_into_flash()
{
    erase_flash_pages( EEPROM_IMAGE_PAGE , EEPROM_IMAGE_PAGE );
    flash_programming( (uintptr_t)&eeprom_image , EEPROM_IMAGE_STARTING_ADDRESS , EEPROM_IMAGE_SIZE );
}

void load_from_eeprom_image()
{
    uint32_t flash_address;
    uint8_t  *eeprom_image_pointer = (uint8_t *)&eeprom_image;
    
    for( flash_address = EEPROM_IMAGE_STARTING_ADDRESS ; flash_address < EEPROM_IMAGE_STARTING_ADDRESS + EEPROM_IMAGE_SIZE ; flash_address++ )
        *eeprom_image_pointer++ = *(uint8_t *)flash_address;
}

void eeprom_image_initialization()
{
    if( *( (uint16_t *)EEPROM_IMAGE_STARTING_ADDRESS ) != SIGNATURE )
    {
        eeprom_image.signature = SIGNATURE;
        store_eeprom_image_into_flash();
    }
    load_from_eeprom_image();
}

uint32_t eeprom_image_get_image_version(IMAGE image)
{
	return eeprom_image.image_details[image].version.version_word;
}

uint32_t eeprom_image_get_crc(IMAGE image)
{
	return eeprom_image.image_details[image].crc;
}

void eeprom_image_copy_image_parameters( IMAGE source_image , IMAGE destination_image )
{
	eeprom_image.images_to_copy.destination_image = destination_image;
	eeprom_image.images_to_copy.source_image	  = source_image;
	store_eeprom_image_into_flash();
}

void eeprom_image_clear_image_parameters(IMAGE image)
{
	eeprom_image.image_details[image].version.version_word = 0;
	eeprom_image.image_details[image].file_size            = 0;
	eeprom_image.image_details[image].crc		           = 0;
	eeprom_image.programming_status.programmed_status_byte &= ~( 0x01 << image);
	store_eeprom_image_into_flash();
}

void eeprom_image_updated_programmed_image_parameters( IMAGE image , uint32_t version , uint32_t size , uint32_t crc )
{
	eeprom_image.image_details[image].version.version_word = version;
	eeprom_image.image_details[image].file_size            = size;
	eeprom_image.image_details[image].crc		           = crc;
	eeprom_image.programming_status.programmed_status_byte |=  ( 0x01 << image);
	store_eeprom_image_into_flash();
}
