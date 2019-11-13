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

/*-------------------------------------------------------------------------------------------*/
/*----------------------------- DEFINITIONS AND ENUMARTIONS ---------------------------------*/
/*-------------------------------------------------------------------------------------------*/

typedef  void (*pFunction)(void);
pFunction Jump_To_Application = 0;
extern void FLASH_PageErase(uint32_t Page, uint32_t Banks);

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/

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

void flash_programming( uint64_t strat_of_source_address , uint32_t destination_address , uint32_t size)
{
    HAL_FLASH_Unlock(); /* Unlock the Flash to enable the flash control register access *************/
    for( uint64_t source_address = strat_of_source_address ; source_address < strat_of_source_address + size ; destination_address += 8 , source_address += 8 )
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, destination_address, *( (uint64_t *)( (uint32_t)source_address ) ) ) != HAL_OK)
			break;
	}
	HAL_FLASH_Lock();
}

void program_file_to_flash( FLASH_ADDRESS flash_address , uint8_t *file_pointer , uint32_t file_size )
{
	flash_programming( (uint64_t)flash_address , (uint32_t)file_pointer , file_size );
}
