/*
 * macronix_flash.c
 *
 *  Created on: 10 במאי 2018
 *      Author: Israel
 *      This Driver is for Macronix MX25L25635F SPI Flash memory
 *      http://www.macronix.com/Lists/Datasheet/Attachments/6729/MX25L25635F,%203V,%20256Mb,%20v1.5.pdf
 */

#include <stdint.h>
#include <stdbool.h>
#include "macronix_flash.h"
#include "stm32l4xx_hal.h"
#include "string.h"           // to use memcpy()


/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define TIMEOUT                    100000
#define DUMMY                      0x00
#define MAXIMUM_TRANSFERRING_BYTES 256

#define RDID                       0x9F //return Manufactory ID + Memory type + Memory density
#define RES                        0xAB //return Electronic ID
#define REMS                       0x90 //return Manufactory ID + Device ID
#define ENTER_4BYTE_ADD_MODE       0xB7 //enter 4-Byte Address Mode
#define CHIP_ERASE                 0xC7 //60
#define WRITE_DATA                 0x12
#define READ_DATA                  0x13
#define WRITE_DISABLE              0x04
#define WRITE_ENABLE               0x06
#define SECTOR_ERASE_4K            0x21 //Sector size: 4K  Byte
#define BLOCK_ERASE_32K            0x5C //Block  size: 32K Byte
#define BLOCK_ERASE_64K            0xDC //Block  size: 64K Byte
#define SATUS_REGISTER             0x05

#define SPI_NSS_PIN_HIGH  HAL_GPIO_WritePin( GPIOE, GPIO_PIN_12, GPIO_PIN_SET   );
#define SPI_NSS_PIN_LOW   HAL_GPIO_WritePin( GPIOE, GPIO_PIN_12, GPIO_PIN_RESET );

typedef union
{
	uint32_t address_word;
	uint8_t  address_array[4];
} ADDRESS_UNION ;

typedef struct __attribute__((packed))
{
	uint8_t       command;
	ADDRESS_UNION address;
} ADDRESS_STRUCT ;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/

uint8_t macronix_flash_spi_read_byte(uint8_t command)
{
	uint8_t result[1]={0};
	SPI_NSS_PIN_LOW;
	HAL_SPI_Transmit(&hspi1,  (uint8_t [1]){ command }, 1, TIMEOUT);
	HAL_SPI_Receive(&hspi1,  &result[0], 1, TIMEOUT);
	SPI_NSS_PIN_HIGH;
	return result[0];
}

/* Use this function to:
 * 1)Read Electronic Signature (RES)
 * 2)Read Electronic Manufacturer & Device ID (REMS)
 *
 * The Sequence:
 * 1)Transmit: command + 3 DUMMY bytes
 * 2)Receive : 4 bytes
 */
uint8_t* macronix_flash_spi_read_bytes_id(uint8_t command, uint8_t data_size)
{
	static uint8_t result[2]={0};
	SPI_NSS_PIN_LOW;
	HAL_SPI_Transmit(&hspi1,  (uint8_t [4]){ command,DUMMY,DUMMY,DUMMY }, 4, TIMEOUT);
	HAL_SPI_Receive(&hspi1,  &result[0], data_size , TIMEOUT);
	SPI_NSS_PIN_HIGH;
	return (uint8_t*)result;
}

void macronix_flash_spi_read_bytes(uint32_t address , uint8_t command , uint32_t data_size , uint8_t* pData  )
{
	SPI_NSS_PIN_LOW;
	//------------ code improvement --------------------------------------
	ADDRESS_STRUCT cmd_sequence ;
	cmd_sequence.command = command;
	cmd_sequence.address.address_word = __REV(address);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&cmd_sequence , 5 , TIMEOUT);
	HAL_SPI_Receive(&hspi1,  (uint8_t *)&pData[0] , data_size, TIMEOUT);
	//--------------------------------------------------------------------
	SPI_NSS_PIN_HIGH;
}

void macronix_flash_spi_write_byte(uint8_t command)
{
	SPI_NSS_PIN_LOW;
	HAL_SPI_Transmit(&hspi1,  (uint8_t [1]){ command }, 1, TIMEOUT);
	SPI_NSS_PIN_HIGH;
}

uint8_t macronix_flash_read_id()
{
	return (uint8_t)macronix_flash_spi_read_byte(RDID);
}

uint8_t* macronix_flash_read_electronic_signature()
{
	return macronix_flash_spi_read_bytes_id(RES,2);
}

uint8_t* macronix_flash_read_electronic_manufacturer_n_device_ids()
{
	return macronix_flash_spi_read_bytes_id(REMS,2);//(uint8_t*)
}

void macronix_flash_write_disable()
{
	macronix_flash_spi_write_byte(WRITE_DISABLE);
}

void macronix_flash_write_enable()
{
	macronix_flash_spi_write_byte(WRITE_ENABLE);
}

void macronix_flash_spi_write_bytes( uint32_t address , uint8_t command , uint8_t *data , uint32_t data_size )
{
	macronix_flash_write_enable();
	SPI_NSS_PIN_LOW;
	static uint8_t transmitted_data[MAXIMUM_TRANSFERRING_BYTES+5]={0}; //Command+4BytesAddress+256DataBytes
	//------------ code improvement --------------------------------------
	ADDRESS_STRUCT cmd_sequence ;
	cmd_sequence.command = command;
	cmd_sequence.address.address_word = __REV(address);

	/* first, copy to transmitted_data[] the command and the address */
	memcpy(transmitted_data, (uint8_t *)&cmd_sequence , 5);

	/* Second, copy to the entire data to transmitted_data[] */
	for( uint16_t data_index = 0 ; data_index < data_size ; data_index++ )
	{
		transmitted_data[ data_index+5 ] = data[data_index];
	}
	HAL_SPI_Transmit(&hspi1, transmitted_data , data_size + 5 , TIMEOUT);
	//--------------------------------------------------------------------
	SPI_NSS_PIN_HIGH;
	macronix_flash_write_disable();
}

void macronix_flash_write_data_to_flash(uint32_t address , uint8_t *data , uint32_t data_size)
{
	/* flashing data of 256 Bytes and Up */
	while(data_size >= MAXIMUM_TRANSFERRING_BYTES )
	{
		macronix_flash_write_enable_latch();  //WEL bit
		macronix_flash_write_in_progress();  //WIP bit
		macronix_flash_spi_write_bytes(  address , WRITE_DATA , data  , MAXIMUM_TRANSFERRING_BYTES );
		data_size -= MAXIMUM_TRANSFERRING_BYTES;
		data      += MAXIMUM_TRANSFERRING_BYTES;
		address   += MAXIMUM_TRANSFERRING_BYTES;
		macronix_flash_write_in_progress();  //WIP bit
	}
	/* flashing the rest of the data (less then 256 Bytes) */
	if(data_size < MAXIMUM_TRANSFERRING_BYTES )
	{
		macronix_flash_write_enable_latch();  //WEL bit
		macronix_flash_write_in_progress();  //WIP bit
		macronix_flash_spi_write_bytes(  address , WRITE_DATA , data  , data_size );
		macronix_flash_write_in_progress();  //WIP bit
	}
}

void macronix_flash_read_data_from_flash(uint32_t address , uint8_t* pData , uint32_t data_size )
{
	macronix_flash_spi_read_bytes( address , READ_DATA , data_size , pData);
}

void macronix_flash_chip_erase()
{
	/*---------must send WRITE_ENABLE before CHIP_ERASE--------*/
	macronix_flash_write_enable_latch();  //WEL bit
	macronix_flash_write_in_progress();  //WIP bit
	macronix_flash_spi_write_byte(CHIP_ERASE);
	macronix_flash_write_in_progress();  //WIP bit
	macronix_flash_write_disable();
}

void macronix_flash_sector_4k_erase(uint32_t address)
{
	/*
	 * because macronix_flash_spi_write_bytes() need a pointer
	 * we will use a dummy pointer and initialize it to MULL
	 */
	macronix_flash_write_enable_latch();  //WEL bit
	macronix_flash_write_in_progress();  //WIP bit
	uint8_t *p=NULL;
	macronix_flash_spi_write_bytes(  address , SECTOR_ERASE_4K , (uint8_t *)&p  , 0 );
	macronix_flash_write_in_progress();  //WIP bit
	macronix_flash_write_disable();
}

void macronix_flash_block_32k_erase(uint32_t address)
{
	macronix_flash_write_enable_latch();  //WEL bit
	macronix_flash_write_in_progress();  //WIP bit
	uint8_t *p=NULL;
	macronix_flash_spi_write_bytes(  address , BLOCK_ERASE_32K , (uint8_t *)&p  , 0 );
	macronix_flash_write_in_progress();  //WIP bit
	macronix_flash_write_disable();
}

void macronix_flash_block_64k_erase(uint32_t address)
{
	macronix_flash_write_enable_latch();  //WEL bit
	macronix_flash_write_in_progress();  //WIP bit
	uint8_t *p=NULL;
	macronix_flash_spi_write_bytes(  address , BLOCK_ERASE_64K , (uint8_t *)&p  , 0 );
	macronix_flash_write_in_progress();  //WIP bit
	macronix_flash_write_disable();
}

void macronix_flash_write_in_progress()
{
	while( (macronix_flash_spi_read_byte(SATUS_REGISTER)&0x01) == 0x01 ) { }
}

void macronix_flash_write_enable_latch()
{
	do
	{
		macronix_flash_write_enable();
	}while( (macronix_flash_spi_read_byte(SATUS_REGISTER)&0x02) == 0x00 );
}

void macronix_flash_enter_4byte_address_mode()
{
	macronix_flash_spi_write_byte(ENTER_4BYTE_ADD_MODE);
}


