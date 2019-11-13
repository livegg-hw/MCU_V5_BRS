/*
 * macronix_flash.h
 *
 *  Created on: 10 במאי 2018
 *      Author: Israel
 */

#ifndef MACRONIX_FLASH_H_
#define MACRONIX_FLASH_H_

#include <stdbool.h>

uint8_t macronix_flash_spi_read_byte(uint8_t command);
uint8_t* macronix_flash_spi_read_bytes_id(uint8_t command , uint8_t data_size);
void macronix_flash_spi_read_bytes(uint32_t address , uint8_t command , uint32_t data_size , uint8_t* pData);

uint8_t macronix_flash_read_id();
uint8_t* macronix_flash_read_electronic_signature();
uint8_t* macronix_flash_read_electronic_manufacturer_n_device_ids();

void macronix_flash_spi_write_byte(uint8_t command);
void macronix_flash_spi_write_bytes( uint32_t address , uint8_t command , uint8_t *data , uint32_t data_size );

void macronix_flash_write_data_to_flash(uint32_t address , uint8_t *data , uint32_t data_size);
void macronix_flash_read_data_from_flash(uint32_t address , uint8_t* pData , uint32_t data_size );

void macronix_flash_write_disable();
void macronix_flash_write_enable();
void macronix_flash_chip_erase();
void macronix_flash_sector_4k_erase(uint32_t address);
void macronix_flash_block_32k_erase(uint32_t address);
void macronix_flash_block_64k_erase(uint32_t address);
void macronix_flash_write_in_progress();
void macronix_flash_write_enable_latch();
void macronix_flash_enter_4byte_address_mode();


#endif /* MACRONIX_FLASH_H_ */
