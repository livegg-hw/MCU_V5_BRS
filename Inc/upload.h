/*
 * download.h
 *
 *  Created on: 25 באפר׳ 2018
 *      Author: oferf
 */

#ifndef UPLOAD_H_
#define UPLOAD_H_

#define UPLOADING_BLOCK_SIZE		1024

void flash_programming( uint32_t source_address , uint32_t destination_address , uint32_t size);
void upload_program_block( uint16_t block_number , uint8_t *data , uint32_t length );
void upload_set_upload_parameters(uint8_t *data);
//void upload_copy_blocks( IMAGE source_image , IMAGE destination_image );

#endif /* UPLOAD_H_ */
