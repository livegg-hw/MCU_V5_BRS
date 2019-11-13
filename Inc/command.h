/*
 * command.h
 *
 *  Created on: 21 במרץ 2018
 *      Author: oferf
 */

#ifndef COMMAND_H_
#define COMMAND_H_

void command_parsing( uint8_t * input_command );
void send_message_to_pc( char *message , uint16_t message_size );

#endif /* COMMAND_H_ */
