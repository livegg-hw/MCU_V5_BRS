/*
 * client.h
 *
 *  Created on: 22 באפר׳ 2018
 *      Author: oferf
 */

#ifndef CLIENT_H_
#define CLIENT_H_

void client_receive_protocol( uint8_t *data , uint16_t size );
void send_wifi_message( uint8_t *message , uint16_t message_length );

#endif /* CLIENT_H_ */
