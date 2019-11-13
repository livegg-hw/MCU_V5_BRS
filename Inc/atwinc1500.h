/*
 * atwinc1500.h
 *
 *  Created on: 26 áôáø× 2018
 *      Author: oferf
 */

#ifndef ATWINC1500_H_
#define ATWINC1500_H_

void atwinc1500_initialization();
void atwinc1500_connection_loop();
void atwinc1500_send_wifi_message( uint8_t *message , uint16_t message_length );
void set_wifi_port_number(int16_t new_port_number);
void get_wifi_port_number();
void atwinc1500_reset_connection_attempt_failed_timer();
uint32_t atwinc1500_get_ip_number();

#endif /* ATWINC1500_H_ */
