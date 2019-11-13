/*
 * tray_id.h
 *
 *  Created on: 7 бреб„ 2018
 *      Author: oferf
 */

#ifndef TRAY_ID_H_
#define TRAY_ID_H_

typedef enum
{
	REGULAR_TRAY,
	GENDER_TRAY
} TRAY_TYPE;

uint16_t port_number_getting();
uint32_t tray_id_and_port_number_getting( char * tray_id , uint16_t * port_number );
uint32_t tray_id_and_port_number_setting( char * new_tray_id , uint16_t port_number );
void set_tray_id(uint8_t *data);
void get_tray_id();
TRAY_TYPE tray_id_get_tray_type();
void tray_id_copy_tray_id(char *tray_id_pointer);
void extCo2BoardMsgHandle(uint8_t *data);
uint32_t getCo2LevelFromExtBoard();


#endif /* TRAY_ID_H_ */
