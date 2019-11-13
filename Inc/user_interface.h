/*
 * user_interface.h
 *
 *  Created on: 21 במרץ 2018
 *      Author: oferf
 */

#ifndef USER_INTERFACE_H_
#define USER_INTERFACE_H_

typedef enum
{
    NORMAL,
    DURING_FILE_RECEIVE,
    DURING_FILE_SEND
} RECEIVING_STATUS;

void uart1_initialization();
void user_interface_recieved_polling();

#endif /* USER_INTERFACE_H_ */
