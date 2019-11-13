/*
 * environment_log.h
 *
 *  Created on: 3 במרץ 2019
 *      Author: oferf
 */

#ifndef ENVIRONMENT_LOG_H_
#define ENVIRONMENT_LOG_H_

void environment_log_update_time_and_date( uint32_t data , uint32_t time );
void environment_log_update_record();
void environment_log_reset_log();
char *environment_log_get_environment_log_pointer();



#endif /* ENVIRONMENT_LOG_H_ */
