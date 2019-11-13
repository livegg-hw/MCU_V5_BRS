/*
 * sensors.h
 *
 *  Created on: 19 באפר׳ 2018
 *      Author: oferf
 */

#ifndef SENSORS_H_
#define SENSORS_H_

typedef enum
{
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
	NUMBER_OF_AXES
} AXIS;

typedef struct
{
	int16_t x_axis;
	int16_t y_axis;
	int16_t z_axis;
} SAMPLE;

typedef struct
{
	float x_axis;
	float y_axis;
	float z_axis;
} ACCURATE_SAMPLE;

#endif /* SENSORS_H_ */
