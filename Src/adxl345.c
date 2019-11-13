/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : adxl345_acc.c                                                                                  *
 *  Abstract   : Digital output accelerometer sensor with I2C Interface.                                        *
 *				 http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf                 *
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : APRIL 2018                                                                                     *
 *--------------------------------------------------------------------------------------------------------------*/

#include <string.h>
#include <stdbool.h>
#include "math.h"
#include "stm32l4xx_hal.h"
#include "i2c.h"
#include "sensors.h"
#include "adxl345.h"
#include "command.h"

/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
//#define ADXL345_SLAVE_ADDRESS       0x3A
#define ADXL345_SLAVE_ADDRESS       0xA6
#define PI       					3.141592654f
#define HALF_PI  					1.570796327f
#define RAD2DEG_FACTOR 				57.2957795131f
#define SQR_F(X)  					((float)X*(float)X)

typedef enum
{
	DEVICE_ID		 		= 0x00,
	THRESHOLD_TAP	 		= 0x1D,
	OFSX			 		= 0x1E,
	OFSY			 		= 0x1F,
	OFSZ			 		= 0x20,
	DUR				 		= 0x21,
	LATENT			 		= 0x22,
	WINDOW			 		= 0x23,
	THRESHOLD_ACTIVITY 		= 0x24,
	THRESHOLD_INACTIVITY 	= 0x25,
	TIME_INACT		 		= 0x26,
	ACTIVE_INACTIVE_CONTROL = 0x27,
	THRESH_FF				= 0x28,
	TIME_FF					= 0x29,
	TAP_AXES				= 0x2A,
	ACT_TAP_STATUS   		= 0x2B,
	BW_RATE					= 0x2C,
	POWER_CTL       		= 0x2D,
	INTERRUPT_ENABLE		= 0x2E,
	INTERRUPT_MAPPING		= 0x2F,
	INTERRUPT_SOURCE		= 0x30,
	DATA_FORMAT				= 0x31,
	DATAX0					= 0x32,
	DATAX1					= 0x33,
	DATAY0					= 0x34,
	DATAY1					= 0x35,
	DATAZ0					= 0x36,
	DATAZ1					= 0x37,
	FIFO_CTL				= 0x38,
	FIFO_STATUS				= 0x39
} ADXL345_REGISTER;

typedef enum
{
	OVERRUN     = 0x01,
	WATERMARK	= 0x02,
	FREE_FALL	= 0x04,
	INACTIVITY	= 0x08,
	ACTIVITY	= 0x10,
	DOUBLE_TAP	= 0x20,
	SINGLE_TAP	= 0x40,
	DATA_READY	= 0x80
} INTERRUPT_ENABLE_REGISTER;

typedef enum
{
	OVERRUN_ROUTED_TO_PIN_1 	= 0x00,
	OVERRUN_ROUTED_TO_PIN_2 	= 0x01,
	WATERMARK_ROUTED_TO_PIN_1	= 0x00,
	WATERMARK_ROUTED_TO_PIN_2	= 0x02,
	FREE_FALL_ROUTED_TO_PIN_1	= 0x00,
	FREE_FALL_ROUTED_TO_PIN_2	= 0x04,
	INACTIVITY_ROUTED_TO_PIN_1	= 0x00,
	INACTIVITY_ROUTED_TO_PIN_2	= 0x08,
	ACTIVITY_ROUTED_TO_PIN_1	= 0x00,
	ACTIVITY_ROUTED_TO_PIN_2	= 0x10,
	DOUBLE_TAP_ROUTED_TO_PIN_1	= 0x00,
	DOUBLE_TAP_ROUTED_TO_PIN_2	= 0x20,
	SINGLE_TAP_ROUTED_TO_PIN_1	= 0x00,
	SINGLE_TAP_ROUTED_TO_PIN_2	= 0x40,
	DATA_READY_ROUTED_TO_PIN_1	= 0x00,
	DATA_READY_ROUTED_TO_PIN_2	= 0x80
} INTERRUPT_MAPPING_REGISTER;

typedef enum
{
	INACTIVITY_Z_ENABLE = 0x01,
	INACTIVITY_Y_ENABLE = 0x02,
	INACTIVITY_X_ENABLE = 0x04,
	INACTIVITY_DC		= 0x00,
	INACTIVITY_AC		= 0x08,
	ACTIVITY_Z_ENABLE 	= 0x10,
	ACTIVITY_Y_ENABLE 	= 0x20,
	ACTIVITY_X_ENABLE 	= 0x40,
	ACTIVITY_DC	    	= 0x00,
	ACTIVITY_AC    		= 0x80
} ACTIVITY_INACTIVITY_CONTROL_REGISTER;

typedef enum
{
	RANGE_2G			  = 0x00,
	RANGE_4G			  = 0x01,
	RANGE_8G			  = 0x02,
	RANGE_16G			  = 0x03,
	RIGHT_JUSTIFY		  = 0x00,
	LEFT_JUSTIFY		  = 0x04,
	FULL_RESOLUTION		  = 0x08,
	ACTIVE_HIGH_INTERRUPT = 0x00,
	ACTIVE_LOW_INTERRUPT  = 0x20,
	SPI_4_WIRE_MODE		  = 0x00,
	SPI_3_WIRE_MODE		  = 0x40,
	SELF_TEST			  = 0x80
} DATA_FORMAT_REGISTER;
/*
typedef enum
{
	OUTPUT_DATA_RATE
};*/
/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
volatile uint8_t id;
int16_t x,y,z;
uint8_t ss = 1;
SAMPLE samples_array;
uint8_t x_lsb , x_msb , y_lsb , y_msb , z_lsb , z_msb , format;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void adxl345_initialization(void)
{
	id = I2C2_read_byte( ADXL345_SLAVE_ADDRESS , DEVICE_ID );
	I2C2_write_byte( ADXL345_SLAVE_ADDRESS , DATA_FORMAT 	   , RANGE_16G | FULL_RESOLUTION | RIGHT_JUSTIFY | ACTIVE_HIGH_INTERRUPT ); // ±16g, up to 13-BIT MODE full resolution mode
	I2C2_write_byte( ADXL345_SLAVE_ADDRESS , BW_RATE	   	   , 0x0C ); // Output Data Rate 400Hz
	I2C2_write_byte( ADXL345_SLAVE_ADDRESS , POWER_CTL  	   , 0x08 ); // START MEASUREMENT
	I2C2_write_byte( ADXL345_SLAVE_ADDRESS , INTERRUPT_ENABLE  , 0x80 ); // ENABLE DATA_READY INTERRUPT.
	I2C2_write_byte( ADXL345_SLAVE_ADDRESS , INTERRUPT_MAPPING , 0x7F ); // map DATA_READY interrupt to INT1 pin.
	//while(ss)
	{
		adxl345_get_accelerometer_sample(&samples_array);
		format = I2C2_read_byte( ADXL345_SLAVE_ADDRESS , DATA_FORMAT );
		x = adxl345_get_accelerometer_axis_sample(AXIS_X);
		y = adxl345_get_accelerometer_axis_sample(AXIS_Y);
		z = adxl345_get_accelerometer_axis_sample(AXIS_Z);
		x_lsb =  I2C2_read_byte( ADXL345_SLAVE_ADDRESS , DATAX0 );
		x_msb =  I2C2_read_byte( ADXL345_SLAVE_ADDRESS , DATAX1 );
		y_lsb =  I2C2_read_byte( ADXL345_SLAVE_ADDRESS , DATAY0 );
		y_msb =  I2C2_read_byte( ADXL345_SLAVE_ADDRESS , DATAY1 );
		z_lsb =  I2C2_read_byte( ADXL345_SLAVE_ADDRESS , DATAZ0 );
		z_msb =  I2C2_read_byte( ADXL345_SLAVE_ADDRESS , DATAZ1 );
	}
}

void adxl345_clear_interrupt()
{
	I2C2_read_byte( ADXL345_SLAVE_ADDRESS , INTERRUPT_SOURCE );
}

uint8_t adxl345_get_device_id()
{
	return I2C2_read_byte( ADXL345_SLAVE_ADDRESS , DEVICE_ID );
}

uint8_t adxl345_get_register(ADXL345_REGISTER adxl345_register)
{
	return I2C2_read_byte( ADXL345_SLAVE_ADDRESS , adxl345_register );
}

uint16_t adxl345_get_accelerometer_axis_sample(AXIS axis)
{
	return I2C2_read_word( ADXL345_SLAVE_ADDRESS , DATAX0 + (ADXL345_REGISTER)axis * 2 );
}

void adxl345_get_accelerometer_sample(SAMPLE *sample_pointer)
{
	I2C2_read_array( ADXL345_SLAVE_ADDRESS , DATAX0 , 6 , (uint8_t *)sample_pointer );
}

void adxl345_get_accelerometer_accurate_sample(ACCURATE_SAMPLE *accurate_sample)
{
	SAMPLE sample;

	adxl345_get_accelerometer_sample(&sample);
	accurate_sample->x_axis = ( sample.x_axis / 256.0F );
	accurate_sample->y_axis = ( sample.y_axis / 256.0F );
	accurate_sample->z_axis = ( sample.z_axis / 256.0F );
}

void adxl345_get_accelerometer_sample_FeedbackFiltered(ACCURATE_SAMPLE *accurate_sample , const float c1, const float maxDiff, const uint32_t minMeasurements, const uint32_t maxMeasurements)
{
	const float c0 = 1 - c1;
	uint32_t index = 1;
	bool isStable = false;
	ACCURATE_SAMPLE previous_accurate_sample;

	adxl345_get_accelerometer_accurate_sample(accurate_sample);
	do
	{
		previous_accurate_sample.x_axis = accurate_sample->x_axis;
		previous_accurate_sample.y_axis = accurate_sample->y_axis;
		previous_accurate_sample.z_axis = accurate_sample->z_axis;

		adxl345_get_accelerometer_accurate_sample(accurate_sample);
		++index;

		accurate_sample->x_axis = previous_accurate_sample.x_axis * c0 + accurate_sample->x_axis * c1;
		accurate_sample->y_axis = previous_accurate_sample.y_axis * c0 + accurate_sample->y_axis * c1;
		accurate_sample->z_axis = previous_accurate_sample.z_axis * c0 + accurate_sample->z_axis * c1;

		isStable =
			fabs(accurate_sample->x_axis - previous_accurate_sample.x_axis) < maxDiff &&
			fabs(accurate_sample->y_axis - previous_accurate_sample.y_axis) < maxDiff &&
			fabs(accurate_sample->z_axis - previous_accurate_sample.z_axis) < maxDiff;

	}while(index < maxMeasurements && (index < minMeasurements || isStable == false));
}
/*
uint8_t GetAccXYZ_FeedbackFiltered(AccMeasureFloat_TypeDef* accMeasure, const float c1, const float maxDiff, const uint32_t minMeasurements, const uint32_t maxMeasurements)
{
	const float c0 = 1 - c1;
	uint32_t i = 0;
	uint8_t isStable = 0;
	
	AccMeasureFloat_TypeDef _accMeasurePrev;
	AccMeasureFloat_TypeDef* accMeasurePrev = &_accMeasurePrev;
	
	GetAccXYZ_Float(accMeasure);
	++i;
	
	do
	{
		accMeasurePrev->x = accMeasure->x;
		accMeasurePrev->y = accMeasure->y;
		accMeasurePrev->z = accMeasure->z;
		
		GetAccXYZ_Float(accMeasure);
		++i;
		
		accMeasure->x = accMeasurePrev->x * c0 + accMeasure->x * c1;
		accMeasure->y = accMeasurePrev->y * c0 + accMeasure->y * c1;
		accMeasure->z = accMeasurePrev->z * c0 + accMeasure->z * c1;
		
		isStable = 
			fabs(accMeasure->x - accMeasurePrev->x) < maxDiff &&
			fabs(accMeasure->y - accMeasurePrev->y) < maxDiff &&
			fabs(accMeasure->z - accMeasurePrev->z) < maxDiff;
		
//		ComUART_Printf("i = %i\r\n", i);
//		ComUART_Printf("diff x = %f\r\n", fabs(accMeasure->x - accMeasurePrev->x));
//		ComUART_Printf("diff y = %f\r\n", fabs(accMeasure->y - accMeasurePrev->y));
//		ComUART_Printf("diff z = %f\r\n", fabs(accMeasure->z - accMeasurePrev->z));
		
	}while(i < maxMeasurements && (i < minMeasurements || !isStable));
	
//	ComUART_PrintString("\r\n");
	return 0;
}
*/
void adxl345_get_accelerometer_average_filtered_sample( ACCURATE_SAMPLE *accurate_sample , uint8_t number_of_samples )
{
	uint8_t  sample_index;
	SAMPLE   sample;
	uint32_t x_axis_sum = 0 , y_axis_sum = 0 , z_axis_sum = 0;
	float    acceleration;

	for( sample_index = 0 ; sample_index < number_of_samples ; sample_index++ )
	{
		adxl345_get_accelerometer_sample(&sample);
		x_axis_sum += sample.x_axis;
		y_axis_sum += sample.y_axis;
		z_axis_sum += sample.z_axis;
	}
	accurate_sample->x_axis = (float)x_axis_sum / number_of_samples;
	accurate_sample->y_axis = (float)y_axis_sum / number_of_samples;
	accurate_sample->z_axis = (float)z_axis_sum / number_of_samples;

	acceleration = sqrtf(SQR_F(accurate_sample->x_axis) +SQR_F(accurate_sample->y_axis) +SQR_F(accurate_sample->z_axis));

	accurate_sample->x_axis = RAD2DEG_FACTOR * asinf( x_axis_sum / acceleration );
	accurate_sample->y_axis = RAD2DEG_FACTOR * asinf( y_axis_sum / acceleration );
	accurate_sample->z_axis = RAD2DEG_FACTOR * asinf( z_axis_sum / acceleration );
}
/*
uint8_t GetAccXYZ_AverageFiltered(AccMeasureFloat_TypeDef* accMeasure, const uint32_t numMeasurements)
{
	int xSum = 0, ySum = 0, zSum = 0;
	uint32_t i = 0;
	int ret = 0;
	float a = 0;
	AccMeasureInt16_TypeDef accMeasureInt16;
	
	for(i = 0; i < numMeasurements; ++i)
	{
		//while(GetAccXYZ_Int16(&accMeasureInt16) != 0);
		ret = GetAccXYZ_Int16(&accMeasureInt16);
		
		xSum += accMeasureInt16.x;
		ySum += accMeasureInt16.y;
		zSum += accMeasureInt16.z;
	}
	// the gravity measurement
  accMeasure->x = (float)xSum / numMeasurements;
  accMeasure->y =	(float)ySum / numMeasurements;
  accMeasure->z =	(float)zSum / numMeasurements;
	
	// convert gravity to angle (in degres)
	
	a = sqrtf(SQR_F(accMeasure->x) +SQR_F(accMeasure->y) +SQR_F(accMeasure->z));
	
	
	accMeasure->x = 57.29577951f * asinf(accMeasure->x/a);
	accMeasure->y = 57.29577951f * asinf(accMeasure->y/a);
	accMeasure->z = 57.29577951f * asinf(accMeasure->z/a);
	
	
//	ComUART_PrintString("\r\n");
	return 0;
}*/

void adxl345_get_accelerometer_to_ground_matrix(ACCURATE_SAMPLE accurate_sample, float* R)
{
	float accRef[3] = {0, 1, 0};
	static float theta;
	static float tmp, x, y, z, w;

	// Normalize x, y, z
	tmp = sqrt(SQR_F(accurate_sample.x_axis) + SQR_F(accurate_sample.y_axis) + SQR_F(accurate_sample.z_axis));
	accurate_sample.x_axis /= tmp;
	accurate_sample.y_axis /= tmp;
	accurate_sample.z_axis /= tmp;

	// Rotate acc axis to cam axis (180 degrees z-axis rotation)
	accurate_sample.x_axis = -accurate_sample.x_axis;
	accurate_sample.y_axis = -accurate_sample.y_axis;


	// Calc rotatoin matrix

	theta = acosf(accurate_sample.x_axis * accRef[0] + accurate_sample.y_axis * accRef[1] + accurate_sample.z_axis * accRef[2]);

	if(theta < 1e-5f)
	{
		R[0] = 1; R[1] = 0; R[2] = 0;
		R[3] = 0; R[4] = 1; R[5] = 0;
		R[6] = 0; R[7] = 0; R[8] = 1;
	}
	else
	{
		tmp = sinf(theta/2) / sinf(theta);

		x = (accurate_sample.y_axis * accRef[2] - accurate_sample.z_axis * accRef[1]) * tmp;
		y = (accurate_sample.z_axis * accRef[0] - accurate_sample.x_axis * accRef[2]) * tmp;
		z = (accurate_sample.x_axis * accRef[1] - accurate_sample.y_axis * accRef[0]) * tmp;
		w = cosf(theta/2);

		R[0] = 1 - 2*(y*y + z*z); R[1] =     2*(x*y - z*w); R[2] =     2*(x*z + y*w);
		R[3] =     2*(x*y + z*w); R[4] = 1 - 2*(x*x + z*z); R[5] =     2*(y*z - x*w);
		R[6] =     2*(x*z - y*w); R[7] =     2*(y*z + x*w); R[8] = 1 - 2*(x*x + y*y);
	}
}
/*
void GetAccToGroundMatrix(AccMeasureFloat_TypeDef accMeasurent, float* R)
{
	float accRef[3] = {0, 1, 0};
	float theta;
	float tmp, x, y, z, w;
	
	// Normalize x, y, z
	tmp = sqrt(SQR_F(accMeasurent.x) + SQR_F(accMeasurent.y) + SQR_F(accMeasurent.z));
	accMeasurent.x /= tmp;
	accMeasurent.y /= tmp;
	accMeasurent.z /= tmp;
	
	// Rotate acc axis to cam axis (180 degrees z-axis rotation)
	accMeasurent.x = -accMeasurent.x;
	accMeasurent.y = -accMeasurent.y;
	
	
	// Calc rotatoin matrix
	
	theta = acosf(accMeasurent.x * accRef[0] + accMeasurent.y * accRef[1] + accMeasurent.z * accRef[2]);
	
	if(theta < 1e-5f)
	{
		R[0] = 1; R[1] = 0; R[2] = 0;
		R[3] = 0; R[4] = 1; R[5] = 0;
		R[6] = 0; R[7] = 0; R[8] = 1;
	}	
	else
	{
		tmp = sinf(theta/2) / sinf(theta);
		
		x = (accMeasurent.y * accRef[2] - accMeasurent.z * accRef[1]) * tmp;
		y = (accMeasurent.z * accRef[0] - accMeasurent.x * accRef[2]) * tmp;
		z = (accMeasurent.x * accRef[1] - accMeasurent.y * accRef[0]) * tmp;
		w = cosf(theta/2);
		
		R[0] = 1 - 2*(y*y + z*z); R[1] =     2*(x*y - z*w); R[2] =     2*(x*z + y*w);
		R[3] =     2*(x*y + z*w); R[4] = 1 - 2*(x*x + z*z); R[5] =     2*(y*z - x*w);
		R[6] =     2*(x*z - y*w); R[7] =     2*(y*z + x*w); R[8] = 1 - 2*(x*x + y*y);
	}
}*/

void get_adxl345_sample(void)
{
    static SAMPLE sample_pointer;
    adxl345_get_accelerometer_sample(&sample_pointer);
    send_message_to_pc( (char *)&sample_pointer , sizeof( SAMPLE ) );
}
