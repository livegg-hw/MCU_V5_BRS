/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : lis3dshtr.c                                                                                    *
 *  Abstract   : Digital output accelerometer sensor with I2C Interface.                                        *
 *							 http://pdf.datasheetcatalog.com/datasheets/stmicroelectronics/LIS3DSH_LIS3DSHTR.pdf*
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : APRIL 2018                                                                                     *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "stm32l4xx_hal.h"
#include "i2c.h"
#include "sensors.h"
#include "lis3dshtr.h"
#include "command.h"
   
/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define LIS3DSHTR_SLAVE_ADDRESS       						0x3A
#define LIS3MDL_DEVICE_IDENTIFICATION 						0x3F
#define REGISTER_ADDRESS_AUTOMATICALLY_INCREMENTED_ENABLE 	0x10

typedef enum
{
	INFO1 	  = 0x0D,
	INFO2 	  = 0x0E,
	WHO_AM_I  = 0x0F,
	CTRL_REG3 = 0x23,
	CTRL_REG4 = 0x20,
	CTRL_REG5 = 0x24,
	CTRL_REG6 = 0x25,
	STATUS    = 0x27,
	OUT_T     = 0x0C,
	OFFSET_X  = 0x10,
	OFFSET_Y  = 0x11,
	OFFSET_Z  = 0x12,
	CS_X	  = 0x13,
	CS_Y      = 0x14,
	CS_Z	  = 0x15,
	LC_L	  = 0x16,
	LC_H	  = 0x17,
	STAT	  = 0x18,
	VFC_1	  = 0x1B,
	VFC_2	  = 0x1C,
	VFC_3	  = 0x1D,
	VFC_4	  = 0x1E,
	THRS3	  = 0x1F,
	OUT_X     = 0x28,
	OUT_X_L   = 0x28,
	OUT_X_H   = 0x29,
	OUT_Y     = 0x2A,
	OUT_Y_L   = 0x2A,
	OUT_Y_H   = 0x2B,
	OUT_Z     = 0x2C,
	OUT_Z_L   = 0x2C,
	OUT_Z_H   = 0x2D,
	FIFO_CTRL = 0x2E,
	FIFO_SRC  = 0x2F,
	CTRL_REG1 = 0x21,
	ST1_1     = 0x40,
	ST1_2     = 0x41,
	ST1_3     = 0x42,
	ST1_4     = 0x43,
	ST1_5     = 0x44,
	ST1_6     = 0x45,
	ST1_7     = 0x46,
	ST1_8     = 0x47,
	ST1_9     = 0x48,
	ST1_10    = 0x49,
	ST1_11    = 0x4A,
	ST1_12    = 0x4B,
	ST1_13    = 0x4C,
	ST1_14    = 0x4D,
	ST1_15    = 0x4E,
	ST1_16    = 0x4F,
	TIM4_1    = 0x50,
	TIM3_1    = 0x51,
	TIM2_1    = 0x52,
	TIM1_1    = 0x54,
	THRS2_1   = 0x56,
	THRS1_1   = 0x57,
	MASK1_B   = 0x59,
	MASK1_A   = 0x5A,
	SETT1     = 0x5B,
	PR1		  = 0x5C,
	TC1	      = 0x5D,
	OUTS1     = 0x5F,
	PEAK1     = 0x19,
	CTRL_REG2 = 0x22,
	ST2_1     = 0x60,
	ST2_2     = 0x61,
	ST2_3     = 0x62,
	ST2_4     = 0x63,
	ST2_5     = 0x64,
	ST2_6     = 0x65,
	ST2_7     = 0x66,
	ST2_8     = 0x67,
	ST2_9     = 0x68,
	ST2_10    = 0x69,
	ST2_11    = 0x6A,
	ST2_12    = 0x6B,
	ST2_13    = 0x6C,
	ST2_14    = 0x6D,
	ST2_15    = 0x6E,
	ST2_16    = 0x6F,
	TIM4_2	  = 0x70,
	TIM3_2	  = 0x71,
	TIM2_2	  = 0x72,
	TIM1_2	  = 0x74,
	THRS2_2	  = 0x76,
	THRS1_2	  = 0x77,
	MASK2_B	  = 0x79,
	MASK2_A	  = 0x7A,
	SETT2	  = 0x7B,
	PR2		  = 0x7C,
	TC2		  = 0x7D,
	OUTS2	  = 0x7F,
	PEAK2	  = 0x1A,
	DES2	  = 0x78
} LIS3DSHTR_REGISTER;

typedef enum
{
	POWER_DOWN = 0x00,
 	ODR_3_125  = 0x10,
	ODR_6_25   = 0x20,
	ODR_12_5   = 0x30,
	ODR_25	   = 0x40,
	ODR_50	   = 0x50,
	ODR_100	   = 0x60,
	ODR_400	   = 0x70,
	ODR_800	   = 0x80,
	ODR_1600   = 0x90
} OUTPUT_DATA_RATE;

typedef enum
{
	X_AXIS_ENABLE = 0x01,
	Y_AXIS_ENABLE = 0x02,
	Z_AXIS_ENABLE = 0x04,
} AXIS_ENABLE;

typedef enum
{
	FULL_SCALE_2G  = 0x00,
	FULL_SCALE_4G  = 0x08,
	FULL_SCALE_6G  = 0x10,
	FULL_SCALE_8G  = 0x18,
	FULL_SCALE_16G = 0x20
} FULL_SCALE;

typedef enum
{
	NORMAL_MODE 			= 0x00,
	POSITIVE_SIGN_SELF_TEST = 0x02,
	NEGATIVE_SIGN_SELF_TEST = 0x04
} SELF_TEST_MODE;

typedef enum
{
	BANDWIDTH_800 = 0x00,
	BANDWIDTH_400 = 0x40,
	BANDWIDTH_200 = 0x80,
	BANDWIDTH_50  = 0xC0
} ANTI_ALIASING_FILTER_BANDWIDTH;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
uint8_t ver,info1,info2;
int16_t x,y,z;
//int16_t LIS3DSHTR_get_accelerometer_axis_x();
//int16_t LIS3DSHTR_get_accelerometer_axis_y();
//int16_t LIS3DSHTR_get_accelerometer_axis_z();
SAMPLE sample_pointer;
void lis3dshtr_get_accelerometer_sample(SAMPLE *sample_pointer);

void LIS3DSHTR_initialization(void)
{
 //   while( ReadByte( I2C2 , LIS3DSHTR_SLAVE_ADDRESS , WHO_AM_I ) != LIS3MDL_DEVICE_IDENTIFICATION );
	info1 = I2C2_read_byte( LIS3DSHTR_SLAVE_ADDRESS , INFO1 );
	info2 = I2C2_read_byte( LIS3DSHTR_SLAVE_ADDRESS , INFO2 );
	ver   = I2C2_read_byte( LIS3DSHTR_SLAVE_ADDRESS , WHO_AM_I );
	I2C2_write_byte( LIS3DSHTR_SLAVE_ADDRESS , CTRL_REG1 , 0x00 );
    I2C2_write_byte( LIS3DSHTR_SLAVE_ADDRESS , CTRL_REG2 , 0x00 );
    I2C2_write_byte( LIS3DSHTR_SLAVE_ADDRESS , CTRL_REG3 , 0x00 );
    I2C2_write_byte( LIS3DSHTR_SLAVE_ADDRESS , CTRL_REG4 , X_AXIS_ENABLE | Y_AXIS_ENABLE | Z_AXIS_ENABLE | ODR_100 );
    I2C2_write_byte( LIS3DSHTR_SLAVE_ADDRESS , CTRL_REG5 , FULL_SCALE_2G | NORMAL_MODE | BANDWIDTH_200 );
    I2C2_write_byte( LIS3DSHTR_SLAVE_ADDRESS , CTRL_REG6 , REGISTER_ADDRESS_AUTOMATICALLY_INCREMENTED_ENABLE );
    I2C2_write_byte( LIS3DSHTR_SLAVE_ADDRESS , FIFO_CTRL , 0x00 );
    /*while(1)
    {
    	lis3dshtr_get_accelerometer_sample(&sample_pointer);
    	lis3dshtr_get_accelerometer_sample(&sample_pointer);
    //x = LIS3DSHTR_get_accelerometer_axis_x();
    //y = LIS3DSHTR_get_accelerometer_axis_y();
    //z = LIS3DSHTR_get_accelerometer_axis_z();
    }*/
}

uint16_t lis3dshtr_get_accelerometer_axis_sample(ACCELEROMETER_AXIS accelerometer_axis)
{
	int16_t accelerometer_sample = I2C2_read_word( LIS3DSHTR_SLAVE_ADDRESS , OUT_X + (LIS3DSHTR_REGISTER)accelerometer_axis * 2 );
//		ComUART1_Printf("in I2C: %d \r\n", accelerometer_sample);
//		ComUART1_Printf("in I2C+32768: %d \r\n", accelerometer_sample+32768);

	//return ( ( accelerometer_sample & 0x0800 ) ? ( accelerometer_sample + 0xF000 ) : accelerometer_sample );
	return accelerometer_sample + 32768;
	
}

int16_t LIS3DSHTR_get_accelerometer_axis_x()
{
	return lis3dshtr_get_accelerometer_axis_sample(ACCELEROMETER_AXIS_X);
}

int16_t LIS3DSHTR_get_accelerometer_axis_y()
{
	return lis3dshtr_get_accelerometer_axis_sample(ACCELEROMETER_AXIS_Y);
}

int16_t LIS3DSHTR_get_accelerometer_axis_z()
{
	return lis3dshtr_get_accelerometer_axis_sample(ACCELEROMETER_AXIS_Z);
}

void lis3dshtr_get_accelerometer_sample(SAMPLE *sample_pointer)
{
	I2C2_read_array( LIS3DSHTR_SLAVE_ADDRESS , OUT_X , 6 , (uint8_t *)sample_pointer );
}

int8_t LIS3DSHTR_get_temperature()
{
	return I2C2_read_byte( LIS3DSHTR_SLAVE_ADDRESS , OUT_T );
}

void get_lis3dshtr_sample(void)
{
    SAMPLE sample_pointer;
    lis3dshtr_get_accelerometer_sample(&sample_pointer);
    send_message_to_pc( (char *)&sample_pointer , sizeof( SAMPLE ) );
}
