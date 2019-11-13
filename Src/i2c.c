/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : i2c.c                                                                                          *
 *  Abstract   : i2c driver.                                                                                    *
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : APRIL 2018                                                                                     *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "i2c.h"

/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define TIMEOUT                         2000
#define MAXIMUM_TRANSFERRING_BYTES      100

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c4;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void I2C_write_byte( I2C_HandleTypeDef *hi2c , uint16_t DevAddress , uint8_t command , uint8_t value )
{
    HAL_I2C_Master_Transmit( hi2c , DevAddress , (uint8_t [2]){ command  , value } , 2 , TIMEOUT ); // Transmit register address to the device ...
}

void I2C2_write_byte( uint16_t DevAddress , uint8_t command , uint8_t value )
{
    I2C_write_byte( &hi2c2 , DevAddress , command , value );
}

void I2C4_write_byte( uint16_t DevAddress , uint8_t command , uint8_t value )
{
    I2C_write_byte( &hi2c4 , DevAddress , command , value );
}

uint8_t I2C_read_byte( I2C_HandleTypeDef *hi2c , uint16_t DevAddress , uint8_t command )
{
    uint8_t result[1];
    HAL_I2C_Master_Transmit( hi2c , DevAddress , (uint8_t [1]){ command } , 1 , TIMEOUT ); // Transmit register address to the device ...
    HAL_I2C_Master_Receive(  hi2c , DevAddress , &result[0] , 1 , TIMEOUT );  // ... and read 1 byte (the register content).
    return result[0];
}

uint8_t I2C2_read_byte( uint16_t DevAddress , uint8_t command )
{
    return I2C_read_byte( &hi2c2 , DevAddress , command );
}

uint8_t I2C4_read_byte( uint16_t DevAddress , uint8_t command )
{
    return I2C_read_byte( &hi2c4 , DevAddress , command );
}

void I2C_write_word( I2C_HandleTypeDef *hi2c , uint16_t DevAddress , uint8_t command , uint16_t value )
{
    HAL_I2C_Master_Transmit( hi2c , DevAddress , (uint8_t [3]){ command  , ( value >> 8 ) , ( value & 0xFF ) } , 2 , TIMEOUT ); // Transmit register address to the device ...
}

void I2C2_write_word( uint16_t DevAddress , uint8_t command , uint16_t value )
{
    I2C_write_word( &hi2c2 , DevAddress , command , value );
}

void I2C4_write_word( uint16_t DevAddress , uint8_t command , uint16_t value )
{
    I2C_write_word( &hi2c4 , DevAddress , command , value );
}

uint16_t I2C_read_word( I2C_HandleTypeDef *hi2c , uint16_t DevAddress , uint8_t command )
{
    uint8_t result[2];
    HAL_I2C_Master_Transmit( hi2c , DevAddress , (uint8_t [1]){ command } , 1 , TIMEOUT ); // Transmit register address to the device ...
    HAL_I2C_Master_Receive(  hi2c , DevAddress , &result[0] , 2 , TIMEOUT );  // ... and read 1 byte (the register content).
    return( ( result[1] << 8 ) | result[0] );
//    return( *( (uint16_t *)result ) );
//    return __REV16( *(uint16_t *)result );
}

uint16_t I2C2_read_word( uint16_t DevAddress , uint8_t command )
{
    return I2C_read_word( &hi2c2 , DevAddress , command );
}

uint16_t I2C4_read_word( uint16_t DevAddress , uint8_t command )
{
    return I2C_read_word( &hi2c4 , DevAddress , command );
}

void I2C_write_bytes( I2C_HandleTypeDef *hi2c , uint16_t DevAddress , uint8_t command , uint8_t *data , uint8_t data_size )
{
    uint8_t transmitted_data[MAXIMUM_TRANSFERRING_BYTES];
    transmitted_data[0] = command;    
    for( uint16_t data_index = 0 ; data_index < data_size ; data_index++ )
        transmitted_data[ data_index + 1 ] = data[data_index];
    HAL_I2C_Master_Transmit( hi2c , DevAddress , transmitted_data , data_size + 1 , TIMEOUT ); // Transmit register address to the device ...
}

void I2C2_write_bytes( uint16_t DevAddress , uint8_t command , uint8_t *data , uint8_t data_size )
{
    I2C_write_bytes( &hi2c2 , DevAddress , command , data , data_size );
}


void I2C4_write_bytes( uint16_t DevAddress , uint8_t command , uint8_t *data , uint8_t data_size )
{
    I2C_write_bytes( &hi2c4 , DevAddress , command , data , data_size );
}

uint8_t * I2C_read_bytes( I2C_HandleTypeDef *hi2c , uint16_t DevAddress , uint8_t command , uint8_t data_size )
{
    static uint8_t result[MAXIMUM_TRANSFERRING_BYTES];
    HAL_I2C_Master_Transmit( hi2c , DevAddress , (uint8_t [1]){ command } , 1 , TIMEOUT ); // Transmit register address to the device ...
    HAL_I2C_Master_Receive(  hi2c , DevAddress , &result[0] , data_size , TIMEOUT );  // ... and read 1 byte (the register content).
    return (uint8_t *)result;
}

uint8_t * I2C2_read_bytes( uint16_t DevAddress , uint8_t command , uint8_t data_size )
{
    return I2C_read_bytes( &hi2c2 , DevAddress , command , data_size );
}

uint8_t * I2C4_read_bytes( uint16_t DevAddress , uint8_t command , uint8_t data_size )
{
    return I2C_read_bytes( &hi2c4 , DevAddress , command , data_size );
}

void I2C_read_array( I2C_HandleTypeDef *hi2c , uint16_t DevAddress , uint8_t command , uint8_t data_size , uint8_t *result)
{
    HAL_I2C_Master_Transmit( hi2c , DevAddress , (uint8_t [1]){ command } , 1 , TIMEOUT ); // Transmit register address to the device ...
    HAL_I2C_Master_Receive(  hi2c , DevAddress , &result[0] , data_size , TIMEOUT );  // ... and read 1 byte (the register content).
}

void I2C2_read_array( uint16_t DevAddress , uint8_t command , uint8_t data_size , uint8_t *result )
{
	I2C_read_array( &hi2c2 , DevAddress , command , data_size , result );
}

void I2C4_read_array( uint16_t DevAddress , uint8_t command , uint8_t data_size , uint8_t *result)
{
	I2C_read_array( &hi2c4 , DevAddress , command , data_size , result );
}

/*---------------------------------------------------------------------------------------------*/
/* THE SHT35-DIS TEMPERATURE SENSOR HAS UNIQUE I2C PROTOCOL, SO A SMALL MODIFICATION IS NEEDED */
/*---------------------------------------------------------------------------------------------*/
void SENSIRION_I2C_read_array( uint16_t DevAddress , uint8_t *command , uint8_t *result , uint8_t result_size )
{
    HAL_I2C_Master_Transmit( &hi2c2 , DevAddress , command , 2 , TIMEOUT );
    HAL_I2C_Master_Receive(  &hi2c2 , DevAddress , &result[0] , result_size , TIMEOUT );
}

uint16_t SENSIRION_I2C_read_word( uint16_t DevAddress , uint8_t *command )
{
    uint8_t result[3];
    HAL_I2C_Master_Transmit( &hi2c2 , DevAddress , command , 2 , TIMEOUT ); // Transmit register address to the device ...
    HAL_I2C_Master_Receive(  &hi2c2 , DevAddress , &result[0] , 3 , TIMEOUT );  // ... and read 1 byte (the register content).
    return( ( result[0] << 8 ) | result[1] );
//    return( *( (uint16_t *)result ) );
//    return __REV16( *(uint16_t *)result );
}

void SENSIRION_I2C_write_general_call_reset( uint8_t *command )
{
    HAL_I2C_Master_Transmit( &hi2c2 , command[0] , &command[1] , 1 , TIMEOUT );
}

