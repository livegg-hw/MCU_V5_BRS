/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : commands.c                                                                                     *
 *  Abstract   : Commands communicated with PC.                                                                 *
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : AUGOST 2017                                                                                    *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "stm32l4xx_hal.h"
#include "atwinc1500.h"
#include "sensors.h"
#include "sht35.h"
#include "adt7420.h"
#include "adxl345.h"
#include "lis3dshtr.h"
#include "leds.h"
#include "battery.h"
#include "charger.h"
#include "photo_diode.h"
#include "tray_id.h"

/*-------------------------------------------------------------------------------------------*/
/*----------------------------- DEFINITIONS AND ENUMARTIONS ---------------------------------*/
/*-------------------------------------------------------------------------------------------*/
typedef void  (*COMMAND_FUNCTION_VOID)(void);
typedef void  (*COMMAND_FUNCTION_BYTE)(uint8_t);
typedef void  (*COMMAND_FUNCTION_WORD)(uint16_t);
typedef void  (*COMMAND_FUNCTION_INT)(int);
typedef void  (*COMMAND_FUNCTION_POINTER)(char *);

typedef enum
{
    VOID,
    CHAR,
    BYTE,
    WORD,
    INT,
    POINTER
} DATA_TYPE;

typedef struct
{
    char      *command_string;
    DATA_TYPE  data_type;
    void      *command_function_pointer;
} COMMAND;

#define UART_TIMEOUT                     3000
#define DEVICE_ID_REGISTER_ADDRESS       0x1FF0F420

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
void comm_liveness_response(void);
void get_mcu_device_id(void);


static COMMAND commands[] =
/*******************************************************************************************/
{// "COMMAND STRING"        , DATA    , FUNCTION POINTER                                  } ,
  //                        , TYPE    ,                                                   } ,
/*******************************************************************************************/
  { "ADT_SAMPLE"            , VOID    , (void *)(get_adt7420_sample)                      } ,
  { "ADXL_SAMPLE"           , VOID    , (void *)(get_adxl345_sample)                      } ,
  { "COMM_ALIVE"            , VOID    , (void *)(comm_liveness_response)                  } ,
  { "CHRG_CTRL"             , CHAR    , (void *)(charger_set_charge_control)              } ,
  { "DEVICE_ID"             , VOID    , (void *)(get_mcu_device_id)                       } ,
  { "LED_CRG"               , INT     , (void *)(led_charge_configuration)                } ,
  { "LED_IND"               , INT     , (void *)(led_indication_configuration)            } ,
  { "GET_BATT"              , VOID    , (void *)(battery_get_battery_measurement)         } ,
  { "GET_CHRG"              , VOID    , (void *)(charger_get_setting)                     } ,
  { "GET_ISET"              , VOID    , (void *)(charger_get_iset_analog_measurement)     } ,
  { "GET_P2"                , VOID    , (void *)(photo_diode_get_sampling_parameters)     } ,
  { "GET_PORT"              , VOID    , (void *)(get_wifi_port_number)                    } ,
  { "GET_SMPL"              , INT     , (void *)(photo_diode_get_samples)                 } ,
  { "GET_TRAY"              , VOID    , (void *)(get_tray_id)                 			  } ,
  { "SET_CHRG"              , BYTE    , (void *)(charger_set_setting)                     } ,
 // { "SET_PORT"              , WORD    , (void *)(set_wifi_port_number)                    } ,
  { "SET_TRAY"              , POINTER , (void *)(set_tray_id)                    		  } ,
  { "LIS_SAMPLE"            , VOID    , (void *)(get_lis3dshtr_sample)                    } ,
  { "SHT_HUMIDITY"          , VOID    , (void *)(sht35_get_humidity_sample)            	  } ,
  { "SHT_SAMPLE"            , VOID    , (void *)(sht35_get_sample)                   	  } ,
  { "SHT_TEMPERATURE"       , VOID    , (void *)(sht35_get_temperature_sample)         	  } ,
  
  { "SCD30 "                , POINTER , (void *)(extCo2BoardMsgHandle)                    } 
};

    uint32_t parameter;
    uint8_t  byte_parameter;
    uint16_t word_parameter;
    int      int_parameter;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void send_message_to_pc( char *message , uint16_t message_size )
{
    HAL_UART_Transmit( &huart1 , (uint8_t *)message , message_size  , UART_TIMEOUT );
}

void command_parsing( uint8_t * input_command )
{
    volatile uint8_t command_index;
    //uint8_t  byte_parameter;
    //uint16_t word_parameter;
    //uint32_t int_parameter;
    //uint32_t parameter;
    char nack[] = "NACK\n";
    char command[20];

    /*if( get_receiving_status() == DURING_FILE_RECEIVE )
    {
        xmodem_block_handler(input_command);
    }
    else*/
    {
        for( command_index = 0 ; command_index < sizeof( commands ) / sizeof( COMMAND ) ; command_index++ )
        {
            if( strncmp( (const char *)commands[command_index].command_string , (const char *)input_command , strlen( commands[command_index].command_string ) ) == 0 )
           // || strstr( (const char *)commands[command_index].command_string , (const char *)input_command ) == 0 )
            {
                switch( commands[command_index].data_type )
                {
                    case VOID:  (*(COMMAND_FUNCTION_VOID)commands[command_index].command_function_pointer)();
                                break;

                    case CHAR:  parameter = *( strchr( (char const *)input_command , ' ' ) + 1 );
                                (*(COMMAND_FUNCTION_BYTE)commands[command_index].command_function_pointer)(parameter);
                                break;

                    case BYTE:  byte_parameter = *( strchr( (char const *)input_command , ' ' ) + 1 );
                                (*(COMMAND_FUNCTION_BYTE)commands[command_index].command_function_pointer)(byte_parameter);
                                break;

                    case WORD:  //word_parameter = *( (uint16_t *)( strchr( (char const *)input_command , ' ' ) + 1 ) );
                    			//(*(COMMAND_FUNCTION_WORD)commands[command_index].command_function_pointer)(word_parameter);
                    			//word_parameter = *( (uint16_t *)( strchr( (char const *)input_command , ' ' ) + 1 ) );
                    			sscanf( (char *)input_command , "%s %d" , command , &int_parameter );
                                (*(COMMAND_FUNCTION_WORD)commands[command_index].command_function_pointer)(int_parameter);
                                break;

                    case INT:   int_parameter = *( (uint32_t *)( strchr( (char const *)input_command , ' ' ) + 1 ) );
                    			sscanf( ( strchr( (char const *)input_command , ' ' ) + 1 ) , "%d" , &int_parameter );
                                (*(COMMAND_FUNCTION_INT)commands[command_index].command_function_pointer)(int_parameter);
                                break;

                    case POINTER:(*(COMMAND_FUNCTION_POINTER)commands[command_index].command_function_pointer)(strchr( (char *)input_command , ' ' ) + 1 );
                                break;

                    default:    break;
                }
                return;
            }
        }
        send_message_to_pc( nack , strlen( nack ) );
    }
}

void comm_liveness_response(void)
{
    char livness_response[] = "AA\n";
    send_message_to_pc( (char *)livness_response , strlen( livness_response ) );
}

void get_mcu_device_id(void)
{
    char devide_id_string[25];
    unsigned int *device_id_address = (unsigned int *)DEVICE_ID_REGISTER_ADDRESS;

    //sprintf( devide_id_string ,"%X%X%X\n" , ( (uint32_t *)( DEVICE_ID_REGISTER_ADDRESS + 8 ) , ( (uint32_t *)( DEVICE_ID_REGISTER_ADDRESS + 4 ) ) , ( (uint32_t *)DEVICE_ID_REGISTER_ADDRESS ) ) );
    sprintf( devide_id_string ,"%08X%08X%08X\n" , *( device_id_address + 2 ) , *( device_id_address + 1 ) , *device_id_address );
    send_message_to_pc( devide_id_string , strlen( devide_id_string ) );
}
