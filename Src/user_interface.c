/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : PANACHE                                                                                        *
 *  File name  : user_interface.c                                                                               *
 *  Abstract   : This module uses as user interface with the GUI.                                               *
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : MARCH 2018                                                                                     *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "stm32l4xx_hal.h"
#include "user_interface.h"
#include "command.h"

/*-------------------------------------------------------------------------------------------*/
/*----------------------------- DEFINITIONS AND ENUMARTIONS ---------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define RECIEVED_DATA_BUFFER_SIZE       512

typedef struct
{
    uint8_t sector_erased;
    uint8_t sector_written;
    uint8_t sector_ok;
} SECTOR_STATUS;

typedef struct
{
    uint32_t sector_first_address;
    uint32_t sector_last_address;
} SECTOR_ADDRESS_BOUNDARY;

typedef struct
{
    uint8_t  test_number;
    uint32_t adress;
    uint32_t expected_data;
    uint32_t actual_data;
} FLASH_ERROR_DETAILS;

typedef struct
{
    uint8_t zeros_test   : 1;
    uint8_t ones_test    : 1;
    uint8_t Ox55_test    : 1;
    uint8_t OxAA_test    : 1;
    uint8_t address_test : 1;
} TESTS_BITS;

typedef union
{
    TESTS_BITS tests_bits;
    uint8_t    tests;
} TESTS;

typedef struct
{
    uint32_t time_of_test_beginning;
    uint32_t time_of_start_erasing;
    uint32_t time_of_stop_erasing;
    uint32_t time_of_start_writing;
    uint32_t time_of_stop_writing;
    uint32_t time_of_start_reading;
    uint32_t time_of_stop_reading;
    uint32_t time_of_test_ending;
} FLASH_TEST_TIMING;

typedef enum
{
    SECTORS_ERASING,
    SECTORS_BURNING,
    SECTORS_CHECKING
} SECTORS_STATE;

#define MAXIMUM_FLASH_ERRORS    100
#define NUMBER_OF_EFLASH_TESTS  5

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
uint8_t recieved_data_buffer[RECIEVED_DATA_BUFFER_SIZE];
uint8_t character;
uint16_t recieved_character_index = 0;
bool    new_command_is_waiting = false;
RECEIVING_STATUS receiving_status = NORMAL;


/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void uart1_initialization()
{
    HAL_UART_Receive_IT( &huart1, (uint8_t *)&character , 1 );
}

void user_interface_recieved_polling()
{
    if( new_command_is_waiting == true )
    {
        new_command_is_waiting = false;
        command_parsing(recieved_data_buffer);
    }
}

//#pragma optimize=none
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 char *ch;
   if (huart->Instance == USART1) {
       // switch(receiving_status)
     //   {
          //  case NORMAL:              
                switch(character)
                {
                    case '\r':
                    case '\n':  
                        if( recieved_character_index == 0 ) {
                          new_command_is_waiting = false;
                        } else {
                          new_command_is_waiting = true;
                          recieved_data_buffer[recieved_character_index++] = '\0';
                          LogPrintfWrapperC(FONT_LIGHT_CYAN, "Rec buffer:%s\r\n", recieved_data_buffer);
                          recieved_character_index = 0;
                         // user_interface_recieved_polling();
                          ch = strstr(  (const char *)recieved_data_buffer, (const char *)"SCD30 ") ;
                          if(NULL != ch ) {
                               new_command_is_waiting = false;
                               extCo2BoardMsgHandle(&ch[6]);
                          }
                        }
                        break;
                              
                    default:    
                         recieved_data_buffer[recieved_character_index++] = character;
                         if( recieved_character_index >= RECIEVED_DATA_BUFFER_SIZE ) {
                             recieved_character_index = 0;
                         }
                }
            //    break;
                                      
//            case DURING_FILE_RECEIVE: 
//                recieved_data_buffer[recieved_character_index++] = character;
//                if( recieved_character_index >= 132 )
//                {
//                  new_command_is_waiting = true;
//                  recieved_character_index = 0;
//                }
//                break;        
//          
//            case DURING_FILE_SEND:    
//                recieved_data_buffer[recieved_character_index++] = character;
//                if( recieved_character_index >= 9 )
//                {
//                  new_command_is_waiting = true;
//                  recieved_character_index = 0;
//                }
//                break; 
 //       }
        HAL_UART_Receive_IT( &huart1, &character , 1 );
    }
}

/**
  * @brief  This function handles UART interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA stream 
  *         used for USART data transmission     
  */
void USART1_IRQHandler(void)
{
  char *ch;
    HAL_UART_IRQHandler(&huart1);
////    HAL_UART_Receive_IT( &huart1, &character , 1 );
 ////   huart1.Instance->CR1 |= ( USART_CR1_PEIE | USART_CR1_RXNEIE );
    //SET_BIT(&huart1->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);

}

//#pragma optimize=high z
