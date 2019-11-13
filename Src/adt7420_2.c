/*Skip to content
Personal Open source Business Explore
Sign upSign inPricingBlogSupport
This repository
Search
 Watch 54  Star 64  Fork 389 analogdevicesinc/no-OS
 Code  Issues 7  Pull requests 1  Wiki  Pulse  Graphs
Branch: master Find file Copy pathno-OS/drivers/ADT7420/ADT7420.c
a12fcf7  on 29 May 2013
@dbogdan dbogdan PMODS: Added the DPOT, GYRO2, IA, IOXP and TMP2 demonstration software.
1 contributor
RawBlameHistory     226 lines (206 sloc)  8.41 KB*/
/***************************************************************************//**
 *   @file   ADT7420.c
 *   @brief  Implementation of ADT7420 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: $WCREV$
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <adt7420_2.h>
#include <stm32l4xx.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "i2c.h"

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
unsigned char resolutionSetting = 1;    // Current resolution setting
uint8_t adt7420_temperature_msb;
uint8_t adt7420_temperature_lsb;

/***************************************************************************//**
 * @brief config ADT7420.
 *
 * @param registerAddress - Address of the register.
 *
 * @return registerValue  - Value of the register.
*******************************************************************************/
float ADT7420_Config (void)
{
	
float TEMP4720=0;

void ADT7420_Reset(void);
	
ADT7420_SetRegisterValue(ADT7420_REG_T_HIGH_MSB,0x00);//define setpoint high TEMP.
	
ADT7420_SetRegisterValue(ADT7420_REG_T_HIGH_LSB,0x78);

ADT7420_SetOperationMode(ADT7420_OP_MODE_ONE_SHOT);//one shot mode
	
ADT7420_SetResolution(resolutionSetting);//0-13bit,1-16bit

TEMP4720 = ADT7420_GetTemperature();//reading temp

return TEMP4720;

}





/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 *
 * @return registerValue  - Value of the register.
*******************************************************************************/
unsigned char ADT7420_GetRegisterValue(unsigned char registerAddress)
{
     unsigned char registerValue = 0x00;
	
//u8 ReadByte(I2C_TypeDef* I2Cx , u8 slaveAddress , u8 subAdd)	 
	registerValue = I2C2_read_byte( ADT7420_ADDRESS , registerAddress);

	// I2C_Read(ADT7420_ADDRESS,&registerAddress,1,0);
	
  // I2C_Read(ADT7420_ADDRESS,&registerValue,1,1);
	
  // u8 WriteByte(I2C_TypeDef* I2Cx , u8 slaveAddress , u8 subAdd , u8 byte)

	return registerValue;
}

/***************************************************************************//**
 * @brief Sets the value of a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue   - Value of the register.
 *
 * @return None.
*******************************************************************************/
void ADT7420_SetRegisterValue(unsigned char registerAddress,
                              unsigned char registerValue)
{
    unsigned char dataBuffer[2] = {0, 0};
		
		dataBuffer[0] = registerAddress;
    
		dataBuffer[1] = registerValue;
    
		WriteArray(I2C2 , ADT7420_ADDRESS , registerAddress , dataBuffer , 2);
		
		
//    
//		
//	
//    I2C_Write(ADT7420_ADDRESS,dataBuffer,2,1);
		
}

/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the device is
 *        present.
 *
 * @return status - The result of the initialization procedure.
 *                  Example: -1 - I2C peripheral was not initialized or the
 *                                device is not present.
 *                            0 - I2C peripheral was initialized and the
 *                                device is present.
*******************************************************************************/
char ADT7420_Init(void)
{
    unsigned char status = 1;
    uint8_t test   = 0;
    
  //status = I2C_Init(1000000000);
	
	test   = ADT7420_GetRegisterValue(ADT7420_REG_ID);
  
  if(test != ADT7420_DEFAULT_ID)
    {
        status = 2;
    }
    
    return status;
}

/***************************************************************************//**
 * @brief Resets the ADT7420.
 *        The ADT7420 does not respond to I2C bus commands while the default
 *        values upload (approximately 200 us).
 *
 * @return None.
*******************************************************************************/
void ADT7420_Reset(void)
{
    unsigned char registerAddress = ADT7420_REG_RESET;
    
	
	WriteByte(I2C2 , ADT7420_ADDRESS ,registerAddress  , 0x020);
	
    resolutionSetting = 1;
}

/***************************************************************************//**
 * @brief Sets the operational mode for ADT7420.
 *
 * @param mode - Operation mode.
 *               Example: ADT7420_OP_MODE_CONT_CONV - continuous conversion;
 *                        ADT7420_OP_MODE_ONE_SHOT  - one shot;
 *                        ADT7420_OP_MODE_1_SPS     - 1 SPS mode;
 *                        ADT7420_OP_MODE_SHUTDOWN  - shutdown.
 *
 * @return None.
*******************************************************************************/
void ADT7420_SetOperationMode(unsigned char mode)
{
    unsigned char registerValue = 0;

    registerValue  = ADT7420_GetRegisterValue(ADT7420_REG_CONFIG);
    registerValue &= ~ADT7420_CONFIG_OP_MODE(ADT7420_OP_MODE_SHUTDOWN);
    registerValue |= ADT7420_CONFIG_OP_MODE(mode);
    ADT7420_SetRegisterValue(ADT7420_REG_CONFIG, registerValue);
}

/***************************************************************************//**
 * @brief Sets the resolution for ADT7420.
 *
 * @param resolution - Resolution.
 *                     Example: 0 - 13-bit resolution;
 *                              1 - 16-bit resolution.
 *
 * @return None.
*******************************************************************************/
void ADT7420_SetResolution(unsigned char resolution)
{
    unsigned char registerValue = 0;

    registerValue  = ADT7420_GetRegisterValue(ADT7420_REG_CONFIG);
    registerValue &= ~ADT7420_CONFIG_RESOLUTION;
    registerValue |= (resolution * ADT7420_CONFIG_RESOLUTION);
    ADT7420_SetRegisterValue(ADT7420_REG_CONFIG, registerValue);
    resolutionSetting = resolution;
}

/***************************************************************************//**
 * @brief Reads the temperature data and converts it to Celsius degrees.
 *
 * @return temperature - Temperature in degrees Celsius.
*******************************************************************************/
float ADT7420_GetTemperature(void)
{
    unsigned char  msbTemp = 0;
    unsigned char  lsbTemp = 0;
    unsigned short temp    = 0;
    float          tempC   = 0;

    msbTemp = ADT7420_GetRegisterValue(ADT7420_REG_TEMP_MSB);
    lsbTemp = ADT7420_GetRegisterValue(ADT7420_REG_TEMP_LSB);
    temp    = ((unsigned short)msbTemp << 8) + lsbTemp;
    if(resolutionSetting)
    {
        if(temp & 0x8000)
        {
            /*! Negative temperature */
            tempC = (float)((signed long)temp - 65536) / 128;
        }
        else
        {
            /*! Positive temperature */
            tempC = (float)temp / 128;
        }
    }
    else
    {
        temp >>= 3;
        if(temp & 0x1000)
        {
            /*! Negative temperature */
            tempC = (float)((signed long)temp - 8192) / 16;
        }
        else
        {
            /*! Positive temperature */
            tempC = (float)temp / 16;
        }
    }

    return tempC;
}

int8_t ADT7420_GetTemperatureBitsMSB(void)
{
		return adt7420_temperature_msb = ADT7420_GetRegisterValue(ADT7420_REG_TEMP_MSB);
}

int8_t ADT7420_GetTemperatureBitsLSB(void)
{
		return adt7420_temperature_lsb = ADT7420_GetRegisterValue(ADT7420_REG_TEMP_LSB);
}


int8_t ADT7420_GetStoredTemperatureBitsMSB(void)
{
		return adt7420_temperature_msb;
}

int8_t ADT7420_GetStoredTemperatureBitsLSB(void)
{
		return adt7420_temperature_lsb;
}

//Status API Training Shop Blog About
//© 2016 GitHub, Inc. Terms Privacy Security Contact Help




