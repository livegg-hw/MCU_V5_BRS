/**
 *
 * \file
 *
 * \brief This module contains NMC1000 bus wrapper APIs implementation.
 *
 * Copyright (c) 2016-2017 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <stdio.h>
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include "stm32l4xx_hal.h"
//#include "asf.h"
//#include "conf_winc.h"

#define NM_BUS_MAX_TRX_SZ	  256
#define SPI_TIMEOUT			  1000
#define SPI_NSS_PIN_HIGH	  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_12, GPIO_PIN_SET   );
#define SPI_NSS_PIN_LOW		  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_12, GPIO_PIN_RESET );

extern SPI_HandleTypeDef hspi2;
void MX_SPI2_Init(void);

tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

#ifdef CONF_WINC_USE_I2C

struct i2c_master_module i2c_master_instance;
#define SLAVE_ADDRESS 0x60

/** Number of times to try to send packet if failed. */
#define I2C_TIMEOUT 100

static sint8 nm_i2c_write(uint8 *b, uint16 sz)
{
	sint8 result = M2M_SUCCESS;
	uint16_t timeout = 0;

	struct i2c_master_packet packet = {
		.address     = SLAVE_ADDRESS,
		.data_length = sz,
		.data        = b,
	};

	/* Write buffer to slave until success. */
	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == I2C_TIMEOUT) {
			break;
		}
	}

	return result;
}

static sint8 nm_i2c_read(uint8 *rb, uint16 sz)
{
	uint16_t timeout = 0;
	sint8 result = M2M_SUCCESS;
	struct i2c_master_packet packet = {
		.address     = SLAVE_ADDRESS,
		.data_length = sz,
		.data        = rb,
	};

	/* Write buffer to slave until success. */
	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) != STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == I2C_TIMEOUT) {
			break;
		}
	}

	return result;
}

static sint8 nm_i2c_write_special(uint8 *wb1, uint16 sz1, uint8 *wb2, uint16 sz2)
{
	static uint8 tmp[NM_BUS_MAX_TRX_SZ];
	m2m_memcpy(tmp, wb1, sz1);
	m2m_memcpy(&tmp[sz1], wb2, sz2);
	return nm_i2c_write(tmp, sz1+sz2);
}
#endif

#ifdef CONF_WINC_USE_SPI

static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
	uint8 u8Dummy = 0;
	uint8 u8SkipMosi = 0, u8SkipMiso = 0;
	uint8_t txd_data = 0;
	uint8_t rxd_data = 0;

	if(((pu8Miso == NULL) && (pu8Mosi == NULL)) ||(u16Sz == 0)) {
		return M2M_ERR_INVALID_ARG;
	}

	if (pu8Mosi == NULL) {
		pu8Mosi = &u8Dummy;
		u8SkipMosi = 1;
	}
	if(pu8Miso == NULL) {
		pu8Miso = &u8Dummy;
		u8SkipMiso = 1;
	}

	SPI_NSS_PIN_LOW;
	while (u16Sz) {
		txd_data = *pu8Mosi;
		HAL_SPI_TransmitReceive(&hspi2,&txd_data,&rxd_data,1,1000);
//		HAL_Delay(1);
#if 0

		while( __HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_TXE) == false );
		//while (!spi_is_ready_to_write(&master));
		while( HAL_SPI_Transmit( &hspi2 , &txd_data, 1, SPI_TIMEOUT) != HAL_OK );
		//while(spi_write(&master, txd_data) != STATUS_OK);

		/* Read SPI master data register. */
		///while (!spi_is_ready_to_read(&master));

		while(  HAL_SPI_Receive( &hspi2, &rxd_data, 1, SPI_TIMEOUT) != HAL_OK );
		//while (spi_read(&master, &rxd_data) != STATUS_OK);
#endif
		*pu8Miso = rxd_data;
			
		u16Sz--;
		if (!u8SkipMiso)
			pu8Miso++;
		if (!u8SkipMosi)
			pu8Mosi++;
	}

	///while (!spi_is_write_complete(&master));

	SPI_NSS_PIN_HIGH;

	return M2M_SUCCESS;
}
#endif

/*
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_init(void *pvinit)
{
	sint8 result = M2M_SUCCESS;

#ifdef CONF_WINC_USE_I2C
	/* Initialize config structure and software module. */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);

	/* Change buffer timeout to something longer. */
	config_i2c_master.buffer_timeout = 1000;

	/* Initialize and enable device with config. */
	i2c_master_init(&i2c_master_instance, SERCOM2, &config_i2c_master);

	i2c_master_enable(&i2c_master_instance);

#elif defined CONF_WINC_USE_SPI
#if 0
	/* Structure for SPI configuration. */
	struct spi_config config;
	struct spi_slave_inst_config slave_config;

	/* Select SPI slave CS pin. */
	/* This step will set the CS high */
	spi_slave_inst_get_config_defaults(&slave_config);
	slave_config.ss_pin = CONF_WINC_SPI_CS_PIN;
	spi_attach_slave(&slave_inst, &slave_config);

	/* Configure the SPI master. */
	spi_get_config_defaults(&config);
	config.mux_setting = CONF_WINC_SPI_SERCOM_MUX;
	config.pinmux_pad0 = CONF_WINC_SPI_PINMUX_PAD0;
	config.pinmux_pad1 = CONF_WINC_SPI_PINMUX_PAD1;
	config.pinmux_pad2 = CONF_WINC_SPI_PINMUX_PAD2;
	config.pinmux_pad3 = CONF_WINC_SPI_PINMUX_PAD3;
	config.master_slave_select_enable = false;
	
	config.mode_specific.master.baudrate = CONF_WINC_SPI_CLOCK;
	if (spi_init(&master, CONF_WINC_SPI_MODULE, &config) != STATUS_OK) {
		return M2M_ERR_BUS_FAIL;
	}

	/* Enable the SPI master. */
	spi_enable(&master);
#endif
	MX_SPI2_Init();
	__HAL_SPI_ENABLE(&hspi2);
	nm_bsp_reset();
	nm_bsp_sleep(1);
#endif
	return result;
}

/*
*	@fn		nm_bus_ioctl
*	@brief	send/receive from the bus
*	@param[IN]	u8Cmd
*					IOCTL command for the operation
*	@param[IN]	pvParameter
*					Arbitrary parameter depenging on IOCTL
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@note	For SPI only, it's important to be able to send/receive at the same time
*/
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter)
{
	sint8 s8Ret = 0;
	switch(u8Cmd)
	{
#ifdef CONF_WINC_USE_I2C
		case NM_BUS_IOCTL_R: {
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_read(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
		case NM_BUS_IOCTL_W: {
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_write(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
		case NM_BUS_IOCTL_W_SPECIAL: {
			tstrNmI2cSpecial *pstrParam = (tstrNmI2cSpecial *)pvParameter;
			s8Ret = nm_i2c_write_special(pstrParam->pu8Buf1, pstrParam->u16Sz1, pstrParam->pu8Buf2, pstrParam->u16Sz2);
		}
		break;
#elif defined CONF_WINC_USE_SPI
		case NM_BUS_IOCTL_RW: {
			tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
			s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
		}
		break;
#endif
		default:
			s8Ret = -1;
			M2M_ERR("invalide ioclt cmd\n");
			break;
	}

	return s8Ret;
}

/*
*	@fn		nm_bus_deinit
*	@brief	De-initialize the bus wrapper
*/
sint8 nm_bus_deinit(void)
{
	sint8 result = M2M_SUCCESS;

	/* Configure control pins as input no pull up. */

#ifdef CONF_WINC_USE_I2C
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	i2c_master_disable(&i2c_master_instance);
	port_pin_set_config(CONF_WINC_I2C_SCL, &pin_conf);
	port_pin_set_config(CONF_WINC_I2C_SDA, &pin_conf);
#endif /* CONF_WINC_USE_I2C */
#ifdef CONF_WINC_USE_SPI
	HAL_GPIO_DeInit(GPIOB , WIFI___SPI2_NSS_Pin|WIFI___SPI2_SCK_Pin|WIFI___SPI2_MISO_Pin|WIFI___SPI2_MOSI_Pin );
	//port_pin_set_output_level(CONF_WINC_SPI_MOSI, false);
	//port_pin_set_output_level(CONF_WINC_SPI_MISO, false);
	//port_pin_set_output_level(CONF_WINC_SPI_SCK,  false);
	//port_pin_set_output_level(CONF_WINC_SPI_SS,   false);
#endif /* CONF_WINC_USE_SPI */
	return result;
}

/*
*	@fn			nm_bus_reinit
*	@brief		re-initialize the bus wrapper
*	@param [in]	void *config
*					re-init configuration data
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_reinit(void* config)
{
	return M2M_SUCCESS;
}

