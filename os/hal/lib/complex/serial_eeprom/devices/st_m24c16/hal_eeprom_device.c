/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_serial_eeprom_device.c
 * @brief   ST M24C16 device serial eeprom driver code.
 *
 * @addtogroup HAL_SERIAL_EEPROM_DEVICE
 * @{
 */

#include "hal.h"
#include "hal_serial_eeprom.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void seeprom_device_init(SEEPROMDriver * devp) {
	/* set write protection on */
	palWriteLine(devp->config->wp_line, SEEPROM_WP_ACTIVE_LEVEL);
}

eeprom_error_t seeprom_device_read(SEEPROMDriver * devp, eeprom_offset_t offset, size_t n, uint8_t *rp) {
	/* Set 1st byte address */
	uint8_t dev_select = M24C16_DEVICE_ID | ((uint8_t) ((offset & 0x700) >> 8U));
	uint8_t adr_byte   = (uint8_t) (offset & 0xFF);

	msg_t msg;

	/* polling on dev_select ACK before read bytes */
	do {
		msg  = i2cMasterTransmitTimeout(devp->config->busp, dev_select, &adr_byte, 1, rp, n, TIME_MS2I(1000));
	} while ( i2cGetErrors(devp->config->busp) == I2C_ACK_FAILURE );

	switch (msg) {
		case MSG_OK:
			return PS_NO_ERROR;
		default:
			return PS_ERROR_READ;
	}
}

eeprom_error_t seeprom_device_read_page(SEEPROMDriver * devp, eeprom_offset_t offset, size_t n, uint8_t *rp) {
	/* page offset in bytes */
	eeprom_offset_t bytes_offset = offset * devp->config->page_size;

	/* Set 1st page address */
	uint8_t dev_select = M24C16_DEVICE_ID | ((uint8_t) ((bytes_offset & 0x700) >> 8U));
	uint8_t adr_byte   = (uint8_t) (bytes_offset & 0xFF);

	msg_t msg;

	/* polling on dev_select ACK before read pages */
	do {
		msg  = i2cMasterTransmitTimeout(devp->config->busp, dev_select, &adr_byte, 1,
				                        rp, n * devp->config->page_size, TIME_MS2I(1000));
	} while ( i2cGetErrors(devp->config->busp) == I2C_ACK_FAILURE );

	switch (msg) {
		case MSG_OK:
			return PS_NO_ERROR;
		default:
			return PS_ERROR_READ;
	}
}

eeprom_error_t seeprom_device_write(SEEPROMDriver * devp, eeprom_offset_t offset, size_t n, const uint8_t *wp) {
	uint8_t dev_select;
	uint8_t tx_data[2];

	msg_t msg;

	/* disable write protection */
	palWriteLine(devp->config->wp_line, !SEEPROM_WP_ACTIVE_LEVEL);

	/* write each byte separately */
	for(uint32_t idx = 0 ; idx < n ; idx++) {
		/* Set byte address */
		dev_select = M24C16_DEVICE_ID | ((uint8_t) (((offset + idx) & 0x700) >> 8U));
		tx_data[0] = (uint8_t) ((offset + idx) & 0xFF);

		/* fill temporary data buffer */
		tx_data[1] = wp[idx];

		/* polling on dev_select ACK before write byte*/
		do {
			msg  = i2cMasterTransmitTimeout(devp->config->busp, dev_select, tx_data, 2, NULL, 0, TIME_MS2I(1000));
		} while ( i2cGetErrors(devp->config->busp) == I2C_ACK_FAILURE );
	}

	/* enable write protection */
	palWriteLine(devp->config->wp_line, SEEPROM_WP_ACTIVE_LEVEL);

	switch (msg) {
		case MSG_OK:
			return PS_NO_ERROR;
		default:
			return PS_ERROR_WRITE;
	}
}

eeprom_error_t seeprom_device_write_page(SEEPROMDriver * devp, eeprom_offset_t offset, size_t n, const uint8_t *wp) {
	/* page offset in bytes */
	eeprom_offset_t bytes_offset = offset * devp->config->page_size;

	uint8_t dev_select;
	uint8_t tx_data[devp->config->page_size + 1];

	msg_t msg;

	/* disable write protection */
	palWriteLine(devp->config->wp_line, !SEEPROM_WP_ACTIVE_LEVEL);

	/* write each page separately */
	for(uint32_t idx = 0 ; idx < n * devp->config->page_size ; idx += devp->config->page_size) {
		/* Set page address */
		dev_select = M24C16_DEVICE_ID | ((uint8_t) (((bytes_offset + idx) & 0x700) >> 8U));
		tx_data[0] = (uint8_t) ((bytes_offset + idx) & 0xFF);

		/* fill temporary data buffer */
		for(uint32_t byte_idx = 0 ; byte_idx < devp->config->page_size ; byte_idx++) {
			tx_data[byte_idx + 1] = wp[idx + byte_idx];
		}

		/* polling on dev_select ACK before write page*/
		do {
			msg  = i2cMasterTransmitTimeout(devp->config->busp, dev_select, tx_data,
					                        devp->config->page_size + 1, NULL, 0, TIME_MS2I(1000));
		} while ( i2cGetErrors(devp->config->busp) == I2C_ACK_FAILURE );
	}

	/* enable write protection */
	palWriteLine(devp->config->wp_line, SEEPROM_WP_ACTIVE_LEVEL);

	switch (msg) {
		case MSG_OK:
			return PS_NO_ERROR;
		default:
			return PS_ERROR_WRITE;
	}
}

/** @} */
