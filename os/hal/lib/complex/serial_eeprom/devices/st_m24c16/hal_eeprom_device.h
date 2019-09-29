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
 * @file    hal_eeprom_device.h
 * @brief   ST M24C16 device serial eeprom driver header.
 *
 * @addtogroup HAL_EEPROM_DEVICE
 * @{
 */

#ifndef HAL_EEPROM_DEVICE_H
#define HAL_EEPROM_DEVICE_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Device identification
 * @{
 */
#define M24C16_DEVICE_ID      0x50U		/* 7 bits slave address w/o shift */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

void seeprom_device_init(SEEPROMDriver * devp);
eeprom_error_t seeprom_device_read(SEEPROMDriver * devp, eeprom_offset_t offset, size_t n, uint8_t *rp);
eeprom_error_t seeprom_device_read_page(SEEPROMDriver * devp, eeprom_offset_t offset, size_t n, uint8_t *rp);
eeprom_error_t seeprom_device_write(SEEPROMDriver * devp, eeprom_offset_t offset, size_t n, const uint8_t *wp);
eeprom_error_t seeprom_device_write_page(SEEPROMDriver * devp, eeprom_offset_t offset, size_t n, const uint8_t *wp);

#ifdef __cplusplus
}
#endif

#endif /* HAL_EEPROM_DEVICE_H */

/** @} */
