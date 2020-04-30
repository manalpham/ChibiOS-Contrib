/*
    Serial eeprom interface for ChibiOS-Contrib
        Copyright (C) 2020 E. Bernet-Rollande aeroman@alpham.eu

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
 * @file    hal_serial_eeprom.h
 * @brief   Serial eeprom driver header.
 *
 * @addtogroup HAL_SERIAL_EEPROM
 * @{
 */

#ifndef HAL_SERIAL_EEPROM_H
#define HAL_SERIAL_EEPROM_H

#include "hal_base_eeprom.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Bus interface modes.
 * @{
 */
#define SEEPROM_BUS_DRIVER_SPI                 0U
#define SEEPROM_BUS_DRIVER_I2C                 1U
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Serial EEPROM has a write protection line
 */
#if !defined(SEEPROM_HAS_WRITE_PROTECTION_LINE) || defined(__DOXYGEN__)
#define SEEPROM_HAS_WRITE_PROTECTION_LINE		FALSE
#endif

/**
 * @brief   Serial EEPROM protection line active level
 */
#if !defined(SEEPROM_WP_ACTIVE_LEVEL) || defined(__DOXYGEN__)
#define SEEPROM_WP_ACTIVE_LEVEL					PAL_HIGH
#endif

/**
 * @brief   Physical transport interface.
 */
#if !defined(SEEPROM_BUS_DRIVER) || defined(__DOXYGEN__)
#define SEEPROM_BUS_DRIVER                      SEEPROM_BUS_DRIVER_I2C
#endif

/**
 * @brief   Shared bus switch.
 * @details If set to @p TRUE the device acquires bus ownership
 *          on each transaction.
 * @note    Requires @p SPI_USE_MUTUAL_EXCLUSION or
 *          @p I2C_USE_MUTUAL_EXCLUSION depending on mode selected
 *          with @p SEEPROM_BUS_MODE.
 */
#if !defined(SEEPROM_SHARED_BUS) || defined(__DOXYGEN__)
#define SEEPROM_SHARED_BUS                     TRUE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (SEEPROM_BUS_DRIVER == SEEPROM_BUS_DRIVER_I2C) || defined(__DOXYGEN__)
#define BUSConfig I2CConfig
#define BUSDriver I2CDriver
#elif SEEPROM_BUS_DRIVER == SEEPROM_BUS_DRIVER_SPI
#error "SPI bus mode not yet supported by this driver"
#define BUSConfig SPIConfig
#define BUSDriver SPIDriver
#else
#error "invalid SEEPROM_BUS_DRIVER setting"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a SEEPROM configuration structure.
 */
typedef struct {
  BUSDriver           *busp;
  const BUSConfig     *buscfg;
#if SEEPROM_HAS_WRITE_PROTECTION_LINE
  const ioline_t			wp_line;	  /* write protection line */
#endif
  const size_t				size;		    /* memory size           */
  const size_t				page_size;  /* memory page size      */
} SEEPROMConfig;

/**
 * @brief   @p SEEPROMDriver specific methods.
 */
#define _serial_eeprom_methods_alone

/**
 * @brief   @p SEEPROMDriver specific methods with inherited ones.
 */
#define _serial_eeprom_methods												                      \
	_base_eeprom_methods													                            \
	_serial_eeprom_methods_alone

/**
 * @brief   @p SEEPROMDriver virtual methods table.
 */
struct SEEPROMDriverVMT {
  _serial_eeprom_methods
};

#define _serial_eeprom_data													                        \
	_base_eeprom_data

/**
 * @extends BaseEeprom
 *
 * @brief   Type of serial eeprom class.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct SEEPROMDriverVMT *vmt;
  _serial_eeprom_data
  const SEEPROMConfig			*config;
} SEEPROMDriver;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

void seepromObjectInit(SEEPROMDriver *devp);
void seepromStart(SEEPROMDriver *devp, const SEEPROMConfig *config);
void seepromStop(SEEPROMDriver *devp);

#ifdef __cplusplus
}
#endif

/* Device-specific implementations.*/
#include "hal_eeprom_device.h"

#endif /* HAL_SERIAL_EEPROM_H */

/** @} */
