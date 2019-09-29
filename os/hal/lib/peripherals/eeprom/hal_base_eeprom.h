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
 * @file    hal_base_eeprom.h
 * @brief   Generic eeprom class header.
 *
 * @addtogroup HAL_BASE_EEPROM
 * @details This module define an abstract interface for generic eeprom.
 * @{
 */

#ifndef HAL_BASE_EEPROM_H
#define HAL_BASE_EEPROM_H

#include "hal_persistent.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  EEPROM_UNINIT     = 0,
  EEPROM_STOP       = 1,
  EEPROM_READY      = 2,
  EEPROM_READ       = 3,
  EEPROM_WRITE      = 4
} eeprom_state_t;

/**
 * @brief   Type of a eeprom error code.
 */
typedef ps_error_t eeprom_error_t;

/**
 * @brief   Type of a eeprom offset.
 */
typedef ps_offset_t eeprom_offset_t;

/**
 * @brief   @p BaseEeprom specific methods.
 */
#define _base_eeprom_methods_alone                                          \
  /* Page size. */                                                          \
  size_t (*get_page_size)(void *instance);                                  \
  /* Number of pages. */                                                    \
  uint32_t (*get_page_number)(void *instance);                              \
  /* Read page operation.*/                                                 \
  eeprom_error_t (*read_page)(void *instance, eeprom_offset_t offset,       \
                     size_t n, uint8_t *rp);                                \
  /* Write page operation.*/                                                \
  eeprom_error_t (*write_page)(void *instance, eeprom_offset_t offset,      \
                     size_t n, const uint8_t *wp);

/**
 * @brief   @p BaseEeprom specific methods with inherited ones.
 */
#define _base_eeprom_methods                                                \
	_base_pers_storage_methods                                              \
	_base_eeprom_methods_alone

/**
 * @brief   @p BaseEeprom virtual methods table.
 */
struct BaseEepromVMT {
  _base_eeprom_methods
};

/**
 * @brief   @p BaseEeprom specific data.
 */
#define _base_eeprom_data                                                   \
	_base_persistent_storage_data											\
	/* Driver state.*/                                                      \
	eeprom_state_t		state;												\
	uint32_t			page_number;
/**
 * @extends BasePersistentStorage
 *
 * @brief   Base eeprom class.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BaseEepromVMT *vmt;
  _base_eeprom_data
} BaseEeprom;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions (BaseEeprom)
 * @{
 */
/**
 * @brief   Instance getter.
 * @details This special method is used to get the instance of this class
 *          object from a derived class.
 */
#define getBaseEeprom(ip)													\
  ((BaseEeprom *)&(ip)->vmt)

/**
 * @brief   Get eeprom size.
 *
 * @param[in] ip        pointer to a @p BaseEeprom or derived class
 * @return              The eeprom size in bytes.
 *
 * @api
 */
#define eepromGetSize(ip)                                                   \
  (ip)->vmt->getsize(ip)

/**
 * @brief   Get eeprom page size.
 *
 * @param[in] ip        pointer to a @p BaseEeprom or derived class
 * @return              The eeprom size in bytes.
 *
 * @api
 */
#define eepromGetPageSize(ip)                                               \
  (ip)->vmt->get_page_size(ip)

/**
 * @brief   Get eeprom number of pages.
 *
 * @param[in] ip        pointer to a @p BaseEeprom or derived class
 * @return              The eeprom pages number.
 *
 * @api
 */
#define eepromGetPageNumber(ip)                                             \
  (ip)->vmt->get_page_number(ip)

/**
 * @brief   Read operation.
 *
 * @param[in] ip        pointer to a @p BaseEeprom or derived class
 * @param[in] offset    eeprom offset
 * @param[in] n         number of bytes to be read
 * @param[out] rp       pointer to the data buffer
 * @return              An error code.
 * @retval PS_NO_ERROR  if there is no erase operation in progress.
 * @retval PS_ERROR_READ if the read operation failed.
 * @retval PS_ERROR_HW_FAILURE if access to the memory failed.
 *
 * @api
 */
#define eepromRead(ip, offset, n, rp)                                       \
  (ip)->vmt->read(ip, offset, n, rp)

/**
 * @brief   Read page operation.
 *
 * @param[in] ip        pointer to a @p BaseEeprom or derived class
 * @param[in] offset    eeprom page offset
 * @param[in] n         number of pages to be read
 * @param[out] rp       pointer to the data buffer
 * @return              An error code.
 * @retval PS_NO_ERROR  if there is no erase operation in progress.
 * @retval PS_ERROR_READ if the read operation failed.
 * @retval PS_ERROR_HW_FAILURE if access to the memory failed.
 *
 * @api
 */
#define eepromReadPage(ip, offset, n, rp)                                   \
  (ip)->vmt->read_page(ip, offset, n, rp)

/**
 * @brief   Write operation.
 *
 * @param[in] ip        pointer to a @p BaseEeprom or derived class
 * @param[in] offset    eeprom offset
 * @param[in] n         number of bytes to be written
 * @param[in] wp        pointer to the data buffer
 * @return              An error code.
 * @retval PS_NO_ERROR  if there is no erase operation in progress.
 * @retval PS_ERROR_WRITE if the write operation failed.
 * @retval PS_ERROR_HW_FAILURE if access to the memory failed.
 *
 * @api
 */
#define eepromWrite(ip, offset, n, wp)                                      \
  (ip)->vmt->write(ip, offset, n, wp)

/**
 * @brief   Write page operation.
 *
 * @param[in] ip        pointer to a @p BaseEeprom or derived class
 * @param[in] offset    eeprom page offset
 * @param[in] n         number of pages to be written
 * @param[in] wp        pointer to the data buffer
 * @return              An error code.
 * @retval PS_NO_ERROR  if there is no erase operation in progress.
 * @retval PS_ERROR_WRITE if the write operation failed.
 * @retval PS_ERROR_HW_FAILURE if access to the memory failed.
 *
 * @api
 */
#define eepromWritePage(ip, offset, n, wp)                                  \
  (ip)->vmt->write_page(ip, offset, n, wp)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* HAL_BASE_EEPROM_H */

/** @} */
