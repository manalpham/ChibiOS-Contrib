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
 * @file    hal_radio.h
 * @brief   Generic radio driver class header.
 *
 * @addtogroup RADIO
 * @{
 */

#ifndef HAL_RADIO_H
#define HAL_RADIO_H

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
 * @brief   BaseRadio specific methods.
 */
#define _base_radio_methods_alone                                           \
  /* Send data.*/                                                           \
  msg_t (*send)(void *instance, uint8_t raddr, uint16_t n, uint8_t* txp);   \
  /* receive data with timeout */											\
  msg_t (*receive_timeout) (void *instance, uint16_t* np, uint8_t* rxp,     \
		  	  	  	  	  sysinterval_t interval);   						\
  /* receive data (polling) */												\
  msg_t (*receive) (void *instance, uint16_t* np, uint8_t* rxp);

/**
 * @brief   BaseRadio specific methods with inherited ones.
 */
#define _base_radio_methods               	                                \
  _base_object_methods                                                      \
  _base_radio_methods_alone

/**
 * @brief   @p BaseRadio virtual methods table.
 */
struct BaseRadioVMT {
  _base_radio_methods
};

/**
 * @brief   @p BaseRadio specific data.
 * @note    It is empty because @p BaseRadio is only an interface
 *          without implementation.
 */
#define _base_radio_data
  _base_object_data

/**
 * @extends BaseObject
 *
 * @brief   Base stream class.
 * @details This class represents a generic blocking unbuffered sequential
 *          data stream.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BaseRadioVMT *vmt;
  _base_radio_data
} BaseRadio;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Radio send packet.
 *
 * @param[in] ip        pointer to a @p BaseRadio or derived class.
 * @param[in] n         size of the packet to send.
 * @param[in] txp       pointer to the packet to send.
 * @param[in] raddr		addressee receiver address.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define radioSend(ip, raddr, n, txp)										\
				(ip)->vmt->send(ip, raddr, n, txp)

/**
 * @brief   Radio receive packet with timeout.
 *
 * @param[in] ip        pointer to a @p BaseRadio or derived class.
 * @param[in] np        pointer to the size of the received packet.
 * @param[in] rxp       pointer to the received packet.
 * @param[in] tout		timeout in ms (if 0 -> tout = infinite)
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define radioReceiveTimeout(ip, np, rxp, tout)								\
				(ip)->vmt->receive_timeout(ip, np, rxp, tout)

/**
 * @brief   Radio receive packet.
 *
 * @param[in] ip        pointer to a @p BaseRadio or derived class.
 * @param[in] np        pointer to the size of the received packet.
 * @param[in] rxp       pointer to the received packet.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define radioReceive(ip, np, rxp)											\
				(ip)->vmt->receive_timeout(ip, np, rxp, 0)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* HAL_RADIO_H */

/** @} */
