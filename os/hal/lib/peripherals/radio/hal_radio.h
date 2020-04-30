/*
    Radio interface for ChibiOS-Contrib
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
 * @brief   BaseRadio packet structure
 *          to be redefine by child.
 */
typedef struct radio_packet_t radio_packet_t;

/**
 * @brief   BaseRadio specific methods.
 */
#define _base_radio_methods_alone                                             \
  msg_t (*send)(void *instance, radio_packet_t *txp);                         \
  void  (*start_send)(void *instance, radio_packet_t *txp);                   \
  void (*stop_send)(void *instance);                                          \
  msg_t (*receive_timeout)(void *instance, radio_packet_t *rxp,               \
      sysinterval_t interval);                                                \
  void  (*start_receive_timeout) (void *instance, radio_packet_t *rxp,        \
      sysinterval_t interval);                                                \
  void (*stop_receive)(void *instance);

/**
 * @brief   BaseRadio specific methods with inherited ones.
 */
#define _base_radio_methods               	                                  \
  _base_object_methods                                                        \
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

/* #TODO : */
/**
 * @extends BaseObject
 *
 * @brief   Base stream class.
 * @details This class represent ....
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
 * @brief   Radio send packet, blocking.
 *
 * @param[in] ip        pointer to a @p BaseRadio or derived class.
 * @param[in] txp       pointer to the packet to send.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define radioSend(ip, txp)	          									                      \
				(ip)->vmt->send(ip, txp)

/**
 * @brief   Radio start send packet, non blocking.
 *
 * @param[in] ip        pointer to a @p BaseRadio or derived class.
 * @param[in] txp       pointer to the packet to send.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define radioStartSend(ip, txp)                                               \
        (ip)->vmt->start_send(ip, txp)

/**
 * @brief   Radio stop send packet.
 *
 * @param[in] ip        pointer to a @p BaseRadio or derived class.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define radioStopSend(ip)                                                     \
        (ip)->vmt->stop_send(ip)

/**
 * @brief   Radio receive packet, blocking.
 *
 * @param[in] ip        pointer to a @p BaseRadio or derived class.
 * @param[in] rxp       pointer to the received packet.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define radioReceive(ip, rxp)                                                 \
        (ip)->vmt->receive_timeout(ip, rxp, TIME_INFINITE)

/**
 * @brief   Radio receive packet with timeout, blocking.
 *
 * @param[in] ip        pointer to a @p BaseRadio or derived class.
 * @param[in] rxp       pointer to the received packet.
 * @param[in] tout		  timeout in ms (if 0 -> tout = infinite)
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define radioReceiveTimeout(ip, rxp, tout)					    			                \
				(ip)->vmt->receive_timeout(ip, rxp, tout)

/**
 * @brief   Radio start receive packet, non blocking.
 *
 * @param[in] ip        pointer to a @p BaseRadio or derived class.
 * @param[in] rxp       pointer to the received packet.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define radioStartReceive(ip, rxp)                                            \
        (ip)->vmt->start_receive_timeout(ip, rxp, TIME_INFINITE)

/**
 * @brief   Radio start receive packet with timeout, non blocking.
 *
 * @param[in] ip        pointer to a @p BaseRadio or derived class.
 * @param[in] rxp       pointer to the received packet.
 * @param[in] tout      timeout in ms (if 0 -> tout = infinite)
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define radioStartReceiveTimeout(ip, rxp, tout)                               \
        (ip)->vmt->start_receive_timeout(ip, rxp, tout)

/**
 * @brief   Radio stop receive packet.
 *
 * @param[in] ip        pointer to a @p BaseRadio or derived class.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define radioStopReceive(ip)                                                  \
        (ip)->vmt->stop_receive(ip)

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
