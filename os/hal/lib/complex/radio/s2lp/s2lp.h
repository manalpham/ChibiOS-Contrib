/*
    ChibiOS - Copyright (C) 2016..2018 Rocco Marco Guglielmi

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/**
 * @file    st_s2lp.h
 * @brief   S2LP radio interface module header.
 *
 *
 * @addtogroup S2LP
 * @{
 */
#ifndef _ST_S2LP_H_
#define _ST_S2LP_H_

#include "hal_radio.h"
#include "S2LP_Config.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/


#define MSG_MAX_BO_CCA_REACH			MAX_BO_CCA_REACH
#define MSG_TX_FIFO_ALMOST_EMPTY		TX_FIFO_ALMOST_EMPTY
#define MSG_TX_FIFO_ERROR				TX_FIFO_ERROR
#define MSG_RX_FIFO_ALMOST_FULL			RX_FIFO_ALMOST_FULL
#define MSG_RX_FIFO_ERROR				RX_FIFO_ERROR
#define MSG_RX_DATA_DISC				RX_DATA_DISC
#define MSG_RX_TIMEOUT					RX_TIMEOUT
#define MSG_CRC_ERROR					CRC_ERROR

#define S2LP_FIFO_SIZE					128
/* RX FIFO ALMOST FULL THRESHOLD	*/
#define S2LP_RX_FIFO_AF_THRESHOLD		16
/* RX FIFO ALMOST EMPTY THRESHOLD	*/
#define S2LP_RX_FIFO_AE_THRESHOLD		16
/* TX FIFO ALMOST FULL THRESHOLD	*/
#define S2LP_TX_FIFO_AF_THRESHOLD		16
/* TX FIFO ALMOST EMPTY THRESHOLD	*/
#define S2LP_TX_FIFO_AE_THRESHOLD		16

#define RX_IRQ_PENDING					0x00000001
#define RX_FIFO_AF_IRQ_PENDING			0x00000002
#define TX_IRQ_PENDING					0x00000004
#define TX_FIFO_AE_IRQ_PENDING			0x00000008

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   S2LP shared SPI switch.
 * @details If set to @p TRUE the device acquires SPI bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires SPI_USE_MUTUAL_EXCLUSION
 */
#if !defined(S2LP_SHARED_SPI) || defined(__DOXYGEN__)
#define S2LP_SHARED_SPI                 FALSE
#endif

/**
 * @brief	S2LP Basic Packet protocol switch
 * @details If set to @p TRUE the Basic packet protocol is used.
 * @note	The default is @p TRUE
 */
#if !defined(S2LP_USE_BASIC_PROTOCOL) || defined(__DOXYGEN__)
#define S2LP_USE_BASIC_PROTOCOL			TRUE
#endif

/**
 * @brief	S2LP CSMA engine switch
 * @details If set to @p TRUE the CSMA engine is activated the transmitter
 * 			proceed a channel sensing before transmitting.
 * @note	The default is @p TRUE
 */
#if !defined(S2LP_USE_CSMA_ENGINE) || defined(__DOXYGEN__)
#define S2LP_USE_CSMA_ENGINE			TRUE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !HAL_USE_SPI
#error "S2LP requires an SPI bus to operate"
#endif

#if S2LP_SHARED_SPI && !SPI_USE_MUTUAL_EXCLUSION
#error "S2LP_SHARED_SPI requires SPI_USE_MUTUAL_EXCLUSION"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    S2LP data structures and types.
 * @{
 */
/**
 * @brief   Structure representing a S2LP driver.
 */
typedef struct S2LPDriver S2LPDriver;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  S2LP_UNINIT   = 0,            /**< Not initialized.                   	*/
  S2LP_STOP     = 1,            /**< Stopped.                           	*/
  S2LP_READY    = 2,            /**< Ready.                             	*/
  S2LP_READY_RX = 3,			/**< Ready, configured for RX				*/
  S2LP_READY_TX = 4,			/**< Ready, configured for TX				*/
  S2LP_TX	    = 5,			/**< TX operation ongoing					*/
  S2LP_RX	    = 6,			/**< RX operation ongoing					*/
} s2lp_state_t;

/**
 * @brief Transceiver receiving mode
 */
typedef enum {
	S2LP_RX_TIMEOUT	= 0,
	S2LP_RX_SNIFF	= 1,
	S2LP_RX_LDC		= 2
} s2lp_rx_mode_t;

typedef struct {
	  /**
	   * @brief GPIO line associated to the shutdown pin (SDN)
	   * 		of this S2LP
	   */
	  ioline_t				sdn_line;
	  /**
	   * @brief GPIO line used for managing events from S2LP
	   * 		MCU side
	   */
	  ioline_t				mcu_irq_line;

	  /**
	   * @brief GPIO line used for managing events from S2LP
	   * 		S2LP side
	   */
	  SGpioInit				*irq_line_cfg;

	  ioline_t				mcu_fifo_irq_line;

	  SGpioInit				*fifo_irq_line_cfg;


} S2LPGPIOConfig;

/**
 * @brief   S2LP configuration structure.
 */
typedef struct {
  /**
   * @brief SPI driver associated to this S2LP.
   */
  SPIDriver             *spip;
  /**
   * @brief SPI configuration associated to this S2LP.
   */
  SPIConfig           	*spi_cfg;

  S2LPGPIOConfig		*gpio_cfg;

  SRadioInit			*radio_cfg;
#if S2LP_USE_BASIC_PROTOCOL
  /**
   * @brief	Basic Protocol part configuration of S2LP
   */
  PktBasicInit			*pkt_basic_cfg;
  /**
   * @brief Basic protocol addresses part configuration of S2LP
   */
  PktBasicAddressesInit	*pkt_basic_addr_cfg;
#endif
#if S2LP_USE_CSMA_ENGINE
  /**
   * @brief	CSMA engine part configuration of S2LP
   */
  SCsmaInit				*csma_cfg;
  int32_t				csma_rssi_threshold;
#endif
  /**
   * @brief external oscillator frequency
   */
  uint32_t				xtal_freq;
  /**
   * @brief Max power ramping index for tx operations
   */
  uint8_t				pwr_idx;
  /**
   * @brief Max power generation (dBm) for tx operations
   */
  float					pwr_dbm;
  /**
   * @brief RSSI Threshold
   */
  int32_t				rssi_threshold;
} S2LPConfig;

/**
 * @brief   @p S2LP specific methods.
 * @note    No methods so far, just a common ancestor interface.
 */
#define _s2lp_methods_alone

/**
 * @brief   @p S2LP specific methods with inherited ones.
 */
#define _s2lp_methods                                                    	\
  _base_object_methods                                                      \
  _s2lp_methods_alone

/**
 * @extends BaseObjectVMT
 *
 * @brief @p S2LP virtual methods table.
 */
struct S2LPVMT {
  _s2lp_methods
};

/**
 * @brief   @p S2LPDriver specific data.
 */
#define _s2lp_data                                                       	\
  /* Driver state.*/                                                        \
  s2lp_state_t      	state;                                         		\
  /* Current configuration data.*/                                          \
  S2LPConfig        	*config;

/**
 * @brief   S2LP radio transceiver class.
 */
struct S2LPDriver {
  /** @brief Virtual Methods Table.*/
  const struct S2LPVMT	*vmt;
  /** @brief Base radio interface.*/
  BaseRadio	            	radio_if;
  _s2lp_data
  thread_reference_t		thread;
  volatile uint32_t			irq_pending;
};

/**
 * @brief   Type of a s2lp event callback.
 */
typedef msg_t (*s2lpcallback_t)(void *arg);

/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   S2LP send packet.
 *
 * @param[in] devp      pointer to a @p S2LPDriver or derived class.
 * @param[in] n         size of the packet to send.
 * @param[in] txp       pointer to the packet to send.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define s2lpSend(devp, raddr, n, txp)										\
			radioSend(&((devp)->radio_if), raddr, n, txp)

/**
 * @brief   S2LP receive packet with timeout.
 *
 * @param[in] devp      pointer to a @p S2LPDriver.
 * @param[in] np        pointer to the size of the received packet.
 * @param[in] rxp       pointer to the received packet.
 * @param[in] interval	timeout in system time interval
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define s2lpReceiveTimeout(devp, np, txp, interval)							\
			radioReceiveTimeout(&((devp)->radio_if), np, txp, interval)

/**
 * @brief   S2LP receive packet (polling).
 *
 * @param[in] devp      pointer to a @p S2LPDriver.
 * @param[in] np        pointer to the size of the received packet.
 * @param[in] rxp       pointer to the received packet.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define s2lpReceive(devp, np, txp)											\
			radioReceive(&((devp)->radio_if), np, txp)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void	s2lpObjectInit(S2LPDriver *devp);
  void	s2lpStart(S2LPDriver *devp, S2LPConfig *config);
  msg_t	s2lpReceiveSniff(S2LPDriver *devp, uint16_t* n, uint8_t* rxp);
  void	s2lpStop(S2LPDriver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _ST_S2LP_H_ */

/** @} */
