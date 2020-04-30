/*
    S2LP for ChibiOS-Contrib - Copyright (C) 2020 E. Bernet-Rollande
                                                   aeroman@alpham.eu

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
 * @file    st_s2lp.h
 * @brief   S2LP radio interface module header.
 *
 *
 * @addtogroup S2LP
 * @{
 */
#ifndef _ST_S2LP_H_
#define _ST_S2LP_H_

#include "ch.h"
#include "hal.h"
#include "hal_radio.h"
#include "S2LP_Config.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define MSG_MAX_BO_CCA_REACH			0x00000001
#define MSG_TX_FIFO_ERROR				  0x00000002
#define MSG_RX_FIFO_ERROR				  0x00000004
#define MSG_RX_DATA_DISC				  0x00000008
#define MSG_RX_TIMEOUT					  0x00000010
#define MSG_CRC_ERROR					    0x00000020
#define MSG_STOPPED               0x00000040
#define MSG_UNEXPECTED_ERROR      0x00000080
#define MSG_START_TXRX            0x00000100
#define MSG_TERMINATE             0x00000200

#define EVT_GLOBAL_IRQ_PENDING		0x00000010
#define EVT_FIFO_IRQ_PENDING		  0x00000020
#define EVT_STOP_TXRX             0x00000040

#define S2LP_FIFO_SIZE					  128
/* RX FIFO ALMOST FULL THRESHOLD	*/
#define S2LP_RX_FIFO_AF_THRESHOLD	16
/* RX FIFO ALMOST EMPTY THRESHOLD	*/
#define S2LP_RX_FIFO_AE_THRESHOLD	16
/* TX FIFO ALMOST FULL THRESHOLD	*/
#define S2LP_TX_FIFO_AF_THRESHOLD	16
/* TX FIFO ALMOST EMPTY THRESHOLD	*/
#define S2LP_TX_FIFO_AE_THRESHOLD	16

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
#define S2LP_SHARED_SPI             FALSE
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
#define S2LP_USE_CSMA_ENGINE			  TRUE
#endif

/**
 * @brief S2LP max packet size to transcieve
 * @details The maximum size of the packet to be
 *          sent or received
 */
#if !defined(S2LP_MAX_PACKET_SIZE) || defined(__DOXYGEN__)
#define S2LP_MAX_PACKET_SIZE        0xFFFF
#endif

/**
 * @brief S2LP Working Threads Priority
 */
#if !defined(S2LP_WORK_PRIORITY) || defined(__DOXYGEN__)
#define S2LP_WORK_PRIORITY          (NORMALPRIO + 20)
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

#if (S2LP_MAX_PACKET_SIZE > 0xFFFF)
#error "S2LP can not transceives packets with size > 0xFFFF"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    S2LP data structures and types.
 * @{
 */

/**
 * @brief   Structure representing an S2LP driver.
 */
typedef struct S2LPDriver S2LPDriver;

/**
 * @brief   Structure representing an S2LP packet.
 */
typedef struct radio_packet_t {
  /* address of the receiver of a packet to send */
  uint8_t  address;
  /* size of the received payload / the payload to send */
  uint16_t size;
  /* reference to the received payload / the payload to send */
  uint8_t  *payload;
  /* error message after tranceive */
  msg_t    error;
} s2lp_packet_t;

/**
 * @brief   Type of an s2lp notification callback.
 */
typedef void (*s2lpcallback_t)(S2LPDriver *devp);

typedef void (*s2lp_finalize_t) (S2LPDriver *devp);

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  S2LP_UNINIT   = 0,      /**< Not initialized.                   	*/
  S2LP_STOP     = 1,      /**< Stopped.                           	*/
  S2LP_READY    = 2,      /**< Ready.                             	*/
  S2LP_READY_RX = 3,			/**< Ready, configured for RX	            */
  S2LP_READY_TX = 4,			/**< Ready, configured for TX	            */
  S2LP_RX	      = 6,			/**< RX operation ongoing			            */
  S2LP_TX       = 5,      /**< TX operation ongoing                 */
} s2lp_state_t;

typedef struct {
	  /**
	   * @brief GPIO line associated to the shutdown pin (SDN)
	   * 		of this S2LP
	   */
	  ioline_t				    sdn_line;
	  /**
	   * @brief GPIO line used for managing events from S2LP
	   * 		MCU side
	   */
	  ioline_t				    mcu_irq_line;
	  /**
	  * @brief GPIO line used for managing fifo events from S2LP
	  * 		MCU side
	  */
	  ioline_t				    mcu_fifo_irq_line;
	  /**
	   * @brief GPIO line used for managing events from S2LP
	   * 		S2LP side
	   */
	  SGpioInit				    *irq_line_cfg;
	  /**
	  * 		S2LP side
	  * @brief GPIO line used for managing FIFO events from S2LP
	  */
	  SGpioInit				    *fifo_irq_line_cfg;
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
  SPIConfig             *spi_cfg;

  S2LPGPIOConfig		    *gpio_cfg;

  SRadioInit			      *radio_cfg;
  s2lpcallback_t        tx_end_cb;
  s2lpcallback_t        rx_end_cb;
#if S2LP_USE_BASIC_PROTOCOL
  /**
   * @brief	Basic Protocol part configuration of S2LP
   */
  PktBasicInit			    *pkt_basic_cfg;
  /**
   * @brief Basic protocol addresses part configuration of S2LP
   */
  PktBasicAddressesInit	*pkt_basic_addr_cfg;
#endif
#if S2LP_USE_CSMA_ENGINE
  /**
   * @brief	CSMA engine part configuration of S2LP
   */
  SCsmaInit				      *csma_cfg;
  int32_t				        csma_rssi_threshold;
#endif
  /**
   * @brief external oscillator frequency
   */
  uint32_t				      xtal_freq;
  /**
   * @brief Max power ramping index for tx operations
   */
  uint8_t				        pwr_idx;
  /**
   * @brief Max power generation (dBm) for tx operations
   */
  float					        pwr_dbm;
  /**
   * @brief RSSI Threshold
   */
  int32_t				        rssi_threshold;
} S2LPConfig;

/**
 * @brief   @p S2LP specific methods.
 * @note    No methods so far, just a common ancestor interface.
 */
#define _s2lp_methods_alone

/**
 * @brief   @p S2LP specific methods with inherited ones.
 */
#define _s2lp_methods                                                    	    \
  _base_object_methods                                                        \
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
#define _s2lp_data                                                       	    \
  /* Driver state.*/                                                          \
  s2lp_state_t      	state;                                         		      \
  /* Current configuration data.*/                                            \
  S2LPConfig        	*config;											                          \
  /* Caller thread reference */                                               \
  thread_reference_t	thread;												                          \
  /* TX/RX worker thread reference */                                         \
  thread_reference_t	txrx_work;											                        \
  /* TX/RX operation finalize function */                                     \
  s2lp_finalize_t     finalize;                                               \
  /* Device lock */                                                           \
  mutex_t				      lock;												                            \
  /* head of the next byte in the packet to send/receive */                   \
  uint16_t            payload_head;                                           \
  /* where is the packet to send / where goes the received packet */          \
  s2lp_packet_t				*packet;

/**
 * @brief   S2LP radio transceiver class.
 */
struct S2LPDriver {
  /** @brief Virtual Methods Table.*/
  const struct S2LPVMT  *vmt;
  /** @brief Base radio interface.*/
  BaseRadio	            radio_if;
  _s2lp_data
};

/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   S2LP send packet, blocking.
 *
 * @param[in] devp      pointer to a @p S2LPDriver or derived class.
 * @param[in] txp       pointer to the packet to send.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define s2lpSend(devp, txp)	          									                      \
			radioSend(&((devp)->radio_if), txp)

/**
 * @brief   S2LP start send packet, non blocking.
 *
 * @param[in] devp      pointer to a @p S2LPDriver or derived class.
 * @param[in] txp       pointer to the packet to send.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define s2lpStartSend(devp, txp)                                              \
      radioStartSend(&((devp)->radio_if), txp)

/**
 * @brief   S2LP stop send packet.
 *
 * @param[in] devp      pointer to a @p S2LPDriver or derived class.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define s2lpStopSend(devp)                                                    \
      radioStopSend(&((devp)->radio_if))

/**
 * @brief   S2LP receive packet, blocking.
 *
 * @param[in] devp      pointer to a @p S2LPDriver.
 * @param[in] rxp       pointer to the received packet.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define s2lpReceive(devp, rxp)                                                \
      radioReceiveTimeout(&((devp)->radio_if), rxp, TIME_INFINITE)

/**
 * @brief   S2LP receive packet with timeout, blocking.
 *
 * @param[in] devp      pointer to a @p S2LPDriver.
 * @param[in] rxp       pointer to the received packet.
 * @param[in] interval	timeout in system time interval.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define s2lpReceiveTimeout(devp, rxp, interval)	    						              \
			radioReceiveTimeout(&((devp)->radio_if), rxp, interval)

/**
 * @brief   S2LP start receive packet, non blocking.
 *
 * @param[in] devp      pointer to a @p S2LPDriver.
 * @param[in] rxp       pointer to the received packet.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define s2lpStartReceive(devp, rxp)                                           \
      radioStartReceiveTimeout(&((devp)->radio_if), rxp, TIME_INFINITE)

/**
 * @brief   S2LP start receive packet with timeout, non blocking.
 *
 * @param[in] devp      pointer to a @p S2LPDriver.
 * @param[in] rxp       pointer to the received packet.
 * @param[in] interval  timeout in system time interval.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define s2lpStartReceiveTimeout(devp, rxp, interval)                          \
      radioStartReceiveTimeout(&((devp)->radio_if), rxp, interval)

/**
 * @brief   S2LP stop receive packet.
 *
 * @param[in] devp      pointer to a @p S2LPDriver or derived class.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define s2lpStopReceive(devp)                                                 \
      radioStopReceive(&((devp)->radio_if))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void	s2lpObjectInit(S2LPDriver *devp);
  void	s2lpStart(S2LPDriver *devp, S2LPConfig *config);
  void	s2lpStop(S2LPDriver *devp);
  void  s2lpAquire(S2LPDriver *devp);
  void  s2lpRelease(S2LPDriver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _ST_S2LP_H_ */

/** @} */
