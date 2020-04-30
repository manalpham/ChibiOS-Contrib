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
 * @file    s2lp.c
 * @brief   S2LP radio interface module code.
 *
 * @addtogroup S2LP
 * @{
 */

#include "s2lp.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/* defaults configurations */
static SGpioInit irq_line_cfg_default = {
	  .xS2LPGpioPin		= S2LP_GPIO_0,
	  .xS2LPGpioMode	= S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
	  .xS2LPGpioIO		= S2LP_GPIO_DIG_OUT_IRQ
};

static SGpioInit fifo_irq_line_cfg_default = {
	  .xS2LPGpioPin		= S2LP_GPIO_1,
	  .xS2LPGpioMode	= S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
	  .xS2LPGpioIO		= S2LP_GPIO_DIG_OUT_GND
};

static SRadioInit radio_cfg_default = {
		.lFrequencyBase		  = 868.0e6,
		.xModulationSelect  = MOD_2GFSK_BT1,
		.lDatarate		    	= 100000,
		.lFreqDev			      = 100e3,
		.lBandwidth			    = 400e3
};

#if S2LP_USE_BASIC_PROTOCOL
static PktBasicInit pkt_basic_cfg_default = {
		.xPreambleLength		  = 128,
		.xSyncLength			    = 32,
		.lSyncWords				    = 0x88888888,
		.xFixVarLength			  = S_ENABLE,
		.cExtendedPktLenField	= S_ENABLE,
		.xCrcMode				      = PKT_CRC_MODE_8BITS,
		.xAddressField			  = S_ENABLE,
		.xFec					        = S_DISABLE,
		.xDataWhitening			  = S_ENABLE
};

static PktBasicAddressesInit pkt_basic_addr_cfg_default = {
		.xFilterOnMyAddress 		    = S_ENABLE,
		.cMyAddress					        = 0x44,
		.xFilterOnMulticastAddress	= S_ENABLE,
		.cMulticastAddress			    = 0xEE,
		.xFilterOnBroadcastAddress	= S_ENABLE,
		.cBroadcastAddress			    = 0xFF
};
#endif

#if S2LP_USE_CSMA_ENGINE
static SCsmaInit csma_cfg_default = {
		.xCsmaPersistentMode	= S_DISABLE,
		.xMultiplierTbit		  = CSMA_PERIOD_64TBIT,
		.xCcaLength				    = 3,
		.cMaxNb					      = 5,
		.nBuCounterSeed			  = 0xFA21,
		.cBuPrescaler			    = 32
};
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/


/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void s2lp_enter_shutdown(S2LPDriver *devp) {
	palSetLine(devp->config->gpio_cfg->sdn_line);
	chThdSleep(TIME_MS2I(1));
}

static void s2lp_exit_shutdown(S2LPDriver *devp) {
	palClearLine(devp->config->gpio_cfg->sdn_line);
	chThdSleep(TIME_MS2I(1));
}

static SFlagStatus s2lp_check_shutdown(S2LPDriver *devp) {
	return (SFlagStatus) palReadLine(devp->config->gpio_cfg->sdn_line);
}

static S2LPStatus s2lp_spi_write_header(S2LPDriver *devp, uint8_t reg_type, uint8_t reg) {
	uint8_t rxFrame[2];
	S2LPStatus* tmpStatus = (S2LPStatus*) rxFrame;
	chSysLock();
	rxFrame[1] = spiPolledExchange(devp->config->spip, reg_type);
	rxFrame[0] = spiPolledExchange(devp->config->spip, reg);
	chSysUnlock();
	return *tmpStatus;
}

S2LPStatus s2lp_spi_write_regs(S2LPDriver *devp, uint8_t addr, uint8_t n, uint8_t *regsp) {
	S2LPStatus status;
#if (S2LP_SHARED_SPI)
	spiAcquireBus(devp->config->spip);
#endif
	spiStart(devp->config->spip, devp->config->spi_cfg);
	spiSelect(devp->config->spip);
	status = s2lp_spi_write_header(devp, WRITE_HEADER, addr);
	spiSend(devp->config->spip, n, regsp);
	spiUnselect(devp->config->spip);
#if (S2LP_SHARED_SPI)
	spiReleaseBus(devp->config->spip);
#endif
	return status;
}

S2LPStatus s2lp_spi_read_regs(S2LPDriver *devp, uint8_t addr, uint8_t n, uint8_t *regsp) {
	S2LPStatus status;
#if (S2LP_SHARED_SPI)
	spiAcquireBus(devp->config->spip);
#endif
	spiStart(devp->config->spip, devp->config->spi_cfg);
	spiSelect(devp->config->spip);
	status = s2lp_spi_write_header(devp, READ_HEADER, addr);
	spiReceive(devp->config->spip, n, regsp);
	spiUnselect(devp->config->spip);
#if (S2LP_SHARED_SPI)
	spiReleaseBus(devp->config->spip);
#endif
	return status;
}

S2LPStatus s2lp_spi_cmd_strobes(S2LPDriver *devp, uint8_t cmd) {
	S2LPStatus status;
#if (S2LP_SHARED_SPI)
	spiAcquireBus(devp->config->spip);
#endif
	spiStart(devp->config->spip, devp->config->spi_cfg);
	spiSelect(devp->config->spip);
	status = s2lp_spi_write_header(devp, COMMAND_HEADER, cmd);
	spiUnselect(devp->config->spip);
#if (S2LP_SHARED_SPI)
	spiReleaseBus(devp->config->spip);
#endif
	return status;
}

static S2LPStatus s2lp_spi_write_fifo(S2LPDriver *devp, uint8_t n, uint8_t *buffp) {
	return s2lp_spi_write_regs(devp, LINEAR_FIFO_ADDRESS, n, buffp);
}

static S2LPStatus s2lp_spi_read_fifo(S2LPDriver *devp, uint8_t n, uint8_t *buffp) {
	return s2lp_spi_read_regs(devp, LINEAR_FIFO_ADDRESS, n, buffp);
}

static void s2lp_power_on(S2LPDriver *devp) {
	if (s2lp_check_shutdown(devp)) {
		s2lp_exit_shutdown(devp);
	} else {
		S2LPCmdStrobeReady(devp);
	}
	do {
	    S2LPRefreshStatus(devp);
	} while (g_xStatus.MC_STATE != MC_STATE_READY);
}

static void s2lp_enable_sqi(S2LPDriver *devp) {
  /* enable SQI check */
  S2LPQiSetSqiThreshold(devp, 0);
  S2LPQiSetSqiCheck(devp, S_ENABLE);
  S2LPRadioSetRssiThreshdBm(devp, devp->config->rssi_threshold);
}

#ifdef S2LP_USE_CSMA_ENGINE
static void s2lp_enable_csma(S2LPDriver *devp) {
	/* Enable CSMA */
	S2LPPacketHandlerSetRxPersistentMode(devp, S_DISABLE);
	S2LPRadioCsBlanking(devp, S_DISABLE);
	S2LPCsmaInit(devp, devp->config->csma_cfg);
	S2LPCsma(devp, S_ENABLE);
	S2LPRadioSetRssiThreshdBm(devp, devp->config->csma_rssi_threshold);
}
#endif

static void s2lp_serve_global_interrupts(void *arg) {

  S2LPDriver *devp = ((S2LPDriver *) arg);

	chSysLockFromISR();

	chEvtSignalI(devp->txrx_work, EVT_GLOBAL_IRQ_PENDING);

	chSysUnlockFromISR();
}

static void s2lp_serve_fifo_interrupts(void *arg) {

  S2LPDriver *devp = ((S2LPDriver *) arg);

	chSysLockFromISR();

	chEvtSignalI(devp->txrx_work, EVT_FIFO_IRQ_PENDING);

	chSysUnlockFromISR();
}

static void s2lp_serve_events(S2LPDriver *devp) {
  /* Events from S2LP irqs to serve */
  static eventmask_t expected_events = EVT_FIFO_IRQ_PENDING   |
                                       EVT_GLOBAL_IRQ_PENDING |
                                       EVT_STOP_TXRX;
  static eventmask_t received_events;

  /* error message */
  msg_t err = MSG_OK;

  /* rx / tx done */
  bool done = false;

  /* tx ongoing */
  bool tx_ongoing = false;

  /* rx ongoing */
  bool rx_ongoing = false;

  /* Start RX / TX */
  switch (devp->state) {
    case S2LP_TX:
      S2LPCmdStrobeTx(devp);
      tx_ongoing = true;
      break;

    case S2LP_RX:
      S2LPCmdStrobeRx(devp);
      break;

    default:
      break;
  }

  while (!done) {
    /* Pending irqs */
    S2LPIrqs pending_irqs = {0};

    /* Wait for an irq to serve */
    received_events = chEvtWaitAny(expected_events);

    if (received_events & EVT_GLOBAL_IRQ_PENDING) {
      /* read pending irqs */
      S2LPGpioIrqGetStatus(devp, &pending_irqs);
    }

    if (received_events & EVT_FIFO_IRQ_PENDING) {
      /* RX FIFO almost full irq  */
      if (devp->state == S2LP_RX) { pending_irqs.IRQ_RX_FIFO_ALMOST_FULL  = 1; }
      /* TX FIFO almost empty irq */
      if (devp->state == S2LP_TX) { pending_irqs.IRQ_TX_FIFO_ALMOST_EMPTY = 1; }
    }

    if (received_events & EVT_STOP_TXRX) {
      err |= MSG_STOPPED;
    }

    /* Now, we are going to treat irqs in priority order */

    /* Remaining payload bytes */
    uint16_t payload_rem = devp->packet->size - devp->payload_head;

    /* TX.1 -> Fill TX FIFO */
    if (pending_irqs.IRQ_TX_FIFO_ALMOST_EMPTY && tx_ongoing) {
      /* TX FIFO available bytes */
      uint8_t nb = S2LP_FIFO_SIZE - S2LPFifoReadNumberBytesTxFifo(devp);

      if (payload_rem <= nb) {
        nb = payload_rem;
      }

      if (nb > 0) {
        palDisableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line);

        (void) s2lp_spi_write_fifo(devp, nb, devp->packet->payload + devp->payload_head);

        palSetLineCallback(devp->config->gpio_cfg->mcu_fifo_irq_line, s2lp_serve_fifo_interrupts, (void *) devp);
        palEnableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line, PAL_EVENT_MODE_RISING_EDGE);

        devp->payload_head += nb;
      }
    }

    /* TX.2 -> transmission succeed */
    if (pending_irqs.IRQ_TX_DATA_SENT && tx_ongoing) {
      err  = MSG_OK;
      tx_ongoing = false;
      done = true;
    }

    /* TX.3 -> CCA failure */
    if (pending_irqs.IRQ_MAX_BO_CCA_REACH && tx_ongoing) {
      err |= MSG_MAX_BO_CCA_REACH;
    }

    /* TX.4 -> FIFO over/ under flow */
    if (pending_irqs.IRQ_TX_FIFO_ERROR && tx_ongoing) {
      err |= MSG_TX_FIFO_ERROR;
    }

    /* RX.1 -> reception started */
    if (pending_irqs.IRQ_VALID_SYNC) {
      rx_ongoing = true;
    }

    /* RX.2 -> Read RX FIFO */
    if (pending_irqs.IRQ_RX_FIFO_ALMOST_FULL && rx_ongoing) {

      /* RX FIFO available bytes */
      uint8_t nb = S2LPFifoReadNumberBytesRxFifo(devp);

      if (nb > 0) {
        /* There are bytes to read */
        if (payload_rem >= nb) {
          /* Receiving packet does not exceed payload size */
          palDisableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line);

          (void) s2lp_spi_read_fifo(devp, nb, devp->packet->payload + devp->payload_head);

          palSetLineCallback(devp->config->gpio_cfg->mcu_fifo_irq_line, s2lp_serve_fifo_interrupts, (void *) devp);
          palEnableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line, PAL_EVENT_MODE_RISING_EDGE);

          devp->payload_head += nb;

        } else {
          /* Receiving packet exceeds payload size, discarding */
          pending_irqs.IRQ_RX_DATA_DISC = 1;
        }
      }
    }

    /* RX.3 -> reception succeed */
    if (pending_irqs.IRQ_RX_DATA_READY && rx_ongoing) {
      err = MSG_OK;
      rx_ongoing = false;
      done = true;
    }

    /* RX.4 -> RX FIFO under/over flow */
    if (pending_irqs.IRQ_RX_FIFO_ERROR && rx_ongoing) {
      err |= MSG_RX_FIFO_ERROR;
    }

    /* RX.5 -> received data CRC error */
    if (pending_irqs.IRQ_CRC_ERROR && rx_ongoing) {
      err |= MSG_CRC_ERROR;
    }

    /* RX.6 -> received data discarded */
    if (pending_irqs.IRQ_RX_DATA_DISC && rx_ongoing) {
      err |= MSG_RX_DATA_DISC;
    }

    /* RX.7 -> RX time out reached */
    if (pending_irqs.IRQ_RX_TIMEOUT) {
      err = MSG_RX_TIMEOUT;
    }

    /* Stop ongoing command & exit on error */
    if (err != MSG_OK) {
      S2LPCmdStrobeSabort(devp);
      done = true;
    }
  }

  /* TX / RX done */
  devp->packet->error = err;

  /* Clear events */
  (void) chEvtGetAndClearEvents(expected_events);
}

static THD_FUNCTION(TxRxWorker, arg) {
  S2LPDriver *devp = ((S2LPDriver *) arg);

  while (true) {
    /* Wait until get message MSG_START_TXRX or MSG_TERMINATE */
    thread_reference_t thd = chMsgWait();
    msg_t msg = chMsgGet(thd);
    chMsgRelease(thd, MSG_OK);

    if (msg == MSG_START_TXRX) {
      /* irqs configuration */
      S2LPIrqs irqs_cfg = {0};

      /* Go to ready state */
      S2LPCmdStrobeReady(devp);
      chThdSleepMicroseconds(100);
      do {
        S2LPRefreshStatus(devp);
      } while (g_xStatus.MC_STATE != MC_STATE_READY);

      switch (devp->state) {
        case S2LP_TX:
          /* Flush the TX FIFO */
          S2LPCmdStrobeFlushTxFifo(devp);

          /* First fill TX FIFO */
          if (devp->packet->size > S2LP_FIFO_SIZE) {
            (void) s2lp_spi_write_fifo(devp, S2LP_FIFO_SIZE, devp->packet->payload);
            devp->payload_head = S2LP_FIFO_SIZE;
          } else {
            (void) s2lp_spi_write_fifo(devp, devp->packet->size, devp->packet->payload);
            devp->payload_head = devp->packet->size;
          }

          /* TX irqs configuration */
          irqs_cfg.IRQ_MAX_BO_CCA_REACH  = 1;
          irqs_cfg.IRQ_TX_FIFO_ERROR     = 1;
          irqs_cfg.IRQ_TX_DATA_SENT      = 1;
          break;

        case S2LP_RX:
          /* Flush the RX FIFO */
          S2LPCmdStrobeFlushRxFifo(devp);

          /* RX irqs configuration */
          irqs_cfg.IRQ_VALID_SYNC    = 1;
          irqs_cfg.IRQ_RX_TIMEOUT    = 1;
          irqs_cfg.IRQ_RX_FIFO_ERROR = 1;
          irqs_cfg.IRQ_RX_DATA_READY = 1;
          irqs_cfg.IRQ_CRC_ERROR     = 1;
          irqs_cfg.IRQ_RX_DATA_DISC  = 1;
          break;

        default:
          break;
      }

      /* Start s2lp interrupts */
      S2LPGpioIrqInit(devp, &irqs_cfg);

      /* IRQ registers blanking */
      S2LPGpioIrqClearStatus(devp);

      /* Register irq callback functions */
      palSetLineCallback(devp->config->gpio_cfg->mcu_irq_line, s2lp_serve_global_interrupts, (void *) devp);
      palSetLineCallback(devp->config->gpio_cfg->mcu_fifo_irq_line, s2lp_serve_fifo_interrupts, (void *) devp);

      /* Enable mcu irqs */
      palEnableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line, PAL_EVENT_MODE_RISING_EDGE);
      palEnableLineEvent(devp->config->gpio_cfg->mcu_irq_line, PAL_EVENT_MODE_FALLING_EDGE);

      s2lp_serve_events(devp);

      /* disable mcu irqs */
      palDisableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line);
      palDisableLineEvent(devp->config->gpio_cfg->mcu_irq_line);

      if (devp->state == S2LP_RX) {
        /* Read RX FIFO residue */
        uint8_t nb = S2LPFifoReadNumberBytesRxFifo(devp);
        if (nb > 0) {
          (void) s2lp_spi_read_fifo(devp, nb, devp->packet->payload + devp->payload_head);
        }

        /* Update received packet size */
#if S2LP_USE_BASIC_PROTOCOL
        devp->packet->size = S2LPPktBasicGetReceivedPktLength(devp);
#endif
      }

      /* Disable CSMA */
      if (S2LPCsmaGetCsma(devp) == S_ENABLE) { S2LPCsma(devp, S_DISABLE); }

      /* Configure Sleep Mode with no FIFO retention */
      S2LPTimerSleepB(devp, S_DISABLE);

      /* Go to sleep state with no fifo retention */
      S2LPCmdStrobeSleep(devp);
      chThdSleepMicroseconds(3);
      do {
        S2LPRefreshStatus(devp);
      } while (g_xStatus.MC_STATE != MC_STATE_SLEEP_NOFIFO);

      /* finalize */
      devp->finalize(devp);
    }

    /* Exit message received */
    if (msg == MSG_TERMINATE) {
      chThdExit(MSG_OK);
    }

  }

}

static void s2lp_wait_finalize(S2LPDriver *devp) {
  /* RX/TX result */
  msg_t msg = (devp->packet->error == MSG_OK) ? MSG_OK : MSG_RESET;
  chMsgSend(devp->thread, msg);
}

static void s2lp_cb_finalize(S2LPDriver *devp) {
  switch (devp->state) {
    case S2LP_TX:
      devp->state = S2LP_READY_TX;
      devp->config->tx_end_cb(devp);
      break;

    case S2LP_RX:
      devp->state = S2LP_READY_RX;
      devp->config->rx_end_cb(devp);
      break;

    default:
      break;
  }
}

static void s2lp_send_initialize(S2LPDriver *devp, s2lp_packet_t *txp){
  /* Initialize payload */
  devp->payload_head = 0;
  devp->packet       = txp;

  if (devp->state != S2LP_READY_TX) {
    /* configure FIFO irq line */
    devp->config->gpio_cfg->fifo_irq_line_cfg->xS2LPGpioIO = S2LP_GPIO_DIG_OUT_TXRX_FIFO_ALMOST_EMPTY;
    S2LPGpioInit(devp, devp->config->gpio_cfg->fifo_irq_line_cfg);
    S2LPFifoMuxRxFifoIrqEnable(devp, S_DISABLE);

#ifdef S2LP_USE_CSMA_ENGINE
    s2lp_enable_csma(devp);
#endif
  }

#if S2LP_USE_BASIC_PROTOCOL
  /* set payload length */
  S2LPPktBasicSetPayloadLength(devp, devp->packet->size);

  /* set destination address */
  S2LPSetRxSourceReferenceAddress(devp, devp->packet->address);
#endif

  /* Send message to txrx_work to start transmission */
  chMsgSend(devp->txrx_work, MSG_START_TXRX);
}

static msg_t s2lp_send(void *ip, s2lp_packet_t *txp) {
	S2LPDriver* devp;

	chDbgCheck(ip != NULL);

	/* Getting parent instance pointer.*/
	devp = objGetInstance(S2LPDriver*, (BaseRadio*)ip);
	chDbgAssert((devp->state == S2LP_READY)	    ||
				      (devp->state == S2LP_READY_TX)	||
				      (devp->state == S2LP_READY_RX),
				       "s2lpSend(), invalid state");

  devp->state = S2LP_TX;

  /* Set blocking send finalize */
  devp->finalize = s2lp_wait_finalize;

  /* Get caller thread reference */
  devp->thread = chThdGetSelfX();

  s2lp_send_initialize(devp, txp);

  /* Wait for send to complete */
  thread_reference_t thd = chMsgWait();
  msg_t tx_msg = chMsgGet(thd);
  chMsgRelease(thd, MSG_OK);

  devp->state = S2LP_READY_TX;

  return tx_msg;
}

static void s2lp_start_send(void *ip, s2lp_packet_t *txp) {
  S2LPDriver* devp;

  chDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(S2LPDriver*, (BaseRadio*)ip);

  chDbgAssert((devp->state == S2LP_READY)     ||
              (devp->state == S2LP_READY_TX)  ||
              (devp->state == S2LP_READY_RX),
              "s2lpStartSend(), invalid state");

  chDbgAssert((devp->config->tx_end_cb != NULL),
              "s2lpStartSend(), TX callback is NULL");

  devp->state = S2LP_TX;

  /* Set non blocking send finalize */
  devp->finalize = s2lp_cb_finalize;

  s2lp_send_initialize(devp, txp);
}

static void s2lp_stop_send(void *ip) {
  S2LPDriver* devp;

  chDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(S2LPDriver*, (BaseRadio*)ip);

  chDbgAssert((devp->state == S2LP_TX),
              "s2lpStopSend(), invalid state");

  chEvtSignal(devp->txrx_work, EVT_STOP_TXRX);
}

static void s2lp_receive_initialize(S2LPDriver *devp, s2lp_packet_t *rxp, sysinterval_t tout) {
  /* Initialize payload */
  devp->payload_head = 0;
  devp->packet       = rxp;
  devp->packet->size = S2LP_MAX_PACKET_SIZE; // max size available for payload to receive

  if (devp->state != S2LP_READY_RX) {
    /* configure FIFO irq line */
    devp->config->gpio_cfg->fifo_irq_line_cfg->xS2LPGpioIO = S2LP_GPIO_DIG_OUT_TXRX_FIFO_ALMOST_FULL;
    S2LPGpioInit(devp, devp->config->gpio_cfg->fifo_irq_line_cfg);
    S2LPFifoMuxRxFifoIrqEnable(devp, S_ENABLE);

#ifdef S2LP_USE_CSMA_ENGINE
    /* disable csma */
    S2LPCsma(devp, S_DISABLE);
#endif
    /* enable cs blanking */
    S2LPRadioCsBlanking(devp, S_ENABLE);
  }

  /* set rx timeout */
  switch (tout) {
    case TIME_INFINITE:
      /* Infinite rx timeout */
      SET_INFINITE_RX_TIMEOUT(devp);
      S2LPTimerSetRxTimerStopCondition(devp, ANY_ABOVE_THRESHOLD);
      /* Set sniff timer expiration time (us) */
      S2LpSetTimerFastRxTermTimerUs(devp, 1000000 *
                       devp->config->pkt_basic_cfg->xPreambleLength /
                       devp->config->radio_cfg->lDatarate);
      S2LpTimerFastRxTermTimer(devp, S_ENABLE);
      break;

    default:
      /* RX timeout configuration */
      if (tout == TIME_IMMEDIATE) { tout = 1; }
      S2LPTimerSetRxTimerMs(devp, TIME_I2MS(tout));
      S2LPTimerSetRxTimerStopCondition(devp, RSSI_AND_SQI_ABOVE_THRESHOLD);
      break;
  }

  /* enable SQI */
  s2lp_enable_sqi(devp);

  /* Send message to rx_work to start receive */
  chMsgSend(devp->txrx_work, MSG_START_TXRX);
}

static msg_t s2lp_receive_timeout(void *ip, s2lp_packet_t *rxp, sysinterval_t tout) {
  S2LPDriver* devp;

  msg_t rx_msg;

  chDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(S2LPDriver*, (BaseRadio*)ip);

	chDbgAssert((devp->state == S2LP_READY)		 ||
			  	      (devp->state == S2LP_READY_TX) ||
				        (devp->state == S2LP_READY_RX),
		            "s2lpReceive(), invalid state");

  devp->state = S2LP_RX;

  /* Set blocking receive finalize */
  devp->finalize = s2lp_wait_finalize;

  /* Get caller thread reference */
  devp->thread = chThdGetSelfX();

  /* Initialize reception */
  s2lp_receive_initialize(devp, rxp, tout);

  /* Wait for reception to terminate */
	thread_reference_t thd = chMsgWait();
	rx_msg = chMsgGet(thd);
	chMsgRelease(thd, MSG_OK);

	devp->state = S2LP_READY_RX;

	return rx_msg;
}

static void s2lp_start_receive_timeout(void *ip, s2lp_packet_t *rxp, sysinterval_t tout) {
  S2LPDriver* devp;

  chDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(S2LPDriver*, (BaseRadio*)ip);

  chDbgAssert((devp->state == S2LP_READY)    ||
              (devp->state == S2LP_READY_TX) ||
              (devp->state == S2LP_READY_RX),
              "s2lpStartReceive(), invalid state");

  chDbgAssert((devp->config->rx_end_cb != NULL),
              "s2lpStartReceive(), RX callback is NULL");

  devp->state = S2LP_RX;

  /* Set blocking receive finalize */
  devp->finalize = s2lp_cb_finalize;

  /* Initialize reception */
  s2lp_receive_initialize(devp, rxp, tout);
}

static void s2lp_stop_receive(void *ip) {
  S2LPDriver* devp;

  chDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(S2LPDriver*, (BaseRadio*)ip);

  chDbgAssert((devp->state == S2LP_RX),
              "s2lpStopReceive(), invalid state");

  chEvtSignal(devp->txrx_work, EVT_STOP_TXRX);
}

static const struct S2LPVMT vmt_device = {
  (size_t)0
};

static const struct BaseRadioVMT vmt_radio = {
  sizeof(struct S2LPVMT*),
  s2lp_send,
  s2lp_start_send,
  s2lp_stop_send,
  s2lp_receive_timeout,
  s2lp_start_receive_timeout,
  s2lp_stop_receive
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void s2lpObjectInit(S2LPDriver *devp) {
	devp->vmt = &vmt_device;
	devp->radio_if.vmt = &vmt_radio;

	devp->config = NULL;

	devp->thread = NULL;
	devp->txrx_work = NULL;

	devp->finalize = NULL;

	devp->packet = NULL;

	chMtxObjectInit(&devp->lock);

	devp->state = S2LP_STOP;
}

void s2lpStart(S2LPDriver *devp, S2LPConfig *config) {

  chDbgCheck((devp != NULL) && (config != NULL));

	chDbgAssert((devp->state == S2LP_STOP) ||
				      (devp->state == S2LP_READY),
	            "s2lpStart(), invalid state");

	/* Create RX/TX worker thread */
	devp->txrx_work = chThdCreateFromHeap(NULL,
	                                      THD_WORKING_AREA_SIZE(128),
	                                      "s2lpTxRxWork",
	                                      S2LP_WORK_PRIORITY,
	                                      TxRxWorker, (void *) devp);

	devp->config = config;

	chDbgCheck((devp->config->spip   != NULL) &&
			     (devp->config->spi_cfg != NULL));

	/* default configurations */
	if (devp->config->gpio_cfg->irq_line_cfg == NULL) {
		devp->config->gpio_cfg->irq_line_cfg = &irq_line_cfg_default;
	}
	if (devp->config->gpio_cfg->fifo_irq_line_cfg == NULL) {
		devp->config->gpio_cfg->fifo_irq_line_cfg = &fifo_irq_line_cfg_default;
	}
	if (devp->config->radio_cfg == NULL) {
		devp->config->radio_cfg = &radio_cfg_default;
	}
#if S2LP_USE_BASIC_PROTOCOL
	if (devp->config->pkt_basic_cfg == NULL) {
		devp->config->pkt_basic_cfg = &pkt_basic_cfg_default;
	}
	if (devp->config->pkt_basic_addr_cfg == NULL) {
		devp->config->pkt_basic_addr_cfg = &pkt_basic_addr_cfg_default;
	}
#endif
#if S2LP_USE_CSMA_ENGINE
	if (devp->config->csma_cfg == NULL) {
		devp->config->csma_cfg = &csma_cfg_default;
	}
#endif

	/** initialize MCU interface : SPI, GPIO  */
	/** everything initialized outside driver */

	/** power on radio */
	s2lp_power_on(devp);

  /* Wait for ready state */
  chThdSleepMicroseconds(100);
  do {
    S2LPRefreshStatus(devp);
  } while (g_xStatus.MC_STATE != MC_STATE_READY);

	/** set the oscillator frequency */
	S2LPRadioSetXtalFrequency(devp->config->xtal_freq);

	/** initialize radio parameters   */
	/** enable automatic calibrations */
	S2LPTimerCalibrationRco(devp, S_ENABLE);
	S2LPRadioCalibrationVco(devp, S_ENABLE, S_ENABLE);

	/* #TODO: AFC AGC */

	/* #TODO: PLL */

	/* configure irq gpio */
	S2LPGpioInit(devp, devp->config->gpio_cfg->irq_line_cfg);

	/** configure frequencies */
	S2LPRadioInit(devp, devp->config->radio_cfg);

	/* configure power */
	S2LPRadioSetPALeveldBm(devp, devp->config->pwr_idx, devp->config->pwr_dbm);
	S2LPRadioSetPALevelMaxIndex(devp, devp->config->pwr_idx);

#if S2LP_USE_BASIC_PROTOCOL

	/* basic packet protocol configuration */
	S2LPPktBasicInit(devp, devp->config->pkt_basic_cfg);

	/* configure preamble pattern to be compliant with Spirit1 */
	S2LPPacketHandlerSwapPreamblePattern(devp, S_DISABLE);

	/* configure addresses */
	S2LPPktBasicAddressesInit(devp, devp->config->pkt_basic_addr_cfg);

#endif

	/* configure FIFO thresholds */
	S2LPFifoSetAlmostEmptyThresholdTx(devp, S2LP_TX_FIFO_AE_THRESHOLD);
	S2LPFifoSetAlmostFullThresholdRx(devp, S2LP_RX_FIFO_AF_THRESHOLD);

  /* Configure Sleep Mode with no FIFO retention */
  S2LPTimerSleepB(devp, S_DISABLE);

  /* Go to sleep state */
  S2LPCmdStrobeSleep(devp);
  chThdSleepMicroseconds(3);
  do {
    S2LPRefreshStatus(devp);
  } while (g_xStatus.MC_STATE != MC_STATE_SLEEP_NOFIFO);

	devp->state = S2LP_READY;
}

void s2lpStop(S2LPDriver *devp) {

	chDbgCheck(devp != NULL);

	chDbgAssert((devp->state == S2LP_STOP) ||
				      (devp->state == S2LP_READY) ||
				      (devp->state == S2LP_READY_RX) ||
				      (devp->state == S2LP_READY_TX),
	            "s2lpStop(), invalid state");

	if (devp->state == S2LP_READY) {
		spiStop(devp->config->spip);
	}

	s2lp_enter_shutdown(devp);

	/* Terminates RX/TX thread */
  chMsgSend(devp->txrx_work, MSG_TERMINATE);
  chThdWait(devp->txrx_work);

	devp->state = S2LP_STOP;
}

void s2lpAquire(S2LPDriver *devp) {
  chDbgCheck(devp != NULL);

  chMtxLock(&devp->lock);
}

void s2lpRelease(S2LPDriver *devp) {
  chDbgCheck(devp != NULL);

  chMtxUnlock(&devp->lock);
}
/** @} */
