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
 * @file    s2lp.c
 * @brief   S2LP radio interface module code.
 *
 * @addtogroup S2LP
 * @{
 */

#include "hal.h"
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
		.lFrequencyBase		= 868.0e6,
		.xModulationSelect	= MOD_2GFSK_BT1,
		.lDatarate			= 38500,
		.lFreqDev			= 20e3,
		.lBandwidth			= 100e3
};

#if S2LP_USE_BASIC_PROTOCOL
static PktBasicInit pkt_basic_cfg_default = {
		.xPreambleLength		= 32,
		.xSyncLength			= 32,
		.lSyncWords				= 0x88888888,
		.xFixVarLength			= S_ENABLE,
		.cExtendedPktLenField	= S_ENABLE,
		.xCrcMode				= PKT_CRC_MODE_8BITS,
		.xAddressField			= S_ENABLE,
		.xFec					= S_DISABLE,
		.xDataWhitening			= S_ENABLE
};

static PktBasicAddressesInit pkt_basic_addr_cfg_default = {
		.xFilterOnMyAddress 		= S_ENABLE,
		.cMyAddress					= 0x44,
		.xFilterOnMulticastAddress	= S_ENABLE,
		.cMulticastAddress			= 0xEE,
		.xFilterOnBroadcastAddress	= S_ENABLE,
		.cBroadcastAddress			= 0xFF
};
#endif

#if S2LP_USE_CSMA_ENGINE
static SCsmaInit csma_cfg_default = {
		.xCsmaPersistentMode	= S_DISABLE,
		.xMultiplierTbit		= CSMA_PERIOD_64TBIT,
		.xCcaLength				= 3,
		.cMaxNb					= 5,
		.nBuCounterSeed			= 0xFA21,
		.cBuPrescaler			= 32
};
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static S2LPDriver *s2lp_ext_devp = NULL;

typedef enum {
	TX_START,
	TX_STOP,
	TX_FILL_FIFO,
	TX_WAIT_EVENT,
	TX_SERVE_EVENT
} s2lp_tx_state_t;

typedef enum {
	RX_START,
	RX_STOP,
	RX_READ_FIFO,
	RX_WAIT_EVENT,
	RX_SERVE_EVENT
} s2lp_rx_state_t;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

void s2lp_enter_shutdown(S2LPDriver *devp) {
	palSetLine(devp->config->gpio_cfg->sdn_line);
	osalThreadSleep(OSAL_MS2I(1));
}

void s2lp_exit_shutdown(S2LPDriver *devp) {
	palClearLine(devp->config->gpio_cfg->sdn_line);
	osalThreadSleep(OSAL_MS2I(1));
}

SFlagStatus s2lp_check_shutdown(S2LPDriver *devp) {
	return (SFlagStatus) palReadLine(devp->config->gpio_cfg->sdn_line);
}

S2LPStatus s2lp_spi_write_header(S2LPDriver *devp, uint8_t reg_type, uint8_t reg) {
	uint8_t rxFrame[2];
	S2LPStatus* tmpStatus = (S2LPStatus*) rxFrame;
	osalSysLock();
	rxFrame[1] = spiPolledExchange(devp->config->spip, reg_type);
	rxFrame[0] = spiPolledExchange(devp->config->spip, reg);
	osalSysUnlock();
	return *tmpStatus;
}

S2LPStatus s2lp_spi_write_regs(uint8_t addr, uint8_t n, uint8_t *regsp) {
	S2LPStatus status;
#if (S2LP_SHARED_SPI)
	spiAcquireBus(s2lp_ext_devp->config->spip);
#endif
	spiStart(s2lp_ext_devp->config->spip, s2lp_ext_devp->config->spi_cfg);
	spiSelect(s2lp_ext_devp->config->spip);
	status = s2lp_spi_write_header(s2lp_ext_devp, WRITE_HEADER, addr);
	spiSend(s2lp_ext_devp->config->spip, n, regsp);
	spiUnselect(s2lp_ext_devp->config->spip);
#if (S2LP_SHARED_SPI)
	spiReleaseBus(s2lp_ext_devp->config->spip);
#endif
	return status;
}

S2LPStatus s2lp_spi_read_regs(uint8_t addr, uint8_t n, uint8_t *regsp) {
	S2LPStatus status;
#if (S2LP_SHARED_SPI)
	spiAcquireBus(s2lp_ext_devp->config->spip);
#endif
	spiStart(s2lp_ext_devp->config->spip, s2lp_ext_devp->config->spi_cfg);
	spiSelect(s2lp_ext_devp->config->spip);
	status = s2lp_spi_write_header(s2lp_ext_devp, READ_HEADER, addr);
	spiReceive(s2lp_ext_devp->config->spip, n, regsp);
	spiUnselect(s2lp_ext_devp->config->spip);
#if (S2LP_SHARED_SPI)
	spiReleaseBus(s2lp_ext_devp->config->spip);
#endif
	return status;
}

S2LPStatus s2lp_spi_cmd_strobes(uint8_t cmd) {
	S2LPStatus status;
#if (S2LP_SHARED_SPI)
	spiAcquireBus(s2lp_ext_devp->config->spip);
#endif
	spiStart(s2lp_ext_devp->config->spip, s2lp_ext_devp->config->spi_cfg);
	spiSelect(s2lp_ext_devp->config->spip);
	status = s2lp_spi_write_header(s2lp_ext_devp, COMMAND_HEADER, cmd);
	spiUnselect(s2lp_ext_devp->config->spip);
#if (S2LP_SHARED_SPI)
	spiReleaseBus(s2lp_ext_devp->config->spip);
#endif
	return status;
}

S2LPStatus s2lp_spi_write_fifo(uint8_t n, uint8_t *buffp) {
	return s2lp_spi_write_regs(LINEAR_FIFO_ADDRESS, n, buffp);
}

S2LPStatus s2lp_spi_read_fifo(uint8_t n, uint8_t *buffp) {
	return s2lp_spi_read_regs(LINEAR_FIFO_ADDRESS, n, buffp);
}

void s2lp_power_on(S2LPDriver *devp) {
	if (s2lp_check_shutdown(devp)) {
		s2lp_exit_shutdown(devp);
	} else {
		S2LPCmdStrobeReady();
	}
	do {
	    S2LPRefreshStatus();
	} while (g_xStatus.MC_STATE != MC_STATE_READY);
}

void s2lp_enable_sqi(S2LPDriver *devp) {
  /* enable SQI check */
  S2LPQiSetSqiThreshold(0);
  S2LPQiSetSqiCheck(S_ENABLE);
  S2LPRadioSetRssiThreshdBm(devp->config->rssi_threshold);
}

#ifdef S2LP_USE_CSMA_ENGINE
void s2lp_enable_csma(S2LPDriver *devp) {
	/* Enable CSMA */
	S2LPPacketHandlerSetRxPersistentMode(S_DISABLE);
	S2LPRadioCsBlanking(S_DISABLE);
	S2LPCsmaInit(devp->config->csma_cfg);
	S2LPCsma(S_ENABLE);
	S2LPRadioSetRssiThreshdBm(devp->config->csma_rssi_threshold);
}
#endif

void s2lp_set_rx_timeout(S2LPDriver *devp, sysinterval_t interval) {
	switch (interval) {
		case TIME_INFINITE:
		    /* infinite rx timeout */
		    SET_INFINITE_RX_TIMEOUT();
		    S2LPTimerSetRxTimerStopCondition(ANY_ABOVE_THRESHOLD);
			break;
		default:
		    /* RX timeout config */
			if (interval == TIME_IMMEDIATE) { interval = 1; }
			S2LPTimerSetRxTimerMs(1000 * interval / OSAL_ST_FREQUENCY);
		    S2LPTimerSetRxTimerStopCondition(RSSI_AND_SQI_ABOVE_THRESHOLD);
			break;
	}
}

void s2lp_serve_interrupts(void *devp ) {
	osalSysLockFromISR();
	switch (((S2LPDriver *) devp)->state) {
		case S2LP_TX:
			((S2LPDriver *) devp)->irq_pending |= TX_IRQ_PENDING;
			break;
		case S2LP_RX:
			((S2LPDriver *) devp)->irq_pending |= RX_IRQ_PENDING;
			break;
		default:
			break;
	}
	osalThreadResumeI(&(((S2LPDriver *) devp)->thread), MSG_OK);
	osalSysUnlockFromISR();
}

void s2lp_serve_fifo_interrupts(void *devp) {
	osalSysLockFromISR();
	switch (((S2LPDriver *) devp)->state) {
		case S2LP_TX:
			((S2LPDriver *) devp)->irq_pending |= TX_FIFO_AE_IRQ_PENDING;
			break;
		case S2LP_RX:
			((S2LPDriver *) devp)->irq_pending |= RX_FIFO_AF_IRQ_PENDING;
			break;
		default:
			break;
	}
	osalThreadResumeI(&(((S2LPDriver *) devp)->thread), MSG_OK);
	osalSysUnlockFromISR();
}

msg_t s2lp_transmission_handler(S2LPDriver *devp, uint16_t n, uint8_t* txp) {

	msg_t tx_msg;
	bool tx_started = false;
	s2lp_tx_state_t tx_state = TX_FILL_FIFO;
	uint16_t txp_idx = 0;	/* position in the tx fifo */
	uint8_t nbytes = 0;		/* number of bytes for a fill fifo operation */
	S2LPIrqs tx_irqs, tx_irqs_cfg = {0};

	/* flush the TX FIFO */
	S2LPCmdStrobeFlushTxFifo();

	/* start tx interrupts */
	tx_irqs_cfg.IRQ_MAX_BO_CCA_REACH	= 1;
	tx_irqs_cfg.IRQ_TX_FIFO_ERROR		= 1;
	tx_irqs_cfg.IRQ_TX_DATA_SENT		= 1;
	S2LPGpioIrqInit(&tx_irqs_cfg);

	/* IRQ registers blanking */
	S2LPGpioIrqClearStatus();

	/* enable mcu irqs */
	devp->irq_pending = 0;
	palEnableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line, PAL_EVENT_MODE_RISING_EDGE);
	palSetLineCallback(devp->config->gpio_cfg->mcu_fifo_irq_line, s2lp_serve_fifo_interrupts, (void *) devp);
	palEnableLineEvent(devp->config->gpio_cfg->mcu_irq_line, PAL_EVENT_MODE_FALLING_EDGE);
	palSetLineCallback(devp->config->gpio_cfg->mcu_irq_line, s2lp_serve_interrupts, (void *) devp);

	/* start send */
	while (tx_state != TX_STOP) {
		switch (tx_state) {
			case TX_FILL_FIFO:
				/* fifo available bytes */
				palDisableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line);
				nbytes = S2LP_FIFO_SIZE
							- S2LPFifoReadNumberBytesTxFifo();
				if ((n - txp_idx) <= nbytes) {
					nbytes = n - txp_idx;
				}
				(void) s2lp_spi_write_fifo(nbytes, txp + txp_idx);
				txp_idx += nbytes;
				if (txp_idx < n) {
					palEnableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line, PAL_EVENT_MODE_RISING_EDGE);
					palSetLineCallback(devp->config->gpio_cfg->mcu_fifo_irq_line, s2lp_serve_fifo_interrupts, (void *) devp);
				} else {
					tx_irqs_cfg.IRQ_TX_FIFO_ERROR		= 0;
					S2LPGpioIrqInit(&tx_irqs_cfg);
				}
				if (tx_started) {
					tx_state = TX_WAIT_EVENT;
				} else {
					tx_state = TX_START;
				}
				break;
			case TX_WAIT_EVENT:
				osalSysLock();
				(void) osalThreadSuspendTimeoutS(&(devp->thread), OSAL_MS2I(100));
				osalOsRescheduleS();
				osalSysUnlock();
				if (devp->irq_pending & TX_IRQ_PENDING) {
					devp->irq_pending &= !TX_IRQ_PENDING;
					tx_state = TX_SERVE_EVENT;
				} else if (devp->irq_pending & TX_FIFO_AE_IRQ_PENDING) {
					devp->irq_pending &= !TX_FIFO_AE_IRQ_PENDING;
					tx_state = TX_FILL_FIFO;
				}
				break;
			case TX_SERVE_EVENT:
				/* get IRQs status */
				S2LPGpioIrqGetStatus(&tx_irqs);
				if (tx_irqs.IRQ_TX_DATA_SENT) {
					tx_state = TX_STOP;
					tx_msg = MSG_OK;
					break;
				}
				if (tx_irqs.IRQ_TX_FIFO_ERROR) {
					tx_state = TX_STOP;
					/* flush the TX FIFO */
					S2LPCmdStrobeFlushTxFifo();
					tx_msg = MSG_TX_FIFO_ERROR;
					break;
				}
				if (tx_irqs.IRQ_MAX_BO_CCA_REACH) {
					tx_state = TX_STOP;
					tx_msg = MSG_MAX_BO_CCA_REACH;
					break;
				}
				tx_state = TX_STOP;
				tx_msg = MSG_RESET;
				break;
			case TX_START:
				tx_started = true;
				S2LPCmdStrobeTx();
				tx_state = TX_WAIT_EVENT;
				break;
			default:
				break;
		}
	}

	/* disable mcu irqs */
	palDisableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line);
	palSetLineCallback(devp->config->gpio_cfg->mcu_fifo_irq_line, NULL, NULL);
	palDisableLineEvent(devp->config->gpio_cfg->mcu_irq_line);
	palSetLineCallback(devp->config->gpio_cfg->mcu_irq_line, NULL, NULL);

	return tx_msg;

}

msg_t s2lp_reception_handler(S2LPDriver *devp, uint8_t* rxp) {

	msg_t rx_msg;
	uint16_t rxp_idx = 0;
	uint8_t nbytes = 0;
	s2lp_rx_state_t rx_state = RX_START;
	S2LPIrqs rx_irqs, rx_irqs_cfg = {0};
	bool rx_complete = false;

		/* flush the RX FIFO */
		S2LPCmdStrobeFlushRxFifo();

		/* enable rx_irqs */
		rx_irqs_cfg.IRQ_VALID_SYNC		= 1;
		rx_irqs_cfg.IRQ_RX_TIMEOUT		= 1;
		S2LPGpioIrqInit(&rx_irqs_cfg);

		/* IRQ registers blanking */
		S2LPGpioIrqClearStatus();

		/* enable mcu irq */
		devp->irq_pending = 0;
		palEnableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line, PAL_EVENT_MODE_RISING_EDGE);
		palSetLineCallback(devp->config->gpio_cfg->mcu_fifo_irq_line, s2lp_serve_fifo_interrupts, (void *) devp);
		palEnableLineEvent(devp->config->gpio_cfg->mcu_irq_line, PAL_EVENT_MODE_FALLING_EDGE);
		palSetLineCallback(devp->config->gpio_cfg->mcu_irq_line, s2lp_serve_interrupts, (void *) devp);

		while (rx_state != RX_STOP) {
			switch (rx_state) {
				case RX_START:
					S2LPCmdStrobeRx();
					rx_state = RX_WAIT_EVENT;
					break;
				case RX_READ_FIFO:
					palDisableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line);
					nbytes = S2LPFifoReadNumberBytesRxFifo();
					if (nbytes > 0) {
						(void) s2lp_spi_read_fifo(nbytes, rxp + rxp_idx);
					}
					palEnableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line, PAL_EVENT_MODE_RISING_EDGE);
					palSetLineCallback(devp->config->gpio_cfg->mcu_fifo_irq_line, s2lp_serve_fifo_interrupts, (void *) devp);
					rxp_idx += nbytes;
					if (rx_complete) {
						rx_state = RX_STOP;
					} else {
						rx_state = RX_WAIT_EVENT;
					}
					break;
				case RX_WAIT_EVENT:
					osalSysLock();
					(void) osalThreadSuspendTimeoutS(&(devp->thread), OSAL_MS2I(100));
					osalOsRescheduleS();
					osalSysUnlock();
					if (devp->irq_pending & RX_IRQ_PENDING) {
						devp->irq_pending &= !RX_IRQ_PENDING;
						rx_state = RX_SERVE_EVENT;
					} else if (devp->irq_pending & RX_FIFO_AF_IRQ_PENDING) {
						devp->irq_pending &= !RX_FIFO_AF_IRQ_PENDING;
						rx_state = RX_READ_FIFO;
					}
					break;
				case RX_SERVE_EVENT:
					// get IRQs status
					(void) S2LPGpioIrqGetStatus(&rx_irqs);
					if (rx_irqs.IRQ_VALID_SYNC) {
						/* a packet is being received */
						rx_irqs_cfg.IRQ_RX_FIFO_ERROR	= 1;
						rx_irqs_cfg.IRQ_RX_DATA_READY	= 1;
						rx_irqs_cfg.IRQ_CRC_ERROR		= 1;
						rx_irqs_cfg.IRQ_RX_DATA_DISC	= 1;
						rx_irqs_cfg.IRQ_VALID_SYNC		= 0;
						S2LPGpioIrqInit(&rx_irqs_cfg);
						rx_state = RX_WAIT_EVENT;
						break;
					}
					if (rx_irqs.IRQ_RX_DATA_READY) {
						rx_complete = true;
						rx_state = RX_READ_FIFO;
						rx_msg = MSG_OK;
						break;
					}
					if (rx_irqs.IRQ_RX_TIMEOUT) {
						rx_state = RX_STOP;
						rx_msg = MSG_RX_TIMEOUT;
						break;
					}
					if (rx_irqs.IRQ_CRC_ERROR) {
						rx_state = RX_STOP;
						rx_msg = MSG_CRC_ERROR;
						break;
					}
					if (rx_irqs.IRQ_RX_FIFO_ERROR) {
						rx_state = RX_STOP;
						/* flush the RX FIFO */
						S2LPCmdStrobeFlushRxFifo();
						rx_msg = MSG_RX_FIFO_ERROR;
						break;
					}
					if (rx_irqs.IRQ_RX_DATA_DISC) {
						rx_state = RX_STOP;
						rx_msg = MSG_RX_DATA_DISC;
						break;
					}
					rx_state = RX_STOP;
					/* return to ready sate */
					S2LPCmdStrobeSabort();
					rx_msg = MSG_RESET;
					break;
				default:
					break;
			}
		}

		/* disable mcu irq */
		palDisableLineEvent(devp->config->gpio_cfg->mcu_fifo_irq_line);
		palSetLineCallback(devp->config->gpio_cfg->mcu_fifo_irq_line, NULL, NULL);
		palDisableLineEvent(devp->config->gpio_cfg->mcu_irq_line);
		palSetLineCallback(devp->config->gpio_cfg->mcu_irq_line, NULL, NULL);

	return rx_msg;
}

msg_t s2lp_send(void *ip, uint8_t raddr, uint16_t n, uint8_t* txp) {
	S2LPDriver* devp;
	msg_t tx_msg;

	osalDbgCheck(ip != NULL);

	/* Getting parent instance pointer.*/
	devp = objGetInstance(S2LPDriver*, (BaseRadio*)ip);
	osalDbgAssert((devp->state == S2LP_READY)	||
				  (devp->state == S2LP_READY_TX)	||
				  (devp->state == S2LP_READY_RX),
		           "s2lpSend(), invalid state");

	if (devp->state != S2LP_READY_TX) {
		/* configure fifo irq line */
		devp->config->gpio_cfg->fifo_irq_line_cfg->xS2LPGpioIO = S2LP_GPIO_DIG_OUT_TXRX_FIFO_ALMOST_EMPTY;
		S2LPGpioInit(devp->config->gpio_cfg->fifo_irq_line_cfg);
		S2LPFifoMuxRxFifoIrqEnable(S_DISABLE);

#ifdef S2LP_USE_CSMA_ENGINE
		s2lp_enable_csma(devp);
#endif
	}

	devp->state = S2LP_TX;

#if S2LP_USE_BASIC_PROTOCOL
	/* set payload length */
	S2LPPktBasicSetPayloadLength(n);

	/* set destination address */
	S2LPSetRxSourceReferenceAddress(raddr);
#endif

	/* send packet */
	tx_msg = s2lp_transmission_handler(devp, n, txp);

	/* wait for ready state */
	do {
	    S2LPRefreshStatus();
	} while (g_xStatus.MC_STATE != MC_STATE_READY);

	devp->state = S2LP_READY_TX;

	return tx_msg;
}

msg_t s2lp_base_receive(S2LPDriver *devp, uint16_t *n, uint8_t* rxp, sysinterval_t interval, s2lp_rx_mode_t rx_mode) {
	msg_t rx_msg;

	osalDbgAssert((devp->state == S2LP_READY)		||
			  	  (devp->state == S2LP_READY_TX)	||
				  (devp->state == S2LP_READY_RX),
		           "s2lpReceive(), invalid state");

	if (devp->state != S2LP_READY_RX) {
		/* configure fifo irq line */
		devp->config->gpio_cfg->fifo_irq_line_cfg->xS2LPGpioIO = S2LP_GPIO_DIG_OUT_TXRX_FIFO_ALMOST_FULL;
		S2LPGpioInit(devp->config->gpio_cfg->fifo_irq_line_cfg);
		S2LPFifoMuxRxFifoIrqEnable(S_ENABLE);

#ifdef S2LP_USE_CSMA_ENGINE
		/* disable csma */
		S2LPCsma(S_DISABLE);
#endif
		/* enable cs blanking */
		S2LPRadioCsBlanking(S_ENABLE);
	}

	/* set rx timeout */
	s2lp_set_rx_timeout(devp, interval);

	/* enable SQI */
	s2lp_enable_sqi(devp);

	if (rx_mode == S2LP_RX_SNIFF) {
		/* set sniff timer expiration time (us) */
		S2LpSetTimerFastRxTermTimerUs( 1000000 *
									   devp->config->pkt_basic_cfg->xPreambleLength /
									   devp->config->radio_cfg->lDatarate);

		S2LpTimerFastRxTermTimer(S_ENABLE);
	}

	devp->state = S2LP_RX;

	rx_msg = s2lp_reception_handler(devp, rxp);

	/* wait for ready state */
	do {
	    S2LPRefreshStatus();
	} while (g_xStatus.MC_STATE != MC_STATE_READY);

#if S2LP_USE_BASIC_PROTOCOL
	*n = S2LPPktBasicGetReceivedPktLength();
#endif

	devp->state = S2LP_READY_RX;
	return rx_msg;
}

/* #TODO: manage packet_length > rx buffer size case */
msg_t s2lp_receive_timeout(void *ip, uint16_t *n, uint8_t* rxp, sysinterval_t interval) {
	S2LPDriver* devp;

	osalDbgCheck(ip != NULL);

	/* Getting parent instance pointer.*/
	devp = objGetInstance(S2LPDriver*, (BaseRadio*)ip);

	return s2lp_base_receive(devp, n, rxp, interval, S2LP_RX_TIMEOUT);
}

msg_t s2lp_receive(	void *ip, uint16_t *n, uint8_t* rxp) {
	return s2lp_receive_timeout(ip, n, rxp, TIME_INFINITE);
}

static const struct S2LPVMT vmt_device = {
  (size_t)0
};

static const struct BaseRadioVMT vmt_radio = {
  sizeof(struct S2LPVMT*),
  s2lp_send,
  s2lp_receive_timeout,
  s2lp_receive
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void s2lpObjectInit(S2LPDriver *devp) {
	devp->vmt = &vmt_device;
	devp->radio_if.vmt = &vmt_radio;

	devp->config = NULL;

	devp->thread = NULL;

	devp->state = S2LP_STOP;
}

void s2lpStart(S2LPDriver *devp, S2LPConfig *config) {

	osalDbgCheck((devp != NULL) && (config != NULL));

	osalDbgAssert((devp->state == S2LP_STOP) ||
				  (devp->state == S2LP_READY),
	               "s2lpStart(), invalid state");

	devp->config = config;

	s2lp_ext_devp = devp;

	osalDbgCheck((devp->config->spip   != NULL) &&
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

	/** set the oscillator frequency */
	S2LPRadioSetXtalFrequency(devp->config->xtal_freq);

	/** initialize radio parameters   */
	/** enable automatic calibrations */
	S2LPTimerCalibrationRco(S_ENABLE);
	S2LPRadioCalibrationVco(S_ENABLE, S_ENABLE);

	/* #TODO: AFC AGC */

	/* #TODO: PLL */

	/* configure irq gpio */
	S2LPGpioInit(devp->config->gpio_cfg->irq_line_cfg);

	/** configure frequencies */
	S2LPRadioInit(devp->config->radio_cfg);

	/* configure power */
	S2LPRadioSetPALeveldBm(devp->config->pwr_idx, devp->config->pwr_dbm);
	S2LPRadioSetPALevelMaxIndex(devp->config->pwr_idx);

#if S2LP_USE_BASIC_PROTOCOL

	/* basic packet protocol configuration */
	S2LPPktBasicInit( devp->config->pkt_basic_cfg);

	/* configure preamble pattern to be compliant with Spirit1 */
	S2LPPacketHandlerSwapPreamblePattern(S_DISABLE);

	/* configure addresses */
	S2LPPktBasicAddressesInit(devp->config->pkt_basic_addr_cfg);

#endif

	/* configure FIFO thresholds */
	S2LPFifoSetAlmostEmptyThresholdTx(S2LP_TX_FIFO_AE_THRESHOLD);
	S2LPFifoSetAlmostFullThresholdRx(S2LP_RX_FIFO_AF_THRESHOLD);

	devp->state = S2LP_READY;
}

msg_t s2lpReceiveSniff(S2LPDriver *devp, uint16_t* n, uint8_t* rxp) {
	return s2lp_base_receive(devp, n, rxp, TIME_INFINITE, S2LP_RX_SNIFF);
}

void s2lpStop(S2LPDriver *devp) {

	osalDbgCheck(devp != NULL);

	osalDbgAssert((devp->state == S2LP_STOP) ||
				  (devp->state == S2LP_READY) ||
				  (devp->state == S2LP_READY_RX) ||
				  (devp->state == S2LP_READY_TX),
	               "s2lpStop(), invalid state");

	if (devp->state == S2LP_READY) {
		spiStop(devp->config->spip);
	}

	s2lp_enter_shutdown(devp);

	devp->state = S2LP_STOP;
}

/** @} */
