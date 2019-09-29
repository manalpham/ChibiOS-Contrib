/*
    ChibiOS - Copyright (C) 2006..2016 Martino Migliavacca

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
 * @file    TIMv1/hal_qei_lld.c
 * @brief   STM32 QEI subsystem low level driver header.
 *
 * @addtogroup QEI
 * @{
 */

#include "hal.h"

#if (HAL_USE_QEI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   LPQEID1 driver identifier.
 * @note    The driver LPQEID1 allocates the complex timer LPTIM1 when enabled.
 */
#if STM32_QEI_USE_LPTIM1 || defined(__DOXYGEN__)
QEIDriver LPQEID1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_QEI_USE_LPTIM1 || defined(__DOXYGEN__)
#if !defined(STM32_LPTIM1_SUPPRESS_ISR)
#if !defined(STM32_LPTIM1_HANDLER)
#error "STM32_LPTIM1_HANDLER not defined"
#endif
/**
 * @brief   LPTIM1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_LPTIM1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  qei_lld_serve_interrupt(&LPQEID1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_LPTIM1_SUPPRESS_ISR) */
#endif /* STM32_QEI_USE_LPTIM1 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level QEI driver initialization.
 *
 * @notapi
 */
void qei_lld_init(void) {

#if STM32_QEI_USE_LPTIM1
  /* Driver initialization.*/
  qeiObjectInit(&LPQEID1);
  LPQEID1.tim = STM32_LPTIM1;
#endif
}

/**
 * @brief   Configures and activates the QEI peripheral.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @notapi
 */
void qei_lld_start(QEIDriver *qeip) {
  osalDbgAssert((qeip->config->min == 0) || (qeip->config->max == 0),
		"only min/max set to 0 is supported");

  if (qeip->state == QEI_STOP) {
    /* Clock activation and timer reset.*/
#if STM32_QEI_USE_LPTIM1
    if (&LPQEID1 == qeip) {
      rccEnableLPTIM1(FALSE);
      rccResetLPTIM1();
#if !defined(STM32_LPTIM1_SUPPRESS_ISR) && STM32_QEI_USE_INTERRUPTS
      EXTI->IMR2 |= 1U;
      nvicEnableVector(STM32_LPTIM1_NUMBER, STM32_QEI_LPTIM1_IRQ_PRIORITY);
#endif
    }
#endif
  }
   /* Timer configuration.*/
  qeip->tim->CR   = 0;                      /* Initially stopped. */
  qeip->tim->CFGR = 0;
  qeip->tim->ICR  = 0x1FF;					/* Clear interrupts flags */
  qeip->tim->IER  = 0;
  qeip->tim->OR	  = 0;

  /* resolution & direction configuration */
  if (qeip->config->resolution == QEI_BOTH_EDGES)
      qeip->tim->CFGR = STM32_LPTIM_CFGR_CKPOL(2);
  else {
	  if (qeip->config->dirinv == QEI_DIRINV_TRUE) {
		  qeip->tim->CFGR = STM32_LPTIM_CFGR_CKPOL(1);
	  }
  }

#if defined(QEI_DRIVER_EXT_FIELDS)
  /* configure inputs */
  qeip->tim->OR = (STM32_LPTIM_OR_0  |
		           STM32_LPTIM_OR_1) &
		           qeip->config->inputs;

  /* configure filters */
  qeip->tim->CFGR |= (STM32_LPTIM_CFGR_TRGFLT_MASK |
		              STM32_LPTIM_CFGR_CKFLT_MASK) &
		              qeip->config->filters;
#endif

#if STM32_QEI_USE_INTERRUPTS
  /* configure interrupts */
  osalDbgAssert((qeip->config->overflow_cb != NULL),
		"an overflow_cb must be specified ");
  qeip->tim->IER = STM32_LPTIM_IER_ARRMIE;
#endif

  /* configure encoder mode */
  qeip->tim->CFGR |= STM32_LPTIM_CFGR_ENC;
}

/**
 * @brief   Deactivates the QEI peripheral.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @notapi
 */
void qei_lld_stop(QEIDriver *qeip) {

  if (qeip->state == QEI_READY) {
    qeip->tim->CR  = 0;                    /* Timer disabled. */
    qeip->tim->ICR = 0x1FF;

    /* Clock deactivation.*/
#if STM32_QEI_USE_LPTIM1
    if (&LPQEID1 == qeip) {
      rccDisableLPTIM1();
    }
#if !defined(STM32_LPTIM1_SUPPRESS_ISR) && STM32_QEI_USE_INTERRUPTS
      nvicEnableVector(STM32_LPTIM1_NUMBER, STM32_QEI_LPTIM1_IRQ_PRIORITY);
      EXTI->IMR2 &= ~1U;
#endif
#endif
  }
}

/**
 * @brief   Enables the input capture.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @notapi
 */
void qei_lld_enable(QEIDriver *qeip) {

  qeip->tim->CR |= LPTIM_CR_ENABLE;           /* Timer enabled. */

#if defined(QEI_DRIVER_EXT_FIELDS)
  /* set auto reload register */
  qeip->tim->ARR = 0xFFFF & (qeip->config->max - qeip->config->min - 1);
#endif

  /* start continuous mode */
  qeip->tim->CR |= STM32_LPTIM_CR_CNTSTRT;

}

/**
 * @brief   Disables the input capture.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @notapi
 */
void qei_lld_disable(QEIDriver *qeip) {

  qeip->tim->CR &= !LPTIM_CR_ENABLE;          /* Timer disabled. */
}

/**
 * @brief   Shared IRQ handler.
 *
 * @param[in] qei      pointer to the @p QEIDriver object
 *
 * @notapi
 */
void qei_lld_serve_interrupt(QEIDriver *qeip) {

	if ((qeip->tim->ISR & STM32_LPTIM_ISR_ARRM) != 0) {
		qeip->config->overflow_cb(qeip, 0);
	}

	/*if ((qeip->tim->ISR & (STM32_LPTIM_ISR_UP | STM32_LPTIM_ISR_DOWN)) != 0) {
		qeip->config->notify_cb(qeip);
	}*/

	qeip->tim->ICR = 0x1FF;  /* Clear pending IRQs. */
}

#endif /* HAL_USE_QEI */

/** @} */
