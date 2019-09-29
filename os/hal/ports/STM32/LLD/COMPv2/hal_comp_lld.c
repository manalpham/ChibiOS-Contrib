/*
    ChibiOS - Copyright (C) 2006..2017 Giovanni Di Sirio
              Copyright (C) 2017 Fabien Poussin (fabien.poussin (at) google's mail)

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
 * @file    STM32/hal_comp_lld.c
 * @brief   STM32 Comp subsystem low level driver header.
 *
 * @addtogroup COMP
 * @{
 */

#include "hal.h"

#if HAL_USE_COMP || defined(__DOXYGEN__)

#include "hal_comp.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   COMPD1 driver identifier.
 * @note    The driver COMPD1 allocates the comparator COMP1 when enabled.
 */
#if STM32_COMP_USE_COMP1 || defined(__DOXYGEN__)
COMPDriver COMPD1;
#endif

/**
 * @brief   COMPD2 driver identifier.
 * @note    The driver COMPD2 allocates the comparator COMP2 when enabled.
 */
#if STM32_COMP_USE_COMP2 || defined(__DOXYGEN__)
COMPDriver COMPD2;
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


/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level COMP driver initialization.
 *
 * @notapi
 */
void comp_lld_init(void) {

#if STM32_COMP_USE_COMP1
  /* Driver initialization.*/
  compObjectInit(&COMPD1);
  COMPD1.reg = COMP1;
  COMPD1.reg->CSR = 0;
#if STM32_COMP_USE_INTERRUPTS
  nvicEnableVector(COMP_IRQn, STM32_COMP_IRQ_PRIORITY);
#endif
#endif

#if STM32_COMP_USE_COMP2
  /* Driver initialization.*/
  compObjectInit(&COMPD2);
  COMPD2.reg = COMP2;
  COMPD2.reg->CSR = 0;
#if STM32_COMP_USE_INTERRUPTS
  nvicEnableVector(COMP_IRQn, STM32_COMP_IRQ_PRIORITY);
#endif
#endif

}

/**
 * @brief  COMP1, COMP2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector140) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR1;
  pr &= EXTI->IMR1 & ((1U << 21) | (1U << 22));
  EXTI->PR1 = pr;
#if STM32_COMP_USE_COMP1
  if (pr & (1U << 21) && COMPD1.config->cb != NULL)
    COMPD1.config->cb(&COMPD1);
#endif
#if STM32_COMP_USE_COMP2
  if (pr & (1U << 22) && COMPD2.config->cb != NULL)
    COMPD2.config->cb(&COMPD2);
#endif

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   Configures and activates an EXT channel (used by comp)
 *
 * @param[in] compp      pointer to the @p COMPDriver object
 * @param[in] channel    EXT channel
 *
 * @notapi
 */
void comp_ext_lld_channel_enable(COMPDriver *compp, uint32_t channel) {

  uint32_t cmask = 1 << channel;

  /* Don't touch other channels */
  if (channel < 21 || channel > 22) {
      return;
  }

  /* Masked out lines must not be touched by this driver.*/
  if ((cmask & STM32_EXTI_IMR1_MASK) != 0U) {
      return;
  }

  /* Programming edge registers.*/
  if (compp->config->irq_mode == COMP_IRQ_RISING || compp->config->irq_mode == COMP_IRQ_BOTH)
      EXTI->RTSR1 |= cmask;
  else
      EXTI->RTSR1 &= ~cmask;
  if (compp->config->irq_mode == COMP_IRQ_FALLING || compp->config->irq_mode == COMP_IRQ_BOTH)
      EXTI->FTSR1 |= cmask;
  else
      EXTI->FTSR1 &= ~cmask;

    /* Programming interrupt and event registers.*/
    EXTI->IMR1 |= cmask;
    EXTI->EMR1 &= ~cmask;

}

/**
 * @brief   Deactivate an EXT channel (used by comp)
 *
 * @param[in] compp      pointer to the @p COMPDriver object
 * @param[in] channel    EXT channel
 *
 * @notapi
 */
void comp_ext_lld_channel_disable(COMPDriver *compp, uint32_t channel) {

  (void) compp;
  uint32_t cmask = 1 << channel;

  /* Don't touch other channels */
  if (channel < 21 || channel > 22) {
      return;
  }

  /* Masked out lines must not be touched by this driver.*/
  if ((cmask & STM32_EXTI_IMR1_MASK) != 0U) {
      return;
  }

  EXTI->IMR1  &= ~cmask;
  EXTI->EMR1  &= ~cmask;
  EXTI->RTSR1 &= ~cmask;
  EXTI->FTSR1 &= ~cmask;
  EXTI->PR1    =  cmask;
}

/**
 * @brief   Configures and activates the COMP peripheral.
 *
 * @param[in] compp      pointer to the @p COMPDriver object
 *
 * @notapi
 */
void comp_lld_start(COMPDriver *compp) {

  // Apply CSR Execpt the enable bit.
  compp->reg->CSR = (compp->config->csr | COMP_CSR_BRGEN | COMP_CSR_SCALEN) & ~COMP_CSR_EN;

  // Inverted output
  if (compp->config->output_mode == COMP_OUTPUT_INVERTED)
    compp->reg->CSR |= COMP_CSR_POLARITY;

#if STM32_COMP_USE_INTERRUPTS
#if STM32_COMP_USE_COMP1
  if (compp == &COMPD1) {
    comp_ext_lld_channel_enable(compp, 21);
  }
#endif

#if STM32_COMP_USE_COMP2
  if (compp == &COMPD2) {
    comp_ext_lld_channel_enable(compp, 22);
  }
#endif
#endif

}

/**
 * @brief   Deactivates the comp peripheral.
 *
 * @param[in] compp      pointer to the @p COMPDriver object
 *
 * @notapi
 */
void comp_lld_stop(COMPDriver *compp) {

  if (compp->state == COMP_READY) {

    compp->reg->CSR = 0;
  }

#if STM32_COMP_USE_INTERRUPTS
#if STM32_COMP_USE_COMP1
  if (compp == &COMPD1) {
    comp_ext_lld_channel_disable(compp, 21);
  }
#endif

#if STM32_COMP_USE_COMP2
  if (compp == &COMPD2) {
    comp_ext_lld_channel_disable(compp, 22);
  }
#endif
#endif

}

/**
 * @brief   Enables the output.
 *
 * @param[in] compp      pointer to the @p COMPDriver object
 *
 * @notapi
 */
void comp_lld_enable(COMPDriver *compp) {

   compp->reg->CSR |= COMP_CSR_EN; /* Enable */
}

/**
 * @brief   Disables the output.
 *
 * @param[in] compp      pointer to the @p COMPDriver object
 *
 * @notapi
 */
void comp_lld_disable(COMPDriver *compp) {

  compp->reg->CSR &= ~COMP_CSR_EN; /* Disable */
}

#endif /* HAL_USE_COMP */

/** @} */
