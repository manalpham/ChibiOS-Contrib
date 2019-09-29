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
 * @file    LPTIMv1/hal_qei_lld.h
 * @brief   STM32 QEI subsystem low level driver header.
 *
 * @addtogroup QEI
 * @{
 */

#ifndef HAL_QEI_LLD_H
#define HAL_QEI_LLD_H

#if (HAL_USE_QEI == TRUE) || defined(__DOXYGEN__)

#include "stm32_tim.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief Mininum usable value for defining counter underflow
 */
#define QEI_COUNT_MIN (0)

/**
 * @brief Maximum usable value for defining counter overflow
 */
#define QEI_COUNT_MAX (65535)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   LPQEID1 driver enable switch.
 * @details If set to @p TRUE the support for QEID1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_QEI_USE_LPTIM1) || defined(__DOXYGEN__)
#define STM32_QEI_USE_LPTIM1                  FALSE
#endif

/**
 * @brief   LPQEID1 interrupt priority level setting.
 */
#if !defined(STM32_QEI_LPTIM1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_QEI_LPTIM1_IRQ_PRIORITY         7
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_QEI_USE_LPTIM1 && !STM32_HAS_LPTIM1
#error "LPTIM1 not present in the selected device"
#endif

#if !STM32_QEI_USE_LPTIM1
#error "QEI driver activated but no LPTIM peripheral assigned"
#endif

/* Checks on allocation of LPTIMx units.*/
#if STM32_QEI_USE_LPTIM1
#if defined(STM32_LPTIM1_IS_USED)
#error "QEID1 requires LPTIM1 but the timer is already used"
#else
#define STM32_LPTIM1_IS_USED
#endif
#endif

/* IRQ priority checks.*/
#if STM32_QEI_USE_INTERRUPTS
#if STM32_QEI_USE_LPTIM1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_QEI_LPTIM1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to LPTIM1"
#endif
#endif

#if QEI_USE_OVERFLOW_DISCARD
#error "QEI_USE_OVERFLOW_DISCARD not supported by this driver"
#endif

#if QEI_USE_OVERFLOW_MINMAX
#error "QEI_USE_OVERFLOW_MINMAX not supported by this driver"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   QEI count mode.
 */
typedef enum {
  QEI_MODE_QUADRATURE = 0,          /**< Quadrature encoder mode.           */
  QEI_MODE_DIRCLOCK = 1,            /**< Direction/Clock mode.              */
} qeimode_t;

/**
 * @brief   QEI resolution.
 */
typedef enum {
  QEI_SINGLE_EDGE = 0,        /**< Count only on edges from first channel.  */
  QEI_BOTH_EDGES = 1,         /**< Count on both edges (resolution doubles).*/
} qeiresolution_t;

/**
 * @brief   QEI direction inversion.
 */
typedef enum {
  QEI_DIRINV_FALSE = 0,             /**< Do not invert counter direction.   */
  QEI_DIRINV_TRUE = 1,              /**< Invert counter direction.          */
} qeidirinv_t;

/**
 * @brief   QEI counter type.
 */
typedef int16_t qeicnt_t;

/**
 * @brief   QEI delta type.
 */
typedef int32_t qeidelta_t;

/* LPTIM additionnals fields */
#if STM32_QEI_USE_LPTIM1
#define QEI_DRIVER_EXT_FIELDS      		\
	uint32_t				inputs; 	\
	uint32_t				filters;
#endif

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Count mode.
   */
  qeimode_t                 mode;
  /**
   * @brief   Resolution.
   */
  qeiresolution_t           resolution;
  /**
   * @brief   Direction inversion.
   */
  qeidirinv_t               dirinv;
  /**
   * @brief   Handling of counter overflow/underflow
   *
   * @details When overflow occurs, the counter value is updated
   *          according to:
   *            - QEI_OVERFLOW_DISCARD:
   *                discard the update value, counter doesn't change
   */
  qeioverflow_t             overflow;
  /**
   * @brief   Min count value.
   *
   * @note    If min == max, then QEI_COUNT_MIN is used.
   *
   * @note    Only min set to 0 / QEI_COUNT_MIN is supported.
   */
  qeicnt_t                  min;
  /**
   * @brief   Max count value.
   *
   * @note    If min == max, then QEI_COUNT_MAX is used.
   *
   * @note    Only max set to 0 / QEI_COUNT_MAX is supported.
   */
  qeicnt_t                  max;
#if defined(QEI_DRIVER_EXT_FIELDS)
  QEI_DRIVER_EXT_FIELDS
#endif
  /**
    * @brief  Notify of value change
    *
    * @note   Called from ISR context.
    */
  qeicallback_t             notify_cb;
  /**
   * @brief   Notify of overflow
   *
   * @note    Overflow notification is performed after
   *          value changed notification.
   * @note    Called from ISR context.
   */
  void (*overflow_cb)(QEIDriver *qeip, qeidelta_t delta);
  /* End of the mandatory fields.*/
} QEIConfig;

/**
 * @brief   Structure representing an QEI driver.
 */
struct QEIDriver {
  /**
   * @brief Driver state.
   */
  qeistate_t                state;
  /**
   * @brief Last count value.
   */
  qeicnt_t                  last;
  /**
   * @brief Current configuration data.
   */
  const QEIConfig           *config;
  /* End of the mandatory fields.*/
  /**
   * @brief Pointer to the LPTIMx registers block.
   */
  stm32_lptim_t             *tim;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Returns the counter value.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 * @return              The current counter value.
 *
 * @notapi
 */
#define qei_lld_get_count(qeip) ((qeip)->tim->CNT)

/**
 * @brief   Set the counter value.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 * @param[in] qeip      counter value
 *
 * @notapi
 */
#define qei_lld_set_count(qeip, value) ((qeip)->tim->CNT = (value))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_QEI_USE_LPTIM1 && !defined(__DOXYGEN__)
extern QEIDriver LPQEID1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void qei_lld_init(void);
  void qei_lld_start(QEIDriver *qeip);
  void qei_lld_stop(QEIDriver *qeip);
  void qei_lld_enable(QEIDriver *qeip);
  void qei_lld_disable(QEIDriver *qeip);
  void qei_lld_serve_interrupt(QEIDriver *qeip);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_QEI */

#endif /* HAL_QEI_LLD_H */

/** @} */
