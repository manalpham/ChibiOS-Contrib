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
 * @file    STM32/comp_lld.h
 * @brief   STM32 Comparator subsystem low level driver header.
 *
 * @addtogroup COMP
 * @{
 */

#ifndef HAL_COMP_LLD_H_
#define HAL_COMP_LLD_H_

#include "hal.h"

#if HAL_USE_COMP || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/


/*!< 1/4 VREFINT connected to comparator inverting input */
#define STM32_COMP_InvertingInput_1_4VREFINT        ((uint32_t)0x00000000)

/*!< 1/2 VREFINT connected to comparator inverting input */
#define STM32_COMP_InvertingInput_1_2VREFINT        COMP_CSR_INMSEL_0

/*!< 3/4 VREFINT connected to comparator inverting input */
#define STM32_COMP_InvertingInput_3_4VREFINT        COMP_CSR_INMSEL_1

/*!< VREFINT connected to comparator inverting input */
#define STM32_COMP_InvertingInput_VREFINT           (COMP_CSR_INMSEL_0 | COMP_CSR_INMSEL_1)

/*!< DAC1_OUT1 (PA4) connected to comparator inverting input */
#define STM32_COMP_InvertingInput_DAC1OUT1          COMP_CSR_INMSEL_2

/*!< DAC1_OUT2 (PA5) connected to comparator inverting input */
#define STM32_COMP_InvertingInput_DAC1OUT2          (COMP_CSR_INMSEL_0 | COMP_CSR_INMSEL_2)

/* L41xx / L42xx / L43xx / L44xx / L45xx / L46xx
 * COMP1 PB1
 * COMP2 PB3 */
#define STM32_COMP_InvertingInput_IO0	(COMP_CSR_INMSEL_1 | COMP_CSR_INMSEL_2)

/* L41xx / L42xx / L43xx / L44xx / L45xx / L46xx
 * COMP1 PC4
 * COMP2 PB7 */
#define STM32_COMP_InvertingInput_IO1	COMP_CSR_INMSEL

/* L41xx / L42xx / L43xx / L44xx / L45xx / L46xx
 * COMP1 PA0
 * COMP2 PA2 */
#define STM32_COMP_InvertingInput_IO2	(COMP_CSR_INMSEL | COMP_CSR_INMESEL_0)

/* L41xx / L42xx / L43xx / L44xx / L45xx / L46xx
 * COMP1 PA4
 * COMP2 PA4 */
#define STM32_COMP_InvertingInput_IO3	(COMP_CSR_INMSEL | COMP_CSR_INMESEL_1)

/* L41xx / L42xx / L43xx / L44xx / L45xx / L46xx
 * COMP1 PA5
 * COMP2 PA5 */
#define STM32_COMP_InvertingInput_IO4	(COMP_CSR_INMSEL | COMP_CSR_INMESEL)

/* L41xx / L42xx / L43xx / L44xx / L45xx / L46xx
 * COMP1 PC5
 * COMP2 PB4 */
#define STM32_COMP_NonInvertingInput_IO0	((uint32_t)0x00000000)

/* L41xx / L42xx / L43xx / L44xx / L45xx / L46xx
 * COMP1 PB2
 * COMP2 PB6 */
#define STM32_COMP_NonInvertingInput_IO1	COMP_CSR_INPSEL_0

/* L41xx / L42xx / L43xx / L44xx / L45xx / L46xx
 * COMP1 PA1
 * COMP2 PA3 */
#define STM32_COMP_NonInvertingInput_IO2	COMP_CSR_INPSEL_1

/* No blanking source can be selected for all comparators */
#define STM32_COMP_BlankingSrce_None		((uint32_t)0x00000000)

/* L41xx / L42xx / L43xx / L44xx / L45xx / L46xx
 * COMP1 TIM1 OC5 */
#define STM32_COMP_BlankingSrce_TIM1_OC5	COMP_CSR_BLANKING_0

/* L41xx / L42xx / L43xx / L44xx / L45xx / L46xx
 * COMP1 TIM2 OC3 */
#define STM32_COMP_BlankingSrce_TIM2_OC3	COMP_CSR_BLANKING_1

/* L41xx / L42xx / L43xx / L44xx / L45xx / L46xx
 * COMP2 TIM15 OC1 */
#define STM32_COMP_BlankingSrce_TIM15_OC1	COMP_CSR_BLANKING_2

#define STM32_COMP_OutputPol_NonInverted              ((uint32_t)0x00000000)  /*!< COMP output on GPIO isn't inverted */
#define STM32_COMP_OutputPol_Inverted                 COMP_CSR_POLARITY       /*!< COMP output on GPIO is inverted */

#define STM32_COMP_Hysteresis_No                      0x00000000           /*!< No hysteresis */
#define STM32_COMP_Hysteresis_Low                     COMP_CSR_HYST_0      /*!< Hysteresis level low */
#define STM32_COMP_Hysteresis_Medium                  COMP_CSR_HYST_1      /*!< Hysteresis level medium */
#define STM32_COMP_Hysteresis_High                    COMP_CSR_HYST        /*!< Hysteresis level high */

#define STM32_COMP_Mode_HighSpeed                     0x00000000            /*!< High Speed */
#define STM32_COMP_Mode_MediumSpeed                   COMP_CSR_PWRMODE_0    /*!< Medium Speed */
#define STM32_COMP_Mode_UltraLowPower                 COMP_CSR_PWRMODE 		/*!< Ultra-low power mode */


#if defined(STM32L452xx) || defined(STM32L431xx)
#define STM32_HAS_COMP1 TRUE
#define STM32_HAS_COMP2 TRUE
#endif

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */

/**
 * @brief   COMP INTERRUPTS.
 * @details If set to @p TRUE the support for COMPD1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_COMP_USE_INTERRUPTS) || defined(__DOXYGEN__)
#define STM32_COMP_USE_INTERRUPTS             FALSE
#endif

/**
 * @brief   COMPD1 driver enable switch.
 * @details If set to @p TRUE the support for COMPD1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_COMP_USE_COMP1) || defined(__DOXYGEN__)
#define STM32_COMP_USE_COMP1                  FALSE
#endif

/**
 * @brief   COMPD2 driver enable switch.
 * @details If set to @p TRUE the support for COMPD2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_COMP_USE_COMP2) || defined(__DOXYGEN__)
#define STM32_COMP_USE_COMP2                  FALSE
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_COMP_USE_INTERRUPTS
#if !defined(STM32_DISABLE_EXTI21_22_HANDLER)
#error "COMP needs these defines in mcuconf to use interrupts: STM32_DISABLE_EXTI21_22_HANDLER"
#endif
#endif

#if STM32_COMP_USE_COMP1 && !STM32_HAS_COMP1
#error "COMP1 not present in the selected device"
#endif

#if STM32_COMP_USE_COMP2 && !STM32_HAS_COMP2
#error "COMP2 not present in the selected device"
#endif

#if !STM32_COMP_USE_COMP1 && !STM32_COMP_USE_COMP2
#error "COMP driver activated but no COMP peripheral assigned"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   COMP output mode.
 */
typedef enum {
  COMP_OUTPUT_NORMAL = 0,
  COMP_OUTPUT_INVERTED = 1
} comp_output_mode_t;

/**
 * @brief   COMP interrupt mode.
 */
typedef enum {
  COMP_IRQ_RISING = 0,
  COMP_IRQ_FALLING = 1,
  COMP_IRQ_BOTH = 2
} comp_irq_mode_t;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Ouput mode.
   */
  comp_output_mode_t        output_mode;

  /**
   * @brief   Ouput mode.
   */
  comp_irq_mode_t           irq_mode;

  /**
   * @brief   Callback.
   */
  compcallback_t             cb;

  /* End of the mandatory fields.*/

  /**
   * @brief COMP CSR register initialization data.
   * @note  The value of this field should normally be equal to zero.
   */
  uint32_t                  csr;
} COMPConfig;

/**
 * @brief   Structure representing an COMP driver.
 */
struct COMPDriver {
  /**
   * @brief Driver state.
   */
  compstate_t                state;
  /**
   * @brief Current configuration data.
   */
  const COMPConfig           *config;
#if defined(COMP_DRIVER_EXT_FIELDS)
  COMP_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief Pointer to the COMPx registers block.
   */
  COMP_TypeDef               *reg;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_COMP_USE_COMP1 && !defined(__DOXYGEN__)
extern COMPDriver COMPD1;
#endif

#if STM32_COMP_USE_COMP2 && !defined(__DOXYGEN__)
extern COMPDriver COMPD2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void comp_lld_init(void);
  void comp_lld_start(COMPDriver *compp);
  void comp_lld_stop(COMPDriver *compp);
  void comp_lld_enable(COMPDriver *compp);
  void comp_lld_disable(COMPDriver *compp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_COMP */

#endif /* _comp_lld_H_ */

/** @} */
