/**
 * @file    S2LP_Timer_ex.c
 * @author  LowPower RF BU - AMG
 * @version 1.2.0
 * @date    October 31, 2016
 * @brief   This file provides functions to manage S2-LP debug.
 * @details
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "S2LP_Timer_ex.h"

#include "S2LP_MCU_Interface.h"
#include "S2LP_Regs.h"
#include "S2LP_Timer.h"


/** @addtogroup S2LP_Libraries
 * @{
 */


/** @addtogroup S2LP_Types
 * @{
 */

/** @defgroup Types_Private_Functions                 Types Private Functions
 * @{
 */


void S2LPTimerSetRxTimerMs(S2LPDriver *devp, float fDesiredMsec)
{
  S2LPTimerSetRxTimerUs(devp, (uint32_t)(fDesiredMsec*1000));
}

void S2LPTimerGetRxTimer(S2LPDriver *devp, float* pfTimeoutMsec, uint8_t* pcCounter , uint8_t* pcPrescaler)
{
  uint32_t timeoutUsec;
  
  S2LPTimerGetRxTimerUs(devp, &timeoutUsec, pcCounter , pcPrescaler);
    
  (*pfTimeoutMsec)=((float)timeoutUsec)/1000;
}

void S2LPTimerSetWakeUpTimerMs(S2LPDriver *devp, float fDesiredMsec)
{
  S2LPTimerSetWakeUpTimerUs(devp, (uint32_t)(fDesiredMsec*1000));
}

void S2LPTimerSetWakeUpTimerReloadMs(S2LPDriver *devp, float fDesiredMsec)
{
  S2LPTimerSetWakeUpTimerReloadUs(devp, (uint32_t)(fDesiredMsec*1000));
}

void S2LPTimerGetWakeUpTimer(S2LPDriver *devp, float* pfWakeUpMsec, uint8_t* pcCounter , uint8_t* pcPrescaler, uint8_t* pcMulti)
{
  uint32_t timeoutUsec;
  
  S2LPTimerGetWakeUpTimerUs(devp, &timeoutUsec, pcCounter , pcPrescaler, pcMulti);
    
  (*pfWakeUpMsec)=((float)timeoutUsec)/1000;
}

void S2LPTimerGetWakeUpTimerReload(S2LPDriver *devp, float* pfWakeUpReloadMsec, uint8_t* pcCounter, uint8_t* pcPrescaler, uint8_t* pcMulti)
{
  uint32_t timeoutUsec;
  
  S2LPTimerGetWakeUpTimerReloadUs(devp, &timeoutUsec, pcCounter , pcPrescaler, pcMulti);
    
  (*pfWakeUpReloadMsec)=((float)timeoutUsec)/1000;
}



/**
 * @}
 */



/**
 * @}
 */



/**
 * @}
 */



/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
