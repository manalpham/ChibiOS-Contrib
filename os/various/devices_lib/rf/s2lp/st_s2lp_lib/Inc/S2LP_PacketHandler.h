/**
 * @file    S2LP_PacketHandler.h
 * @author  LowPower RF BU - AMG
 * @version 1.3.0
 * @date    10-July-2017
 * @brief   Configuration and management of the common features of S2-LP packets.
 * @details
 *
 * This module provides all the common functions and definitions used by the
 * packets modules.
 * Here are also defined all the generic enumeration types that are redefined
 * in the specific packets modules, but every enumeration value is referred
 * to this module. So the user who wants to configure the preamble of a Basic,
 * or a STack packet has to use the enumeration values defined here.
 *
 * <b>Example:</b>
 * @code
 *
 *   ...
 *
 *   S2LPPktBasicSetPreambleLength(PKT_PREAMBLE_LENGTH_18BYTES);
 *
 *   ...
 *
 * @endcode
 *
 * @note Is recommended for the user to not use these API directly
 * importing this module in his application.
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
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __S2LP_PKT_COMMON_H
#define __S2LP_PKT_COMMON_H

/* Includes ------------------------------------------------------------------*/

#include "S2LP_Regs.h"
#include "S2LP_Types.h"

#ifdef __cplusplus
 extern "C" {
#endif


/**
 * @addtogroup S2LP_Libraries
 * @{
 */


/**
 * @defgroup S2LP_PktCommon           Pkt Common
 * @brief Configuration and management of the common features of S2LP packets.
 * @details See the file <i>@ref S2LP_PktCommon.h</i> for more details.
 * @{
 */

/**
 * @defgroup PktCommon_Exported_Types   Pkt Common Exported Types
 * @{
 */


/**
 * @brief  CRC length in bytes enumeration.
 */
typedef enum {
  PKT_NO_CRC               = 0x00, /*!< No CRC                              */
  PKT_CRC_MODE_8BITS       = 0x20, /*!< CRC length 8 bits  - poly: 0x07     */
  PKT_CRC_MODE_16BITS_1    = 0x40, /*!< CRC length 16 bits - poly: 0x8005   */
  PKT_CRC_MODE_16BITS_2    = 0x60, /*!< CRC length 16 bits - poly: 0x1021   */
  PKT_CRC_MODE_24BITS      = 0x80, /*!< CRC length 24 bits - poly: 0x864CFB */
  PKT_CRC_MODE_32BITS      = 0xA0, /*!< CRC length 32 bits - poly: 0x04C011BB7 */
} PktCrcMode;


/**
 * @brief  Direct transmission mode enumeration for SPIRIT.
 */
typedef enum
{
  NORMAL_TX_MODE = 0x00,          /*!< Normal mode, no direct transmission is used */
  DIRECT_TX_FIFO_MODE = 0x04,     /*!< Source is FIFO: payload bits are continuously read from the TX FIFO */
  DIRECT_TX_GPIO_MODE = 0x08,     /*!< Source is GPIO: payload bits are continuously read from one of the GPIO ports and transmitted without any processing */
  PN9_TX_MODE = 0x0C              /*!< A pseudorandom binary sequence is generated internally */
} DirectTx;


/**
 * @brief  Direct receive mode enumeration for SPIRIT.
 */
typedef enum
{
  NORMAL_RX_MODE = 0x00,          /*!< Normal mode, no direct reception is used */
  DIRECT_RX_FIFO_MODE = 0x10,     /*!< Destination is FIFO: payload bits are continuously written to the RX FIFO and not subjected to any processing*/
  DIRECT_RX_GPIO_MODE = 0x20      /*!< Destination is GPIO: payload bits are continuously written to one of the GPIO ports and not subjected to any processing*/
} DirectRx;


/**
 *@}
 */


/**
 * @defgroup PktCommon_Exported_Constants               Pkt Common Exported Constants
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup PktCommon_Exported_Macros                  Pkt Common Exported Macros
 * @{
 */

#define S2LPSetPreambleLengthByte(devp, xPreambleLength)      S2LPSetPreambleLength(devp, 4*xPreambleLength)
#define S2LPSetSyncLengthByte(devp, cSyncLength)              S2LPSetSyncLength(devp, 8*cSyncLength)

#define IS_PREAMBLE_LEN(VAL)  (VAL<=1024)
#define IS_SYNC_LEN(VAL)      (VAL<=64)

#define IS_PKT_LEN_FIELD_WID(LENGTH)   ((LENGTH == PKT_LENGTH_FIELD_1BYTE) || \
                                        (LENGTH == PKT_LENGTH_FIELD_2BYTE))


#define IS_PKT_CRC_MODE(MODE)   ((MODE == PKT_NO_CRC) || \
                                (MODE == PKT_CRC_MODE_8BITS)  || \
                                (MODE == PKT_CRC_MODE_16BITS_1)  || \
                                (MODE == PKT_CRC_MODE_16BITS_2) || \
                                (MODE == PKT_CRC_MODE_24BITS))
/**
 *@}
 */


/**
 * @defgroup PktCommon_Exported_Functions               Pkt Common Exported Functions
 * @{
 */

void S2LPSetPreambleLength(S2LPDriver *devp, uint16_t cPreambleLength);
uint16_t S2LPGetPreambleLength(S2LPDriver *devp);
void S2LPSetSyncLength(S2LPDriver *devp, uint8_t cSyncLength);
uint8_t S2LPGetSyncLength(S2LPDriver *devp);
void S2LPSetSyncWords(S2LPDriver *devp, uint32_t lSyncWords, uint8_t xSyncLength);
void S2LPGetSyncWords(S2LPDriver *devp, uint32_t* lSyncWords, uint8_t* cSyncLength);
void S2LPPacketHandlerWhitening(S2LPDriver *devp, SFunctionalState xNewState);
void S2LPPacketHandlerFec(S2LPDriver *devp, SFunctionalState xNewState);
void S2LPPacketHandler3OutOf6(S2LPDriver *devp, SFunctionalState xNewState);
void S2LPPacketHandlerManchester(S2LPDriver *devp, SFunctionalState xNewState);
uint8_t S2LPGetPacketFormat(S2LPDriver *devp);
void S2LPPktCommonFilterOnCrc(S2LPDriver *devp, SFunctionalState xNewState);
uint8_t S2LPGetReceivedDestinationAddress(S2LPDriver *devp);
uint8_t S2LPGetReceivedSourceAddress(S2LPDriver *devp);
uint8_t S2LPGetMyAddress(S2LPDriver *devp);
uint8_t S2LPGetBroadcastAddress(S2LPDriver *devp);
uint8_t S2LPGetMulticastAddress(S2LPDriver *devp);
uint8_t S2LPGetRxSourceMask(S2LPDriver *devp);
uint8_t S2LPGetRxSourceReferenceAddress(S2LPDriver *devp);
void S2LPPacketHandlerSetTxMode(S2LPDriver *devp, DirectTx xNewState);
void S2LPPacketHandlerSetRxMode(S2LPDriver *devp, DirectRx xNewState);
DirectTx S2LPPacketHandlerGetTxMode(S2LPDriver *devp);
DirectRx S2LPPacketHandlerGetRxMode(S2LPDriver *devp);
void S2LPPacketHandlerSetExtendedLenField(S2LPDriver *devp, SFunctionalState xExtendedLenField);
void S2LPPacketHandlerSwap4FSKSymbol(S2LPDriver *devp, SFunctionalState xSwapSymbol);
void S2LPPacketHandlerSwapFifoEndianess(S2LPDriver *devp, SFunctionalState xEnableSwap);
void S2LPPacketHandlerSwapPreamblePattern(S2LPDriver *devp, SFunctionalState xEnableSwap);
void S2LPPacketHandlerSetCrcMode(S2LPDriver *devp, PktCrcMode xPktCrcMode);
PktCrcMode S2LPPacketHandlerGetCrcMode(S2LPDriver *devp);
void S2LPPacketHandlerSelectSecondarySync(S2LPDriver *devp, SFunctionalState xSecondarySync);
void S2LPPacketHandlerSetAutoPcktFilter(S2LPDriver *devp, SFunctionalState xNewState);
void S2LPPacketHandlerSetRxPersistentMode(S2LPDriver *devp, SFunctionalState xNewState);
void S2LPPacketHandlerSetSrcAddrFlt(S2LPDriver *devp, SFunctionalState xNewState);
void S2LPPacketHandlerSetVariableLength(S2LPDriver *devp, SFunctionalState xVarLen);
void S2LPSetDualSyncWords(S2LPDriver *devp, uint32_t lSyncWords);
void S2LPGetDualSyncWords(S2LPDriver *devp, uint32_t* lSyncWords);
void S2LPSetRxSourceMask(S2LPDriver *devp, uint8_t address);
void S2LPSetRxSourceReferenceAddress(S2LPDriver *devp, uint8_t address);
void S2LPSetBroadcastAddress(S2LPDriver *devp, uint8_t address);
void S2LPSetMulticastAddress(S2LPDriver *devp, uint8_t address);
void S2LPSetMyAddress(S2LPDriver *devp, uint8_t address);
uint8_t S2LPGetMyAddress(S2LPDriver *devp);


/**
 *@}
 */

/**
 *@}
 */


/**
 *@}
 */

#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
