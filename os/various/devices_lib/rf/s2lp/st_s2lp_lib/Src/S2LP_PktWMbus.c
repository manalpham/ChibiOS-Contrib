/**
 * @file    S2LP_PktWMbus.c
 * @author  LowPower RF BU - AMG
 * @version 1.3.0
 * @date    10-July-2017
 * @brief   Configuration and management of S2-LP WMBUS packets.
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
 */


/* Includes ------------------------------------------------------------------*/
#include "S2LP_PktWMbus.h"

#include "S2LP_MCU_Interface.h"

/**
 * @addtogroup S2LP_Libraries
 * @{
 */


/**
 * @addtogroup S2LP_PktWMbus
 * @{
 */



/**
 * @defgroup PktWMbus_Private_Defines                    Pkt WMBUS Private Defines
 * @{
 */
#define PKT_FORMAT_WMBUS_CODE                   0x00

#define WMBUS_PREAMBLE_LEN_S1S2LONGHEADER           (uint16_t)279
#define WMBUS_PREAMBLE_LEN_S1MS2T2OTHERTOMETER      (uint16_t)15
#define WMBUS_PREAMBLE_LEN_T1T2METERTOOTHER         (uint16_t)19
#define WMBUS_PREAMBLE_LEN_R2                       (uint16_t)39
#define WMBUS_PREAMBLE_LEN_N1N2                     (uint16_t)8

#define WMBUS_SYNC_LEN_S1S2LONGHEADER               (uint8_t)18
#define WMBUS_SYNC_LEN_S1MS2T2OTHERTOMETER          (uint8_t)18
#define WMBUS_SYNC_LEN_T1T2METERTOOTHER             (uint8_t)10
#define WMBUS_SYNC_LEN_R2                           (uint8_t)18
#define WMBUS_SYNC_LEN_N1N2                         (uint16_t)16

#define WMBUS_SYNCWORD_S1S2LONGHEADER           (uint32_t)0xE25A4000
#define WMBUS_SYNCWORD_S1MS2T2OTHERTOMETER      (uint32_t)0xE25A4000
#define WMBUS_SYNCWORD_T1T2METERTOOTHER         (uint32_t)0x0F400000
#define WMBUS_SYNCWORD_R2                       (uint32_t)0xE25A4000
#define WMBUS_SYNCWORD_N1N2                     (uint32_t)0xf68d0000
/**
 *@}
 */


/**
 * @defgroup PktWMbus_Private_Macros                     Pkt WMBUS Private Macros
 * @{
 */

#define IS_WMBUS_SUBMODE(MODE)   (((MODE) == WMBUS_SUBMODE_S1_S2_LONG_HEADER) || \
                                 ((MODE) == WMBUS_SUBMODE_NOT_CONFIGURED) || \
                                 ((MODE) == WMBUS_SUBMODE_S1_M_S2_T2_OTHER_TO_METER) || \
                                 ((MODE) == WMBUS_SUBMODE_T1_T2_METER_TO_OTHER) || \
                                 ((MODE) == WMBUS_SUBMODE_R2_SHORT_HEADER))

/**
 *@}
 */


/**
 * @defgroup PktWMbus_Private_Variables                  Pkt WMBUS Private Variables
 * @{
 */
static WMbusSubmode s_cWMbusSubmode = WMBUS_SUBMODE_NOT_CONFIGURED;

/**
 *@}
 */


/**
 * @defgroup PktWMbus_Private_Functions                  Pkt WMBUS Private Functions
 * @{
 */


/**
 * @brief  Initialize the S2LP WMBUS packet according to the specified parameters in the PktWMbusInit struct.
 * @param  pxPktWMbusInit pointer to a PktWMbusInit structure that contains the configuration information for the specified S2LP WMBUS PACKET FORMAT.
 *         This parameter is a pointer to @ref PktWMbusInit.
 * @retval None.
 */
void S2LPPktWMbusInit(S2LPDriver *devp, PktWMbusInit* pxPktWMbusInit)
{
  uint8_t tmp;
  s_assert_param(IS_WMBUS_SUBMODE(pxPktWMbusInit->xWMbusSubmode));
  
  /* Packet format config */
  S2LPPktWMbusSetFormat(devp);
  S2LPPktCommonFilterOnCrc(devp, S_DISABLE);
  
  s_cWMbusSubmode = pxPktWMbusInit->xWMbusSubmode;
    
  if(s_cWMbusSubmode==WMBUS_SUBMODE_S1_S2_LONG_HEADER) {
    S2LPSetPreambleLength(devp, ((uint16_t)pxPktWMbusInit->cPreambleLength) + WMBUS_PREAMBLE_LEN_S1S2LONGHEADER);
    S2LPSetSyncLength(devp, WMBUS_SYNC_LEN_S1S2LONGHEADER);
    S2LPSetSyncWords(devp, WMBUS_SYNCWORD_S1S2LONGHEADER, WMBUS_SYNC_LEN_S1S2LONGHEADER);
    S2LPPacketHandlerManchester(devp, S_ENABLE);
    
    /* Constellation map setting */
  S2LPSpiReadRegisters(devp, MOD1_ADDR, 1, &tmp);
  tmp &= ~G4FSK_CONST_MAP_REGMASK;
  tmp |= (((uint8_t)2)<<4);
  S2LPSpiWriteRegisters(devp, MOD1_ADDR, 1, &tmp);
    
    S2LPSpiReadRegisters(devp, 0xF1, 1, &tmp);
    if((tmp&0xC0)==0xC0)
      S2LPPktWMbusSetPostamblePattern(devp, 0x01);
  }
  else if(s_cWMbusSubmode==WMBUS_SUBMODE_S1_M_S2_T2_OTHER_TO_METER) {
    S2LPSetPreambleLength(devp, ((uint16_t)pxPktWMbusInit->cPreambleLength) + WMBUS_PREAMBLE_LEN_S1MS2T2OTHERTOMETER);
    S2LPSetSyncLength(devp, WMBUS_SYNC_LEN_S1MS2T2OTHERTOMETER);
    S2LPSetSyncWords(devp, WMBUS_SYNCWORD_S1MS2T2OTHERTOMETER, WMBUS_SYNC_LEN_S1MS2T2OTHERTOMETER);
    S2LPPacketHandlerManchester(devp, S_ENABLE);
    /* Constellation map setting */
  S2LPSpiReadRegisters(devp, MOD1_ADDR, 1, &tmp);
  tmp &= ~G4FSK_CONST_MAP_REGMASK;
  tmp |= (((uint8_t)2)<<4);
  S2LPSpiWriteRegisters(devp, MOD1_ADDR, 1, &tmp);
    
    S2LPSpiReadRegisters(devp, 0xF1, 1, &tmp);
    if((tmp&0xC0)==0xC0)
      S2LPPktWMbusSetPostamblePattern(devp, 0x01);
  }
  else if(s_cWMbusSubmode==WMBUS_SUBMODE_T1_T2_METER_TO_OTHER) {
    S2LPSetPreambleLength(devp, ((uint16_t)pxPktWMbusInit->cPreambleLength) + WMBUS_PREAMBLE_LEN_T1T2METERTOOTHER);
    S2LPSetSyncLength(devp, WMBUS_SYNC_LEN_T1T2METERTOOTHER);
    S2LPSetSyncWords(devp, WMBUS_SYNCWORD_T1T2METERTOOTHER, WMBUS_SYNC_LEN_T1T2METERTOOTHER);
    S2LPPacketHandler3OutOf6(devp, S_ENABLE);
  }
  else if(s_cWMbusSubmode==WMBUS_SUBMODE_R2_SHORT_HEADER) {
    S2LPSetPreambleLength(devp, ((uint16_t)pxPktWMbusInit->cPreambleLength) + WMBUS_PREAMBLE_LEN_R2);
    S2LPSetSyncLength(devp, WMBUS_SYNC_LEN_R2);
    S2LPSetSyncWords(devp, WMBUS_SYNCWORD_R2, WMBUS_SYNC_LEN_R2);
    S2LPPacketHandlerManchester(devp, S_ENABLE);
    /* Constellation map setting */
  S2LPSpiReadRegisters(devp, MOD1_ADDR, 1, &tmp);
  tmp &= ~G4FSK_CONST_MAP_REGMASK;
  tmp |= (((uint8_t)2)<<4);
  S2LPSpiWriteRegisters(devp, MOD1_ADDR, 1, &tmp);
  }
  
  
  S2LPPktWMbusSetPostamble(devp, pxPktWMbusInit->cPostambleLength);
}

/**
 * @brief  Return the S2LP WMBUS packet structure according to the specified parameters in the registers.
 * @param  pxPktWMbusInit WMBUS packet init structure.
 *         This parameter is a pointer to @ref PktWMbusInit.
 * @retval None.
 */
void S2LPPktWMbusGetInfo(S2LPDriver *devp, PktWMbusInit* pxPktWMbusInit)
{
  uint16_t tmp;
  
  tmp = S2LPGetPreambleLength(devp);
  
  if(s_cWMbusSubmode==WMBUS_SUBMODE_S1_S2_LONG_HEADER) {
    if(tmp>=WMBUS_PREAMBLE_LEN_S1S2LONGHEADER)
      tmp -= WMBUS_PREAMBLE_LEN_S1S2LONGHEADER;
  }
  else if(s_cWMbusSubmode==WMBUS_SUBMODE_S1_M_S2_T2_OTHER_TO_METER) {
    if(tmp>=WMBUS_PREAMBLE_LEN_S1MS2T2OTHERTOMETER)
    tmp -= WMBUS_PREAMBLE_LEN_S1MS2T2OTHERTOMETER;
  }
  else if(s_cWMbusSubmode==WMBUS_SUBMODE_T1_T2_METER_TO_OTHER) {
    if(tmp>=WMBUS_PREAMBLE_LEN_T1T2METERTOOTHER)
    tmp -= WMBUS_PREAMBLE_LEN_T1T2METERTOOTHER;
  }
  else if(s_cWMbusSubmode==WMBUS_SUBMODE_R2_SHORT_HEADER) {
    if(tmp>=WMBUS_PREAMBLE_LEN_R2)
    tmp -= WMBUS_PREAMBLE_LEN_R2;
  }
  
  pxPktWMbusInit->cPreambleLength = (uint8_t)tmp;
  pxPktWMbusInit->cPostambleLength = S2LPPktWMbusGetPostamble(devp);
  pxPktWMbusInit->xWMbusSubmode = s_cWMbusSubmode;

}


/**
 * @brief  Configure the WMBUS packet format as the one used by S2LP.
 * @param  None.
 * @retval None.
 */
void S2LPPktWMbusSetFormat(S2LPDriver *devp)
{
  uint8_t tmpBuffer[4] = {0,0,0,0};

  /* Configure the WMBUS mode packet format and reset all the other setting */
  tmpBuffer[1] |= PKT_FORMAT_WMBUS_CODE;
  g_xStatus = S2LPSpiWriteRegisters(devp, PCKTCTRL4_ADDR, 4, tmpBuffer);

}



/**
 * @brief  Set how many chips will be used in postamble
 * @param  cPostamble the number of chip sequence.
 *         This parameter is an uint8_t.
 * @retval None.
 */
void S2LPPktWMbusSetPostamble(S2LPDriver *devp, uint8_t cPostamble)
{
  uint8_t tmp;
  g_xStatus = S2LPSpiReadRegisters(devp, PCKT_PSTMBL_ADDR, 1, &tmp);
  tmp  = (tmp & 0xC0) | cPostamble;
  g_xStatus = S2LPSpiWriteRegisters(devp, PCKT_PSTMBL_ADDR, 1, &tmp);
}


/**
 * @brief  Set the pattern of the postamble.
 * @param  cPostamble id the preamble pattern:
 *         This parameter can be '00','01','10' or '11'.
 *         This parameter is an uint8_t.
 * @retval None.
 */
void S2LPPktWMbusSetPostamblePattern(S2LPDriver *devp, uint8_t cPostamble)
{
  uint8_t tmp;
  g_xStatus = S2LPSpiReadRegisters(devp, PCKT_PSTMBL_ADDR, 1, &tmp);
  tmp  = (tmp & 0x3F) | (cPostamble<<6);
  g_xStatus = S2LPSpiWriteRegisters(devp, PCKT_PSTMBL_ADDR, 1, &tmp);
}


/**
 * @brief  Returns how many chips are used in the postamble
 * @param  None.
 * @retval uint8_t Postamble in number chip sequences.
 */
uint8_t S2LPPktWMbusGetPostamble(S2LPDriver *devp)
{
  uint8_t tmp;
  g_xStatus = S2LPSpiReadRegisters(devp, PCKT_PSTMBL_ADDR, 1, &tmp);
  return (tmp & 0x3F);
}


/**
 * @brief  Returns the pattern of the postamble.
 * @param  None.
 * @retval uint8_t Postamble in chips.
 */
uint8_t S2LPPktWMbusGetPostamblePattern(S2LPDriver *devp)
{
  uint8_t tmp;
  g_xStatus = S2LPSpiReadRegisters(devp, PCKT_PSTMBL_ADDR, 1, &tmp);
  return ((tmp & 0xC0)>>6);
}


/**
* @brief  Set the W-MBus submode.
* @param  xWMbusSubmode This parameter of @ref WMbusSubmode .
* @retval None.
*/
void S2LPPktWMbusSetSubmode(WMbusSubmode xWMbusSubmode)
{
  s_cWMbusSubmode = xWMbusSubmode;
}


/**
 * @brief  Return the WMBUS submode used.
 * @param  None.
 * @retval WMbusSubmode WMBUS submode.
 */
WMbusSubmode S2LPPktWMbusGetSubmode(void)
{
  return s_cWMbusSubmode;
}


/**
 * @brief  Sets the payload length for S2LP WMBUS packets.
 * @param  nPayloadLength payload length in bytes.
 *         This parameter is an uint16_t.
 * @retval None.
 */
void S2LPPktWMbusSetPayloadLength(S2LPDriver *devp, uint16_t nPayloadLength)
{
  uint8_t tmpBuffer[2];

  tmpBuffer[0] = (uint8_t)(nPayloadLength>>8);
  tmpBuffer[1] = (uint8_t)nPayloadLength;
  g_xStatus = S2LPSpiWriteRegisters(devp, PCKTLEN1_ADDR, 2, tmpBuffer);
}


/**
 * @brief  Return the payload length for WMBUS packets.
 * @param  None.
 * @retval uint16_t Payload length in bytes.
 */
uint16_t S2LPPktWMbusGetPayloadLength(S2LPDriver *devp)
{
  uint8_t tmpBuffer[2];
  uint16_t nPayloadLength;

  g_xStatus = S2LPSpiReadRegisters(devp, PCKTLEN1_ADDR, 2, tmpBuffer);
  nPayloadLength = (((uint16_t)tmpBuffer[0])<<8) | ((uint16_t)tmpBuffer[1]);
  
  return nPayloadLength;
}

/**
 *@}
 */

/**
 *@}
 */


/**
 *@}
 */





/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
