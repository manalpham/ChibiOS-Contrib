/**
 * @file    MCU_Interface.h
 * @author  LowPower RF BU - AMG
 * @version 1.1.0
 * @date    July 1, 2016
 * @brief   Header file for low level S2LP SPI driver.
 * @details
 *
 * This header file constitutes an interface to the SPI driver used to
 * communicate with S2LP.
 * It exports some function prototypes to write/read registers and FIFOs
 * and to send command strobes.
 * Since the S2LP libraries are totally platform independent, the implementation
 * of these functions are not provided here. The user have to implement these functions
 * taking care to keep the exported prototypes.
 *
 * These functions are:
 *
 * <ul>
 * <li>S2LPSpiInit</i>
 * <li>S2LPSpiWriteRegisters</i>
 * <li>S2LPSpiReadRegisters</i>
 * <li>S2LPSpiCommandStrobes</i>
 * <li>S2LPSpiWriteLinearFifo</i>
 * <li>S2LPSpiReadLinearFifo</i>
 * </ul>
 *
 * @note An example of SPI driver implementation is available in the <i>Sdk_Eval</i> library.
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
#ifndef __S2LP_MCU_INTERFACE_H
#define __S2LP_MCU_INTERFACE_H


/* Includes ------------------------------------------------------------------*/
#include "S2LP_Types.h"

#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup S2LP_Libraries
 * @{
 */

/** @defgroup S2LP_SPI_Driver         SPI Driver
 * @brief Header file for low level S2LP SPI driver.
 * @details See the file <i>@ref MCU_Interface.h</i> for more details.
 * @{
 */


/** @defgroup SPI_Exported_Types        SPI Exported Types
 * @{
 */

//typedef S2LPStatus StatusBytes;

/**
 * @}
 */


/** @defgroup SPI_Exported_Constants    SPI Exported Constants
 * @{
 */

/* SPIRIT1_Spi_config_Headers */
#define HEADER_WRITE_MASK     0x00                                /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01                                /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00                                /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80                                /*!< Command mask for header byte*/
#define LINEAR_FIFO_ADDRESS   0xFF                                /*!< Linear FIFO address*/

/**
 * @}
 */



/** @defgroup SPI_Exported_Macros       SPI Exported Macros
 * @{
 */

/* SPIRIT1_Spi_config_Private_Macros */
#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)                             /*!< macro to build the header byte*/
#define WRITE_HEADER        BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define READ_HEADER         BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define COMMAND_HEADER      BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command */

/**
 * @}
 */


/** @defgroup SPI_Exported_Functions    SPI Exported Functions
 * @{
 */

S2LPStatus s2lp_spi_write_regs(uint8_t addr, uint8_t n, uint8_t *regsp);
S2LPStatus s2lp_spi_read_regs(uint8_t addr, uint8_t n, uint8_t *regsp);
S2LPStatus s2lp_spi_cmd_strobes(uint8_t cmd);

/**
 * @}
 */


/** @defgroup SPI_Exported_Macros       SPI Exported Macros
 * @{
 */

#define S2LPSpiWriteRegisters(addr, n, regsp)								\
			s2lp_spi_write_regs(addr, n, regsp)
#define S2LPSpiReadRegisters(addr, n, regsp)       							\
			s2lp_spi_read_regs(addr, n, regsp)
#define S2LPSpiCommandStrobes(cmd)		                         			\
			s2lp_spi_cmd_strobes(cmd)

/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */



#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
