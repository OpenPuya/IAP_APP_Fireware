/**
  ******************************************************************************
  * @file    py32f4xx_ll_sdio.h
  * @author  MCU Application Team
  * @brief   Header file of SDIO HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PY32F4XX_LL_SDIO_H
#define PY32F4XX_LL_SDIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f4xx_hal_def.h"

/** @addtogroup PY32F4xx_Driver
  * @{
  */

/** @addtogroup SDIO_LL
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SDIO_Exported_Types SDIO Exported Types
  * @{
  */
  
/**
  * @brief  SDIO Clock configuration structure definition
  */
typedef struct
{    
  uint32_t ClockDiv;         /*!< Specifies the clock frequency of the SDIO controller
                                 (default is SDIO_INIT_CLK_DIV_DEFAULT or SDIO_TRANSFER_CLK_DIV_DEFAULT).
                                 This parameter can be a value between 0x00 and 0xFF.                      */

  uint32_t ClockPowerSave;   /*!< Specifies whether SDMMC Clock output is enabled or
                                 disabled when the bus is idle.
                                 This parameter can be a value of @ref SDMMC_LL_Clock_Power_Save           */

  uint32_t BusWide;          /*!< Specifies the SDIO bus width.
                                 This parameter can be a value of @ref SDIO_LL_Bus_Wide */

  uint32_t SMPClock;         /*!< Pre-sampling clock selection.
                                 This parameter can be a value of @ref SDIO_LL_SMP_Clock */
  
  uint32_t SMPState;         /*!< Set Pre-Sampling Status.
                                 This parameter can be a value of @ref SDIO_LL_SMP_State */
  
  uint32_t CLockOutput;      /*!< Select the output SD clock.
                                      This parameter can be a value of @ref SDIO_LL_Clock_Output */
} SDIO_InitTypeDef;               


/**
  * @brief  SDIO CMD configuration structure definition
  */
typedef struct
{
  uint32_t Argument;       /*!< Specifies the SDIO command argument which is sent
                              to a card as part of a command message. If a command
                              contains an argument, it must be loaded into this register
                              before writing the command to the command register            */

  uint32_t CmdIndex;       /*!< Specifies the SDIO command index. It must be lower than 64. */
  
  uint32_t Response;       /*!< Specifies the SDIO response type.
                              This parameter can be a value of @ref SDIO_LL_Response_Type */

  uint32_t Wait;           /*!< Specifies whether SDIO wait for interrupt request is enabled or disabled.
                                 This parameter can be a value of @ref SDIO_LL_Wait_Transfer_State */
    
  uint32_t InitSignal;     /*!< Send initialization sequence before sending this command.
                                 This parameter can be a value of @ref SDIO_LL_Initsignal */

  uint32_t Response_CRC;   /*!< Check CRC response.
                                 This parameter can be a value of @ref SDIO_LL_Response_CRC */

  uint32_t DataTransfer;   /*!< Expected data transfer.
                                 This parameter can be a value of @ref SDIO_LL_DataTransfer */

  uint32_t CPSM;           /*!< Specifies whether SDIO Command path state machine (CPSM)
                                     is enabled or disabled.
                                     This parameter can be a value of @ref SDIO_LL_CPSM_State */
} SDIO_CmdInitTypeDef;

/**
  * @brief  SDIO Data configuration structure definition
  */
typedef struct
{
  uint32_t DataTimeOut;    /*!< Specifies the data timeout period in card bus clock periods. */

  uint32_t DataLength;     /*!< Specifies the number of data bytes to be transferred. */
 
  uint32_t DataBlockSize;  /*!< Specifies the data block size for block transfer.
                                This parameter can be a value of @ref SDIO_LL_Data_Block_Size */
 
  uint32_t TransferDir;    /*!< Specifies the data transfer direction, whether the transfer
                                is a read or write.
                                This parameter can be a value of @ref SDIO_LL_Transfer_Direction */
 
  uint32_t TransferMode;   /*!< Specifies whether data transfer is in stream or block mode.
                                     This parameter can be a value of @ref SDIO_LL_Transfer_Type */

} SDIO_DataInitTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SDIO_Exported_Constants SDIO Exported Constants
  * @{
  */

/** 
  * @brief SDIO Commands Index 
  */
#define SDIO_CMD_GO_IDLE_STATE                       ((uint8_t)0U)   /*!< Resets the SD memory card.                                                               */
#define SDIO_CMD_SEND_OP_COND                        ((uint8_t)1U)   /*!< Sends host capacity support information and activates the card's initialization process. */
#define SDIO_CMD_ALL_SEND_CID                        ((uint8_t)2U)   /*!< Asks any card connected to the host to send the CID numbers on the CMD line.             */
#define SDIO_CMD_SET_REL_ADDR                        ((uint8_t)3U)   /*!< Asks the card to publish a new relative address (RCA).                                   */
#define SDIO_CMD_SET_DSR                             ((uint8_t)4U)   /*!< Programs the DSR of all cards.                                                           */
#define SDIO_CMD_SDMMC_SEN_OP_COND                   ((uint8_t)5U)   /*!< Sends host capacity support information (HCS) and asks the accessed card to send its
                                                                           operating condition register (OCR) content in the response on the CMD line.             */
#define SDIO_CMD_HS_SWITCH                           ((uint8_t)6U)   /*!< Checks switchable function (mode 0) and switch card function (mode 1).                   */
#define SDIO_CMD_SEL_DESEL_CARD                      ((uint8_t)7U)   /*!< Selects the card by its own relative address and gets deselected by any other address    */
#define SDIO_CMD_HS_SEND_EXT_CSD                     ((uint8_t)8U)   /*!< Sends SD Memory Card interface condition, which includes host supply voltage information
                                                                           and asks the card whether card supports voltage.                                        */
#define SDIO_CMD_SEND_CSD                            ((uint8_t)9U)   /*!< Addressed card sends its card specific data (CSD) on the CMD line.                       */
#define SDIO_CMD_SEND_CID                            ((uint8_t)10U)  /*!< Addressed card sends its card identification (CID) on the CMD line.                      */
#define SDIO_CMD_READ_DAT_UNTIL_STOP                 ((uint8_t)11U)  /*!< SD card doesn't support it.                                                              */
#define SDIO_CMD_STOP_TRANSMISSION                   ((uint8_t)12U)  /*!< Forces the card to stop transmission.                                                    */
#define SDIO_CMD_SEND_STATUS                         ((uint8_t)13U)  /*!< Addressed card sends its status register.                                                */
#define SDIO_CMD_HS_BUSTEST_READ                     ((uint8_t)14U)  /*!< Reserved                                                                                 */
#define SDIO_CMD_GO_INACTIVE_STATE                   ((uint8_t)15U)  /*!< Sends an addressed card into the inactive state.                                         */
#define SDIO_CMD_SET_BLOCKLEN                        ((uint8_t)16U)  /*!< Sets the block length (in bytes for SDSC) for all following block commands
                                                                           (read, write, lock). Default block length is fixed to 512 Bytes. Not effective 
                                                                           for SDHS and SDXC.                                                                      */
#define SDIO_CMD_READ_SINGLE_BLOCK                   ((uint8_t)17U)  /*!< Reads single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of
                                                                           fixed 512 bytes in case of SDHC and SDXC.                                               */
#define SDIO_CMD_READ_MULT_BLOCK                     ((uint8_t)18U)  /*!< Continuously transfers data blocks from card to host until interrupted by
                                                                           STOP_TRANSMISSION command.                                                              */
#define SDIO_CMD_HS_BUSTEST_WRITE                    ((uint8_t)19U)  /*!< 64 bytes tuning pattern is sent for SDR50 and SDR104.                                    */
#define SDIO_CMD_WRITE_DAT_UNTIL_STOP                ((uint8_t)20U)  /*!< Speed class control command.                                                             */
#define SDIO_CMD_SET_BLOCK_COUNT                     ((uint8_t)23U)  /*!< Specify block count for CMD18 and CMD25.                                                 */
#define SDIO_CMD_WRITE_SINGLE_BLOCK                  ((uint8_t)24U)  /*!< Writes single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of
                                                                           fixed 512 bytes in case of SDHC and SDXC.                                               */
#define SDIO_CMD_WRITE_MULT_BLOCK                    ((uint8_t)25U)  /*!< Continuously writes blocks of data until a STOP_TRANSMISSION follows.                    */
#define SDIO_CMD_PROG_CID                            ((uint8_t)26U)  /*!< Reserved for manufacturers.                                                              */
#define SDIO_CMD_PROG_CSD                            ((uint8_t)27U)  /*!< Programming of the programmable bits of the CSD.                                         */
#define SDIO_CMD_SET_WRITE_PROT                      ((uint8_t)28U)  /*!< Sets the write protection bit of the addressed group.                                    */
#define SDIO_CMD_CLR_WRITE_PROT                      ((uint8_t)29U)  /*!< Clears the write protection bit of the addressed group.                                  */
#define SDIO_CMD_SEND_WRITE_PROT                     ((uint8_t)30U)  /*!< Asks the card to send the status of the write protection bits.                           */
#define SDIO_CMD_SD_ERASE_GRP_START                  ((uint8_t)32U)  /*!< Sets the address of the first write block to be erased. (For SD card only).              */
#define SDIO_CMD_SD_ERASE_GRP_END                    ((uint8_t)33U)  /*!< Sets the address of the last write block of the continuous range to be erased.           */
#define SDIO_CMD_ERASE_GRP_START                     ((uint8_t)35U)  /*!< Sets the address of the first write block to be erased. Reserved for each command
                                                                           system set by switch function command (CMD6).                                           */
#define SDIO_CMD_ERASE_GRP_END                       ((uint8_t)36U)  /*!< Sets the address of the last write block of the continuous range to be erased.
                                                                           Reserved for each command system set by switch function command (CMD6).                 */
#define SDIO_CMD_ERASE                               ((uint8_t)38U)  /*!< Reserved for SD security applications.                                                   */
#define SDIO_CMD_FAST_IO                             ((uint8_t)39U)  /*!< SD card doesn't support it (Reserved).                                                   */
#define SDIO_CMD_GO_IRQ_STATE                        ((uint8_t)40U)  /*!< SD card doesn't support it (Reserved).                                                   */
#define SDIO_CMD_LOCK_UNLOCK                         ((uint8_t)42U)  /*!< Sets/resets the password or lock/unlock the card. The size of the data block is set by
                                                                           the SET_BLOCK_LEN command.                                                              */
#define SDIO_CMD_APP_CMD                             ((uint8_t)55U)  /*!< Indicates to the card that the next command is an application specific command rather
                                                                           than a standard command.                                                                */
#define SDIO_CMD_GEN_CMD                             ((uint8_t)56U)  /*!< Used either to transfer a data block to the card or to get a data block from the card
                                                                           for general purpose/application specific commands.                                      */
#define SDIO_CMD_NO_CMD                              ((uint8_t)64U)  /*!< No command                                                                               */

/** 
  * @brief Following commands are SD Card Specific commands.
  *        SDMMC_APP_CMD should be sent before sending these commands. 
  */
#define SDIO_CMD_APP_SD_SET_BUSWIDTH                           6U    /*!< (ACMD6) Defines the data bus width to be used for data transfer. The allowed data bus
                                                                            widths are given in SCR register.                                                       */
#define SDIO_CMD_SD_APP_STATUS                                 13U   /*!< (ACMD13) Sends the SD status.                                                            */
#define SDIO_CMD_SD_APP_SEND_NUM_WRITE_BLOCKS                  22U   /*!< (ACMD22) Sends the number of the written (without errors) write blocks. Responds with
                                                                           32bit+CRC data block.                                                                    */
#define SDIO_CMD_SD_APP_OP_COND                                41U   /*!< (ACMD41) Sends host capacity support information (HCS) and asks the accessed card to
                                                                           send its operating condition register (OCR) content in the response on the CMD line.     */
#define SDIO_CMD_SD_APP_SET_CLR_CARD_DETECT                    42U   /*!< (ACMD42) Connect/Disconnect the 50 KOhm pull-up resistor on CD/DAT3 (pin 1) of the card  */
#define SDIO_CMD_SD_APP_SEND_SCR                               51U   /*!< Reads the SD Configuration Register (SCR).                                               */
#define SDIO_CMD_SDMMC_RW_DIRECT                               52U   /*!< For SD I/O card only, reserved for security specification.                               */
#define SDIO_CMD_SDMMC_RW_EXTENDED                             53U   /*!< For SD I/O card only, reserved for security specification.                               */

/** 
  * @brief Following commands are SD Card Specific security commands.
  *        SDMMC_CMD_APP_CMD should be sent before sending these commands. 
  */
#define SDIO_CMD_SD_APP_GET_MKB                                43U
#define SDIO_CMD_SD_APP_GET_MID                                44U
#define SDIO_CMD_SD_APP_SET_CER_RN1                            45U
#define SDIO_CMD_SD_APP_GET_CER_RN2                            46U
#define SDIO_CMD_SD_APP_SET_CER_RES2                           47U
#define SDIO_CMD_SD_APP_GET_CER_RES1                           48U
#define SDIO_CMD_SD_APP_SECURE_READ_MULTIPLE_BLOCK             18U
#define SDIO_CMD_SD_APP_SECURE_WRITE_MULTIPLE_BLOCK            25U
#define SDIO_CMD_SD_APP_SECURE_ERASE                           38U
#define SDIO_CMD_SD_APP_CHANGE_SECURE_AREA                     49U
#define SDIO_CMD_SD_APP_SECURE_WRITE_MKB                       48U

/** @defgroup SDIO Initialization Frequency
  * @{
  */
#define SDIO_INIT_CLK_DIV_DEFAULT     ((uint8_t)0xA)    /* 48MHz / (SDMMC_INIT_CLK_DIV * 2) = 400KHz */
#define SDIO_TRANSFER_CLK_DIV_DEFAULT ((uint8_t)0x1)     /* 48MHz / (SDMMC_INIT_CLK_DIV * 2) = 24MHz */
/**
  * @}
  */

/** @defgroup SDIO_LL_Bus_Wide Bus Width
  * @{
  */
#define SDIO_BUS_WIDE_1B                       ((uint32_t)0x00000000)
#define SDIO_BUS_WIDE_4B                       (SDIO_CLKCR_WIDBUS_0)
#define SDIO_BUS_WIDE_8B                       (SDIO_CLKCR_WIDBUS_1)
/**
  * @}
  */

/** @defgroup SDIO_LL_Clock_Power_Save Clock Power Saving
  * @{
  */
#define SDIO_CLOCK_POWER_SAVE_DISABLE           ((uint32_t)0x00000000U)
#define SDIO_CLOCK_POWER_SAVE_ENABLE            SDIO_CLKCR_PWRSAV
/**
  * @}
  */

/** @defgroup SDIO_LL_Power_State Power state.
  * @{
  */
#define SDIO_POWER_STATE_OFF           ((uint32_t)0x00000000U)
#define SDIO_POWER_STATE_ON            SDIO_POWER_PWRCTRL
/**
  * @}
  */

/** @defgroup SDIO_LL_SMP_Clock Select a pre-sampled clock source.
  * @{
  */
#define SDIO_SMP_CLOCK_SDCLK_OFFSET270          ((uint32_t)0x00000000U)
#define SDIO_SMP_CLOCK_SDCLK_DEFAULT            SDIO_CLKCR_SMPCLKSEL
/**
  * @}
  */

/** @defgroup SDIO_LL_SMP_State Set Pre-Sampling Status.
  * @{
  */
#define SDIO_SMP_DISABLE                        ((uint32_t)0x00000000U)
#define SDIO_SMP_ENABLE                         SDIO_CLKCR_SMPEN
/**
  * @}
  */

/** @defgroup SDIO_LL_Clock_Output Select SDIO Output Clock.
  * @{
  */
#define SDIO_CLOCK_OUTPUT_OFFSET90              ((uint32_t)0x00000000U)
#define SDIO_CLOCK_OUTPUT_OFFSET180             SDIO_CLKCR_CKSEL
/**
  * @}
  */

/** @defgroup SDIO_LL_Response_Type Set Response Length.
  * @{
  */
#define SDIO_RESPONSE_NO                      ((uint32_t)0x00000000)
#define SDIO_RESPONSE_SHORT                   SDIO_CMD_WAITRESP
#define SDIO_RESPONSE_LONG                    (SDIO_CMD_WAITRESP| SDIO_CMD_RESPLEN)
/**
  * @}
  */

/** @defgroup SDIO_LL_Wait_Transfer_State CPSM Waits for Data Transfer to End.
  * @{
  */
#define SDIO_WAIT_NO                     ((uint32_t)0x00000000)
#define SDIO_WAIT_PEND                   SDIO_CMD_WAITPEND
/**
  * @}
  */

/** @defgroup SDIO_LL_Initsignal Whether to send initialization sequence.
  * @{
  */
#define SDIO_INITSIGNAL_DISABLE          ((uint32_t)0x00000000) 
#define SDIO_INITSIGNAL_ENABLE           SDIO_CMD_AUTOINIT
/**
  * @}
  */
  
/** @defgroup SDIO_LL_Response_CRC Check response CRC.
  * @{
  */
#define SDIO_RESPONSE_CRC_DISABLE        ((uint32_t)0x00000000) 
#define SDIO_RESPONSE_CRC_ENABLE         SDIO_CMD_CHECKRESPCRC
/**
  * @}
  */
  
/** @defgroup SDIO_LL_DataTransfer Expected data transfer
  * @{
  */
#define SDIO_DATATRANSFER_DISABLE             ((uint32_t)0x00000000) 
#define SDIO_DATATRANSFER_ENABLE              SDIO_CMD_DEXPECT
/**
  * @}
  */
  
/** @defgroup SDIO_LL_CPSM_State CPSM State
  * @{
  */
#define SDIO_CPSM_DISABLE                     ((uint32_t)0x00000000)
#define SDIO_CPSM_ENABLE                      SDIO_CMD_STARTCMD
/**
  * @}
  */
  

/** @defgroup SDIO_LL_Data_Block_Size Data Block Size
  * @{
  */
#define SDIO_DATABLOCKSIZE_1B               ((uint32_t)0x00000001)
#define SDIO_DATABLOCKSIZE_2B               ((uint32_t)0x00000002)
#define SDIO_DATABLOCKSIZE_4B               ((uint32_t)0x00000004)
#define SDIO_DATABLOCKSIZE_8B               ((uint32_t)0x00000008)
#define SDIO_DATABLOCKSIZE_16B              ((uint32_t)0x00000010)
#define SDIO_DATABLOCKSIZE_32B              ((uint32_t)0x00000020)
#define SDIO_DATABLOCKSIZE_64B              ((uint32_t)0x00000040)
#define SDIO_DATABLOCKSIZE_128B             ((uint32_t)0x00000080)
#define SDIO_DATABLOCKSIZE_256B             ((uint32_t)0x00000100)
#define SDIO_DATABLOCKSIZE_512B             ((uint32_t)0x00000200)
#define SDIO_DATABLOCKSIZE_1024B            ((uint32_t)0x00000400)
#define SDIO_DATABLOCKSIZE_2048B            ((uint32_t)0x00000800)
#define SDIO_DATABLOCKSIZE_4096B            ((uint32_t)0x00001000)
#define SDIO_DATABLOCKSIZE_8192B            ((uint32_t)0x00002000)
#define SDIO_DATABLOCKSIZE_16384B           ((uint32_t)0x00004000)
/**
  * @}
  */

/** @defgroup SDIO_LL_Transfer_Direction Transfer Direction
  * @{
  */
#define SDIO_TRANSFERDIR_TO_SDIO            ((uint32_t)0x00000000)
#define SDIO_TRANSFERDIR_TO_CARD            SDIO_CMD_DIR
/**
  * @}
  */

/** @defgroup SDIO_LL_Transfer_Type Transfer Type
  * @{
  */
#define SDIO_TRANSFERMODE_BLOCK             ((uint32_t)0x00000000)
#define SDIO_TRANSFERMODE_STREAM            SDIO_CMD_DTMODE
/**
  * @}
  */

/** @defgroup FIFO Threshold
  * @{
  */
#define SDIO_FIFO_WMARK_1                    ((uint32_t)0x00000001)
#define SDIO_FIFO_WMARK_2                    ((uint32_t)0x00000002)
#define SDIO_FIFO_WMARK_3                    ((uint32_t)0x00000003)
#define SDIO_FIFO_WMARK_4                    ((uint32_t)0x00000004)
#define SDIO_FIFO_WMARK_5                    ((uint32_t)0x00000005)
#define SDIO_FIFO_WMARK_6                    ((uint32_t)0x00000006)
#define SDIO_FIFO_WMARK_7                    ((uint32_t)0x00000007)
#define SDIO_FIFO_WMARK_8                    ((uint32_t)0x00000008)
#define SDIO_FIFO_WMARK_9                    ((uint32_t)0x00000009)
#define SDIO_FIFO_WMARK_10                   ((uint32_t)0x00000010)
#define SDIO_FIFO_WMARK_11                   ((uint32_t)0x00000011)
#define SDIO_FIFO_WMARK_12                   ((uint32_t)0x00000012)
#define SDIO_FIFO_WMARK_13                   ((uint32_t)0x00000013)
#define SDIO_FIFO_WMARK_14                   ((uint32_t)0x00000014)
#define SDIO_FIFO_WMARK_15                   ((uint32_t)0x00000015)
#define SDIO_FIFO_WMARK_16                   ((uint32_t)0x00000016)
/**
  * @}
  */

/** @defgroup Interrupt pending
  * @{
  */
#define SDIO_IT_CD                           ((uint32_t)0x00000001)
#define SDIO_IT_RE                           ((uint32_t)0x00000002)
#define SDIO_IT_CMDD                         ((uint32_t)0x00000004)
#define SDIO_IT_DTO                          ((uint32_t)0x00000008)
#define SDIO_IT_TXDR                         ((uint32_t)0x00000010)
#define SDIO_IT_RXDR                         ((uint32_t)0x00000020)
#define SDIO_IT_RCRC                         ((uint32_t)0x00000040)
#define SDIO_IT_DCRC                         ((uint32_t)0x00000080)
#define SDIO_IT_RTO                          ((uint32_t)0x00000100)
#define SDIO_IT_DRTO                         ((uint32_t)0x00000200)
#define SDIO_IT_HTO                          ((uint32_t)0x00000400)
#define SDIO_IT_FRUN                         ((uint32_t)0x00000800)
#define SDIO_IT_HLE                          ((uint32_t)0x00001000)
#define SDIO_IT_SBE                          ((uint32_t)0x00002000)
#define SDIO_IT_ACD                          ((uint32_t)0x00004000)
#define SDIO_IT_EBE                          ((uint32_t)0x00008000)
#define SDIO_IT_FROM_CARD                    ((uint32_t)0x00010000)
/**
  * @}
  */

/** @defgroup SDIO_Exported_Macros SDIO Exported Macros
  * @{
  */
  
/**
  * @brief  Enable CMD pullup.
  * @retval None
  */    
#define __SDIO_OD_PULLUP_ENABLE()    SET_BIT(SDIO->CTRL, SDIO_CTRL_ODPUEN)
  
/**
  * @brief  Disable CMD pullup.
  * @retval None
  */    
#define __SDIO_OD_PULLUP_DISABLE()    CLEAR_BIT(SDIO->CTRL, SDIO_CTRL_ODPUEN)
  
/**
  * @brief  Enables the SDIO card to suspend queues.
  * @retval None
  */    
#define __SDIO_PEND_QUEUSE()         SET_BIT(SDIO->CTRL, SDIO_CTRL_ABORTRD)

/**
  * @brief  Send automatic IRQ response.
  * @retval None
  */    
#define __SDIO_SEND_IRQ()            SET_BIT(SDIO->CTRL, SDIO_CTRL_AUTOIRQRESP)

/**
  * @brief  Send read wait to SDIO card
  * @retval None
  */    
#define __SDIO_SEND_READ_WAIT()      SET_BIT(SDIO->CTRL, SDIO_CTRL_READWAIT)

/**
  * @brief  Reset FIFO
  * @retval None
  */    
#define __SDIO_FIFO_RESET()      SET_BIT(SDIO->CTRL, SDIO_CTRL_FIFORST)

/**
  * @brief  Reset SDIO controller
  * @retval None
  */    
#define __SDIO_CONTR_RESET()      SET_BIT(SDIO->CTRL, SDIO_CTRL_SDIORST)

/**
  * @brief  Send CCSD to CE-ATA.
  * @retval None
  */    
#define __SDIO_CEATA_SEND_CCSD()    SET_BIT(SDIO->CTRL, SDIO_CTRL_CCSDEN)
  
/**
  * @brief  Automatically send STOP after sending CCSD to CE-ATA.
  * @retval None
  */    
#define __SDIO_CEATA_SEND_STOP()    SET_BIT(SDIO->CTRL, SDIO_CTRL_AUTOSTOPCCSD)

/**
  * @brief  Enable the CE-ATA device interrupt.
  * @retval None
  */    
#define __SDIO_CEATA_DEVICE_ENABLE_IT()    SET_BIT(SDIO->CTRL, SDIO_CTRL_CEATAINTEN)

/**
  * @brief  Disable the CE-ATA device interrupt.
  * @retval None
  */  
#define __SDIO_CEATA_DEVICE_DISABLE_IT()   CLEAR_BIT(SDIO->CTRL, SDIO_CTRL_CEATAINTEN)

/**
  * @brief  Enable the CE-ATA interrupt.
  * @retval None
  */    
#define __SDIO_CEATA_ENABLE_IT()    SET_BIT(SDIO->CMD, SDIO_CMD_IEN)

/**
  * @brief  Disable the CE-ATA interrupt.
  * @retval None
  */  
#define __SDIO_CEATA_DISABLE_IT()   CLEAR_BIT(SDIO->CMD, SDIO_CMD_IEN)

/**
  * @brief  Enable send CE-ATA command (CMD61).
  * @retval None
  */  
#define __SDIO_CEATA_SENDCMD_ENABLE()   SET_BIT(SDIO->CMD, SDIO_CMD_STARTCMD)

/**
  * @brief  Disable send CE-ATA command (CMD61).
  * @retval None
  */  
#define __SDIO_CEATA_SENDCMD_DISABLE()   CLEAR_BIT(SDIO->CMD, SDIO_CMD_STARTCMD)

/**
  * @brief  Force start operation.
  * @retval None
  */  
#define __SDIO_EMMC_BOOT_MODE_FORCE()   SET_BIT(SDIO->CMD, SDIO_CMD_BOOTMODE)

/**
  * @brief  Backup startup operation.
  * @retval None
  */  
#define __SDIO_EMMC_BOOT_MODE_BACKUP()   CLEAR_BIT(SDIO->CMD, SDIO_CMD_BOOTMODE)

/**
  * @brief  Enable boot startup.
  * @retval None
  */  
#define __SDIO_EMMC_BOOT_ENABLE()   MODIFY_REG(SDIO->CMD, (SDIO_CMD_BOOTEN| SDIO_CMD_BOOTDIS), \
                                                          (SDIO_CMD_BOOTEN| ~SDIO_CMD_BOOTDIS))

/**
  * @brief  Disable boot startup.
  * @retval None
  */  
#define __SDIO_EMMC_BOOT_DISABLE()   MODIFY_REG(SDIO->CMD, (SDIO_CMD_BOOTEN| SDIO_CMD_BOOTDIS), \
                                                           (~SDIO_CMD_BOOTEN| SDIO_CMD_BOOTDIS))

/**
  * @brief  Reset boot startup.
  * @retval None
  */  
#define __SDIO_EMMC_BOOT_RESET()   CLEAR_BIT(SDIO->CMD,(SDIO_CMD_BOOTEN| SDIO_CMD_BOOTDIS))

/**
  * @brief  Expect boot ack ensable.
  * @retval None
  */  
#define __SDIO_EMMC_BOOT_ACK_ENABLE()   SET_BIT(SDIO->CMD, SDIO_CMD_BOOTMODE)

/**
  * @brief  Expect boot ack disable.
  * @retval None
  */  
#define __SDIO_EMMC_BOOT_ACK_DISABLE()   CLEAR_BIT(SDIO->CMD, SDIO_CMD_BOOTMODE)
/**
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/** @addtogroup SDIO_Exported_Functions
  * @{
  */

/*  Function used to set the SDIO configuration to the default reset state ****/
void SDIO_DeInit(void);
void SDIO_Init(SDIO_TypeDef *SDIOx, SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_ClockCmd(SDIO_TypeDef *SDIOx,FunctionalState NewState);
void SDIO_SetClock(SDIO_TypeDef *SDIOx,uint32_t SDIO_ClkDiv);
void SDIO_SetCardBusWidth(SDIO_TypeDef *SDIOx,uint32_t SDIO_Width);
void SDIO_SetPowerState(SDIO_TypeDef *SDIOx,uint32_t SDIO_PowerState);
uint32_t SDIO_GetPowerState(SDIO_TypeDef *SDIOx);

void SDIO_IsAutoStopCmd(SDIO_TypeDef *SDIOx,FunctionalState NewState);
/* Command path state machine (CPSM) management functions *********************/
void SDIO_SendCommand(SDIO_TypeDef *SDIOx,SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
void SDIO_CmdStructInit(SDIO_CmdInitTypeDef* SDIO_CmdInitStruct);
uint32_t SDIO_GetResponse(SDIO_TypeDef *SDIOx,uint32_t SDIO_RESP);

/* Data path state machine (DPSM) management functions ************************/
void SDIO_DataConfig(SDIO_TypeDef *SDIOx,SDIO_DataInitTypeDef* SDIO_DataInitStruct);
void SDIO_DataStructInit(SDIO_DataInitTypeDef* SDIO_DataInitStruct);

/* FIFO Threshold management functions ************************/
void SDIO_SetTxWaterMark(SDIO_TypeDef *SDIOx,uint16_t SDIO_WaterMark);
void SDIO_SetRxWaterMark(SDIO_TypeDef *SDIOx,uint16_t SDIO_WaterMark);
uint16_t SDIO_GetFifoCount(SDIO_TypeDef *SDIOx);
uint32_t SDIO_IsFifoEmpty(SDIO_TypeDef *SDIOx);
uint32_t SDIO_IsFifoFull(SDIO_TypeDef *SDIOx);
uint32_t SDIO_ReadData(SDIO_TypeDef * SDIOx);
void SDIO_WriteData(SDIO_TypeDef * SDIOx,uint32_t Data);

/* Status management functions ************************/
uint32_t SDIO_GetFIFOCount(SDIO_TypeDef * SDIOx);
uint16_t SDIO_GetCmdState(SDIO_TypeDef *SDIOx);
uint32_t SDIO_IsCardBusy(SDIO_TypeDef *SDIOx);
uint8_t SDIO_GetRspIdx(SDIO_TypeDef *SDIOx);

/* DMA transfers management functions *****************************************/
void SDIO_DMACmd(SDIO_TypeDef * SDIOx,FunctionalState NewState);

/* Interrupts and flags management functions **********************************/
void SDIO_ITCmd(SDIO_TypeDef * SDIOx,FunctionalState NewState);
void SDIO_ITConfig(SDIO_TypeDef * SDIOx,uint32_t SDIO_IT, FunctionalState NewState);
ITStatus SDIO_GetITStatus(SDIO_TypeDef * SDIOx,uint32_t SDIO_IT);
void SDIO_ClearITStatus(SDIO_TypeDef * SDIOx,uint32_t SDIO_IT);

/* Private functions -----------------------------------------------------------*/


/**
  * @}
  */
  
/** @addtogroup SDIO_Private_Macros
  * @{
  */
#define IS_SDIO_BUS_WIDE(WIDE)              (((WIDE) == SDIO_BUS_WIDE_1B) || \
                                             ((WIDE) == SDIO_BUS_WIDE_4B) || \
                                             ((WIDE) == SDIO_BUS_WIDE_8B))

#define IS_SDIO_CLOCK_POWER_SAVE(SAVE)      (((SAVE) == SDIO_CLOCK_POWER_SAVE_DISABLE) || \
                                             ((SAVE) == SDIO_CLOCK_POWER_SAVE_ENABLE ))

#define IS_SDIO_SMP_CLOCK(CLOCK)            (((CLOCK) == SDIO_SMP_CLOCK_SDCLK_OFFSET270) || \
                                             ((CLOCK) == SDIO_SMP_CLOCK_SDCLK_DEFAULT  ))

#define IS_SDIO_SMP_STATE(STATE)            (((STATE) == SDIO_SMP_DISABLE) || \
                                             ((STATE) == SDIO_SMP_ENABLE ))
                                            
#define IS_SDIO_CLOCK_Output(CLOCK)         (((CLOCK) == SDIO_CLOCK_OUTPUT_OFFSET90 ) || \
                                             ((CLOCK) == SDIO_CLOCK_OUTPUT_OFFSET180))

#define IS_SDIO_RESPONSE_TYPE(TYPE)         (((TYPE) == SDIO_RESPONSE_NO   ) || \
                                             ((TYPE) == SDIO_RESPONSE_SHORT) || \
                                             ((TYPE) == SDIO_RESPONSE_LONG ))

#define IS_SDIO_WAIT_INTERRUPT_STATE(STATE) (((STATE) == SDIO_WAIT_NO  ) || \
                                             ((STATE) == SDIO_WAIT_PEND))

#define IS_SDIO_INITSIGNAL(STATE)          (((STATE) == SDIO_INITSIGNAL_DISABLE) || \
                                            ((STATE) == SDIO_INITSIGNAL_ENABLE ))

#define IS_SDIO_RESPONSE_CRC(STATE)        (((STATE) == SDIO_RESPONSE_CRC_DISABLE) || \
                                            ((STATE) == SDIO_RESPONSE_CRC_ENABLE ))

#define IS_SDIO_DATATRANSFER(STATE)        (((STATE) == SDIO_DATATRANSFER_DISABLE) || \
                                            ((STATE) == SDIO_DATATRANSFER_ENABLE ))

#define SDIO_CPSM_STATE(STATE)             (((STATE) == SDIO_CPSM_DISABLE) || \
                                            ((STATE) == SDIO_CPSM_ENABLE ))

#define SDIO_DATA_BLOCK_SIZE(SIZE)         (((SIZE) == SDIO_DATABLOCKSIZE_1B    ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_2B    ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_4B    ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_8B    ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_16B   ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_32B   ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_64B   ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_128B  ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_256B  ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_512B  ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_1024B ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_2048B ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_4096B ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_8192B ) || \
                                            ((SIZE) == SDIO_DATABLOCKSIZE_16384B))

#define SDIO_TRANSFER_DIRECTION(DIR)       (((DIR) == SDIO_TRANSFERDIR_TO_HOST) || \
                                            ((DIR) == SDIO_TRANSFERDIR_TO_CARD))

#define SDIO_TRANSFER_TYPE(TYPE)           (((TYPE) == SDIO_TRANSFERMODE_BLOCK ) || \
                                            ((TYPE) == SDIO_TRANSFERMODE_STREAM))

/**
  * @}
  */
  
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

#endif /* __PY32F4XX_LL_SDIO_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
