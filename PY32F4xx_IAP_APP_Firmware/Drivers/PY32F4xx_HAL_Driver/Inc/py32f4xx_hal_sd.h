/**
  ******************************************************************************
  * @file    py32f4xx_hal_sd.h
  * @author  MCU Application Team
  * @brief   Header file of SD HAL module.
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
#ifndef PY32F4XX_HAL_SD_H
#define PY32F4XX_HAL_SD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f4xx_ll_sdio.h"

/** @addtogroup PY32F4xx_HAL_Driver
  * @{
  */

/** @addtogroup SD
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SD_Exported_Types SD Exported Types
  * @{
  */
  
extern DMA_HandleTypeDef SD_DMA_ReadBlockHandle;
extern DMA_HandleTypeDef SD_DMA_WriteBlockHandle;

/**
  * @brief  SDIO Transfer state
  */
typedef enum
{
  SD_TRANSFER_OK  = 0,
  SD_TRANSFER_BUSY = 1,
  SD_TRANSFER_ERROR
} SDTransferState;
  
/**
  * @brief  SD card CSD register type
  */
typedef struct
{
  uint8_t  CSDStruct;            /*!< CSD structure */
  uint8_t  SysSpecVersion;       /*!< System specification version */
  uint8_t  Reserved1;            /*!< Reserved */
  uint8_t  TAAC;                 /*!< Data read access-time 1 */
  uint8_t  NSAC;                 /*!< Data read access-time 2 in CLK cycles */
  uint8_t  MaxBusClkFrec;        /*!< Max. bus clock frequency */
  uint16_t CardComdClasses;      /*!< Card command classes */
  uint8_t  RdBlockLen;           /*!< Max. read data block length */
  uint8_t  PartBlockRead;        /*!< Partial blocks for read allowed */
  uint8_t  WrBlockMisalign;      /*!< Write block misalignment */
  uint8_t  RdBlockMisalign;      /*!< Read block misalignment */
  uint8_t  DSRImpl;              /*!< DSR implemented */
  uint8_t  Reserved2;            /*!< Reserved */
  uint32_t DeviceSize;           /*!< Device Size */
  uint8_t  MaxRdCurrentVDDMin;   /*!< Max. read current @ VDD min */
  uint8_t  MaxRdCurrentVDDMax;   /*!< Max. read current @ VDD max */
  uint8_t  MaxWrCurrentVDDMin;   /*!< Max. write current @ VDD min */
  uint8_t  MaxWrCurrentVDDMax;   /*!< Max. write current @ VDD max */
  uint8_t  DeviceSizeMul;        /*!< Device size multiplier */
  uint8_t  EraseGrSize;          /*!< Erase group size */
  uint8_t  EraseGrMul;           /*!< Erase group size multiplier */
  uint8_t  WrProtectGrSize;      /*!< Write protect group size */
  uint8_t  WrProtectGrEnable;    /*!< Write protect group enable */
  uint8_t  ManDeflECC;           /*!< Manufacturer default ECC */
  uint8_t  WrSpeedFact;          /*!< Write speed factor */
  uint8_t  MaxWrBlockLen;        /*!< Max. write data block length */
  uint8_t  WriteBlockPaPartial;  /*!< Partial blocks for write allowed */
  uint8_t  Reserved3;            /*!< Reserded */
  uint8_t  ContentProtectAppli;  /*!< Content protection application */
  uint8_t  FileFormatGrouop;     /*!< File format group */
  uint8_t  CopyFlag;             /*!< Copy flag (OTP) */
  uint8_t  PermWrProtect;        /*!< Permanent write protection */
  uint8_t  TempWrProtect;        /*!< Temporary write protection */
  uint8_t  FileFormat;           /*!< File Format */
  uint8_t  ECC;                  /*!< ECC code */
  uint8_t  CSD_CRC;              /*!< CSD CRC */
  uint8_t  Reserved4;            /*!< always 1*/
} SD_CSD;   

/**
  * @brief  SD card CID register type
  */
typedef struct
{
  uint8_t  ManufacturerID;       /*!< ManufacturerID */
  uint16_t OEM_AppliID;          /*!< OEM/Application ID */
  uint32_t ProdName1;            /*!< Product Name part1 */
  uint8_t  ProdName2;            /*!< Product Name part2*/
  uint8_t  ProdRev;              /*!< Product Revision */
  uint32_t ProdSN;               /*!< Product Serial Number */
  uint8_t  Reserved1;            /*!< Reserved1 */
  uint16_t ManufactDate;         /*!< Manufacturing Date */
  uint8_t  CID_CRC;              /*!< CID CRC */
  uint8_t  Reserved2;            /*!< always 1 */
} SD_CID; 
  
/**
  * @brief  SD card Info type
  */
typedef struct
{
  SD_CSD SD_csd;            /*!< CSD register           */
  SD_CID SD_cid;            /*!< CID register           */
  long long CardCapacity;   /*!< SD card capacity       */
  uint32_t CardBlockSize;   /*!< Block size             */
  uint16_t RCA;             /*!< Relative Card Address  */ 
  uint8_t CardType;         /*!< Card Type              */
} SD_CardInfo;
  
  
/**
  * @brief SD Error State 
  * @{
  */ 
typedef enum{
  HAL_SD_ERROR_NONE               = 0x00000000U,  /* NO error                          */
  HAL_SD_ERROR_RE                 = 0x00000001U,  /* Response error                    */
  HAL_SD_ERROR_RCRC               = 0x00000002U,  /* Response CRC check failed         */
  HAL_SD_ERROR_DCRC               = 0x00000003U,  /* Data CRC check failed             */
  HAL_SD_ERROR_RTO                = 0x00000004U,  /* Response timeout                  */
  HAL_SD_ERROR_DRTO               = 0x00000005U,  /* Read data timeout                 */
  HAL_SD_ERROR_HTO                = 0x00000006U,  /* Host timeout                      */
  HAL_SD_ERROR_FRUN               = 0x00000007U,  /* FIFO underrun                     */
  HAL_SD_ERROR_HLE                = 0x00000008U,  /* Hardware lock error               */
  HAL_SD_ERROR_SBE                = 0x00000009U,  /* Start bit error                   */
  HAL_SD_ERROR_EBE                = 0x0000000AU,  /* End bit error                     */
  HAL_SD_ERROR_DE                 = 0x0000000BU,  /* Card detec error                  */
  HAL_SD_ERROR_CIE                = 0x0000000CU,  /* Inconsistent response and command */
  
  HAL_SD_ERROR_INTERNAL           = 0x0000000DU,  /* Internal error                    */
  HAL_SD_ERROR_INVALID_PARAMETER  = 0x0000000EU,  /* Invalid parameter                 */
  HAL_SD_REQUEST_NOT_APPLICABLE   = 0x0000000FU,  /* Request not applicable            */
  HAL_SD_LOCK_UNLOCK_FAILED       = 0x00000010U   /* Sequence or password error has been detected in unlock
                                                      command or if there was an attempt to access a locked card */
}SD_Error;
/** 
  * @}
  */

/**
  * @brief  SD detection on its memory slot
  */
#define SD_PRESENT                                 ((uint8_t)0x01)
#define SD_NOT_PRESENT                             ((uint8_t)0x00)

/**
  * @brief  Supported card types
  */
#define SDIO_STD_CAPACITY_SD_CARD_V1_1             ((uint32_t)0x00000000)
#define SDIO_STD_CAPACITY_SD_CARD_V2_0             ((uint32_t)0x00000001)
#define SDIO_HIGH_CAPACITY_SD_CARD                 ((uint32_t)0x00000002)
#define SDIO_MULTIMEDIA_CARD                       ((uint32_t)0x00000003)
#define SDIO_SECURE_DIGITAL_IO_CARD                ((uint32_t)0x00000004)
#define SDIO_HIGH_SPEED_MULTIMEDIA_CARD            ((uint32_t)0x00000005)
#define SDIO_SECURE_DIGITAL_IO_COMBO_CARD          ((uint32_t)0x00000006)
#define SDIO_HIGH_CAPACITY_MMC_CARD                ((uint32_t)0x00000007)

/**
  * @brief  card Voltage and capacity
  */
#define SD_VOLTAGE_WINDOW_SD                       ((uint32_t)0x80100000)
#define SD_HIGH_CAPACITY                           ((uint32_t)0x40000000)
#define SD_STD_CAPACITY                            ((uint32_t)0x00000000)
#define SD_CHECK_PATTERN                           ((uint32_t)0x000001AA)
#define SD_MAX_VOLT_TRIAL                          ((uint32_t)0x0000FFFF)

/**
  * @brief  card error
  */
#define SDIO_STATIC_FLAGS                          ((uint32_t)0xFFFFFFFF)
#define SD_OCR_ERRORBITS                           ((uint32_t)0xFDFFE008)
#define SDIO_DATATIMEOUT                           ((uint32_t)0xFFFFFFFF)
#define SDIO_CMD0TIMEOUT                           ((uint32_t)0x00010000)

/**
  * @brief  card state
  */
#define SD_CARD_LOCKED                             ((uint32_t)0x02000000)
#define SD_CARD_PROGRAMMING                        ((uint32_t)0x00000007)
#define SD_CARD_RECEIVING                          ((uint32_t)0x00000006)


/**
  * @brief  Command Class Supported
  */
#define SD_CCCC_LOCK_UNLOCK                       ((uint32_t)0x00000080)
#define SD_CCCC_WRITE_PROT                        ((uint32_t)0x00000040)
#define SD_CCCC_ERASE                             ((uint32_t)0x00000020)

/** 
  * @brief  Command Class supported
  */
#define SDIO_CCCC_ERASE                       0x00000020U

#define SDIO_CMDTIMEOUT                       5000U         /* Command send and response timeout */
#define SDIO_MAXERASETIMEOUT                  63000U        /* Max erase Timeout 63 s            */
#define SDIO_STOPTRANSFERTIMEOUT              100000000U    /* Timeout for STOP TRANSMISSION command */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SD_Exported_Functions SD Exported Functions
  * @{
  */
  
SD_Error HAL_SD_Init(void);
void SD_DeInit(void);

SD_Error HAL_SD_PowerON(void);    
SD_Error HAL_SD_InitializeCards(void);

SD_Error HAL_SD_EnableWideBusOperation(uint32_t wmode);
SD_Error HAL_SD_SetDeviceMode(uint32_t mode);
SD_Error HAL_SD_SelectDeselect(uint32_t addr);

SD_Error HAL_SD_ReadBlock(uint32_t *pData,uint32_t BlockAdd,uint16_t Blocksize);
SD_Error HAL_SD_WriteBlock(uint32_t *pData,uint32_t BlockAdd,  uint16_t DataLength);

SD_Error HAL_SD_StopTransfer(void);
SD_Error HAL_SD_Erase(uint32_t startaddr, uint32_t endaddr);

void HAL_SD_DMA_ReadBlockInit(DMA_HandleTypeDef  *hdma_sd);
void HAL_SD_DMA_WriteBlockInit(DMA_HandleTypeDef  *hdma_sd);

SD_Error HAL_SD_ReadBlockDMA(DMA_HandleTypeDef  *hdma_sdio,uint8_t *pData,uint32_t BlockAdd,uint16_t BlockSize);
SD_Error HAL_SD_WriteBlockDMA(DMA_HandleTypeDef  *hdma_sdio,uint8_t *pData, uint32_t BlockAdd,  uint16_t BlockSize);

SD_Error HAL_SD_ReadBlockIT(uint8_t *pData,uint32_t BlockAdd,uint16_t BlockSize);
SD_Error HAL_SD_WriteBlockIT(uint8_t *pData,uint32_t BlockAdd,  uint16_t BlockSize);

SD_Error SD_ReadMultiBlocks(uint8_t *pData, uint32_t BlockAdd, uint16_t BlockSize, uint32_t NumberOfBlocks);  
SD_Error SD_WriteMultiBlocks(uint8_t *pData, uint32_t BlockAdd, uint16_t BlockSize, uint32_t NumberOfBlocks);

void HAL_SD_IRQHandler(void);
void HAL_SD_DMA_RxCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_DMA_RxErrorCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_DMA_TxErrorCallback(DMA_HandleTypeDef *hdma);
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

#endif /* __PY32F4XX_HAL_SD_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
