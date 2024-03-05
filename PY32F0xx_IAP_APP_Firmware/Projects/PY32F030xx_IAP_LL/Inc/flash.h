/**
  ******************************************************************************
  * @file    flash.h
  * @author  Puya Application Team
  * @brief   Header for flash.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Puya Semiconductor.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_H
#define __FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define SYSTEM_BASE             (0x1FFF0000UL)
#define OPTION_BASE             (0x1FFF0E80UL)
#define OPTR_BASE               (OPTION_BASE + 0x00000000UL)
#define SDKR_BASE               (OPTION_BASE + 0x00000004UL)
#define WRPR_BASE               (OPTION_BASE + 0x0000000CUL)
#define DBGMCU_IDCODE           ((DBGMCU->IDCODE) & IDCODE_DEVID_MASK)//(0x1FFF0FF8UL)
#define IDCODE_DEVID_MASK       (0x00000FFFUL)

#define FLASH_OPTR               0x4F55B0AA
#define FLASH_SDKR               0xFFE0001F
#define FLASH_WRPR               0x0000FFFF

#define STM32F0_FLASHSIZE_BASE  (0x1FFFF7CCUL)
#define STM32F1_FLASHSIZE_BASE  (0x1FFFF7E0UL)
#define ST_UID_BASE             (0x1FFFF7E8UL)
#define ST_OPTION_BASE          (0x1FFFF800UL)
#define ST_BID_BASE             (0x1FFFF7A6UL)


/** @defgroup FLASH_PROGRAM_ERASE_CLOCK FLASH Program and Erase Clock
  * @{
  */
#define FLASH_PROGRAM_ERASE_CLOCK_4MHZ        0x00000000U           /*!< 4MHz */
#define FLASH_PROGRAM_ERASE_CLOCK_8MHZ        0x00000001U           /*!< 8MHz */
#define FLASH_PROGRAM_ERASE_CLOCK_16MHZ       0x00000002U           /*!< 16MHz */
#define FLASH_PROGRAM_ERASE_CLOCK_22P12MHZ    0x00000003U           /*!< 22.12MHz */
#define FLASH_PROGRAM_ERASE_CLOCK_24MHZ       0x00000004U           /*!< 24MHz */
/**
  * @}
  */

#define __HAL_FLASH_TIME_REG_SET(__EPPARA0__,__EPPARA1__,__EPPARA2__,__EPPARA3__,__EPPARA4__)           \
                                                        do {                                            \
                                                            FLASH->TS0  = (__EPPARA0__)&0xFF;           \
                                                            FLASH->TS1  = ((__EPPARA0__)>>16)&0x1FF;    \
                                                            FLASH->TS3  = ((__EPPARA0__)>>8)&0xFF;      \
                                                            FLASH->TS2P = (__EPPARA1__)&0xFF;           \
                                                            FLASH->TPS3 = ((__EPPARA1__)>>16)&0x7FF;    \
                                                            FLASH->PERTPE = (__EPPARA2__)&0x1FFFF;      \
                                                            FLASH->SMERTPE = (__EPPARA3__)&0x1FFFF;     \
                                                            FLASH->PRGTPE = (__EPPARA4__)&0xFFFF;       \
                                                            FLASH->PRETPE = ((__EPPARA4__)>>16)&0x3FFF; \
                                                         } while(0U)

/* Exported functions ------------------------------------------------------- */
ErrorStatus WriteFlash(uint32_t dwAddr, uint8_t* pucDataBuf, uint8_t ucDataLength);
ErrorStatus WriteOption(uint32_t dwAddr, uint8_t* pucDataBuf, uint8_t ucDataLength);
ErrorStatus MassErase(void);
ErrorStatus PageErase(uint16_t* pwDataBuf, uint8_t ucDataLength, uint8_t ucPageCount);
ErrorStatus SectorErase(uint16_t* pwDataBuf, uint8_t ucDataLength);

#endif /* __FLASH_H */

/************************ (C) COPYRIGHT Puya Semiconductor *****END OF FILE****/
