/**
  ******************************************************************************
  * @file    app_flash.h
  * @author  MCU Application Team
  * @brief   Header for app_flash.c module
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
#ifndef __APP_FLASH_H
#define __APP_FLASH_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define FLASH_OPTR_RDP_LEVEL_0          (0xAA)
#define FLASH_OPTR_RDP_LEVEL_1          (0x55)

/* Exported macro ------------------------------------------------------------*/
#define SYSTEM_BASE             (0x1FFF0000UL)
#define OPTR_BASE               (OB_BASE + 0x00000000UL)
#define SDKR_BASE               (OB_BASE + 0x00000008UL)
#define WRPR_BASE               (OB_BASE + 0x00000018UL)
#define DBGMCU_IDCODE           ((DBGMCU->IDCODE) & IDCODE_DEVID_MASK)
#define IDCODE_DEVID_MASK       (0x00000FFFUL)

#define FLASH_OPTR               0x2755D8AA
#define FLASH_SDKR               0xFFE0001F
#define FLASH_WRPR               0x0000FFFF

#define STM32F0_FLASHSIZE_BASE  (0x1FFFF7CCUL)
#define STM32F1_FLASHSIZE_BASE  (0x1FFFF7E0UL)
#define STM32F3_FLASHSIZE_BASE  (0x1FFFF7CCUL)
#define STM32F4_FLASHSIZE_BASE  (0x1FFF7A22UL)
#define STM32F0_UID_BASE        (0x1FFFF7ACUL)
#define STM32F1_UID_BASE        (0x1FFFF7E8UL)
#define STM32F3_UID_BASE        (0x1FFFF7ACUL)
#define STM32F4_UID_BASE        (0x1FFF7A10UL)
#define STM32F0_OB_BASE         (0x1FFFF800UL)
#define STM32F4_OB_BASE         (0x1FFFC000UL)
#define STM32F0_BID_BASE        (0x1FFFF6A6UL)
#define STM32F4_BID_BASE        (0x1FFF77DEUL)

/* Exported functions ------------------------------------------------------- */
ErrorStatus APP_WriteFlash(uint32_t dwAddr, uint8_t* pucDataBuf, uint8_t ucDataLength);
ErrorStatus APP_WriteOption(uint32_t dwAddr, uint8_t* pucDataBuf, uint8_t ucDataLength);
ErrorStatus APP_MassErase(void);
ErrorStatus APP_PageErase(uint16_t* pwDataBuf, uint8_t ucDataLength, uint8_t ucPageCount);
ErrorStatus APP_SectorErase(uint16_t* pwDataBuf, uint8_t ucDataLength);
//ErrorStatus APP_BlockErase(uint16_t* pwDataBuf, uint8_t ucDataLength);

#endif /* __APP_FLASH_H */

/************************ (C) COPYRIGHT Puya Semiconductor *****END OF FILE****/
