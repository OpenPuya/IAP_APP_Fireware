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
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define OPTR_BASE               (OB_BASE + 0x00000000UL)
#define SDKR_BASE               (OB_BASE + 0x00000004UL)
#define BTCR_BASE               (OB_BASE + 0x00000008UL)
#define WRPR_BASE               (OB_BASE + 0x0000000CUL)

#define FLASH_OPTR               0x4F55B0AA
#define FLASH_SDKR               0xFFF4000B
#define FLASH_BTCR               0xFFFF0000
#define FLASH_WRPR               0xFFC0003F

/* Exported functions ------------------------------------------------------- */
ErrorStatus WriteFlash(uint32_t dwAddr, uint8_t* pucDataBuf, uint8_t ucDataLength);
ErrorStatus WriteOption(uint32_t dwAddr, uint8_t* pucDataBuf, uint8_t ucDataLength);
//ErrorStatus MassErase(void);
ErrorStatus PageErase(uint16_t* pwDataBuf, uint8_t ucDataLength);
ErrorStatus SectorErase(uint16_t* pwDataBuf, uint8_t ucDataLength);

#endif /* __FLASH_H */

/************************ (C) COPYRIGHT Puya Semiconductor *****END OF FILE****/
