/**
  ******************************************************************************
  * @file    app_flash.c
  * @author  MCU Application Team
  * @brief   Contains FLASH HW configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Puya Semiconductor.
  * All rights reserved.</center></h2>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_flash.h"
#include "app_usart.h"
#include "app_bootloader.h"
#include "app_wdg.h"

#define ADJUST_ADDR_AND_SIZE

/**
  * @brief  This function is used to write data in FLASH memory.
  * @param  dwAddr The address where that data will be written.
  * @param  pucDataBuf The data to be written.
  * @param  ucDataLength The length of the data to be written.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Write FLASH operation done
  *          - ERROR:   Write FLASH operation failed or the value of one parameter is not ok
  */
ErrorStatus APP_WriteFlash(uint32_t dwAddr, uint8_t *pucDataBuf, uint8_t ucDataLength)
{
#ifdef CHECK_WRP
  uint8_t ucSector;
#endif
  uint16_t i, j;
  uint16_t wDataLength;
  uint32_t dwOffsetAddr;
  ErrorStatus eResultFlag;

#ifdef ADJUST_ADDR_AND_SIZE
  uint32_t dwOrgAddr;//备份调整前的地址
  uint32_t dwEndAddr;//调整后的结束地址

  wDataLength = ucDataLength + 1;
  /*
    参考uISP工具:
    当地为0x08000000，大小为0xC8时，调整后的起始地址为0x08000000，结束地址为0x080000FF，所以大小为0x100
    当地为0x080000C8，大小为0xC8时，调整后的起始地址为0x08000000，结束地址为0x080001FF，所以大小为0x200
    当地为0x08000190，大小为0xC8时，调整后的起始地址为0x08000100，结束地址为0x080002FF，所以大小为0x100
  */
  dwOrgAddr = dwAddr;//备份调整前的地址

  if (dwAddr & 0xFF) //起始地址不是按照256字节对齐，需要调整
  {
    CLEAR_BIT(dwAddr, 0xFF);//按256字节对方方式调整得到起始地址

    //数据向后偏移(dwOrgAddr-dwAddr)字节, (dwOrgAddr-dwAddr)==(dwOrgAddr&0xFF)
    for (i = 0; i < wDataLength; i++)
    {
      pucDataBuf[(dwOrgAddr & 0xFF) + wDataLength - 1 - i] = pucDataBuf[wDataLength - 1 - i];
    }

    //因为调整起始地址而补齐的数据从FLASH中读取，这里必须要先数据偏移然后才能读取
    for (i = 0; i < (dwOrgAddr & 0xFF); i++)
    {
      pucDataBuf[i] = HW8_REG(dwAddr + i);
    }
  }

  if ((dwOrgAddr & 0xFF) || (wDataLength & 0xFF)) //起始地址/大小不是按照256字节对齐，需要调整
  {
    dwEndAddr = ((dwOrgAddr + wDataLength - 1 + 0xFF) & ~0xFF) - 1; //按256字节对方方式调整得到结束地址

    //因为调整结束地址而补齐的数据从FLASH中读取
    for (i = ((dwOrgAddr & 0xFF) + wDataLength); i < (dwEndAddr - dwAddr + 1); i++)
    {
      pucDataBuf[i] = HW8_REG(dwAddr + i);
    }

    wDataLength = dwEndAddr - dwAddr + 1;//大小 = 结束地址-起始地址+1
  }
#else
  wDataLength = ucDataLength + 1;

  if ((wDataLength & 0xFF) || (dwAddr & 0xFF))
  {
    return ERROR;
  }
#endif

  //1) 检查 FLASH_SR 寄存器 BSY 位，确认没有正在进行的 flash 操作

  //2) 如果没有正在进行的 flash erase 或者 program 操作，则软件读出该 Page 的 32 个
  //word（如果该 page 已有数据存放，则进行该步骤，否则跳过该步骤）

  //3) 向 FLASH_KEYR 寄存器依次写 KEY1 和 KEY2，解除 FLASH_CR 寄存器的保护
  //统一FLASH_Unlock入口，防止芯片跑飞，误操作FLASH

  dwOffsetAddr = 0;
  //5) 向目标地址进行 program 数据的操作，只接受 32bit 的 program
  for (i = 0; i < wDataLength / FLASH_PAGE_SIZE; i++)
  {
    APP_WDG_Refresh();
    
    //4) 置位 FLASH_CR 寄存器的 PG 位
    SET_BIT(FLASH->CR, (FLASH_CR_EOPIE | FLASH_CR_PG));

#ifdef CHECK_WRP
    ucSector = (dwAddr + dwOffsetAddr - FLASH_BASE) / 0x1000;
    if (0 == READ_BIT(FLASH->WRPR, (1U << ucSector)))
    {
      eResultFlag = ERROR;
      break;
    }
#endif
    for (j = 0; j < FLASH_PAGE_SIZE/4; j++) //0x40 = 64
    {
      HW32_REG(dwAddr + dwOffsetAddr) = HW32_REG(pucDataBuf + dwOffsetAddr);

      dwOffsetAddr += 4;

      if (((FLASH_PAGE_SIZE/4) - 2) == j) //0x40 - 2 = 62
      {
        //6) 在软件写第 63 个 word 后， 置位 FLASH_CR 寄存器的 PGSTRT
        SET_BIT(FLASH->CR, FLASH_CR_PGSTRT);
      }
    }

    //7) 写第 64 个 word 后
    //8) 等待 FLASH_SR 寄存器的 BSY 位被清零
    while (FLASH_SR_BSY == READ_BIT(FLASH->SR, FLASH_SR_BSY));
    
    //10) 如果不再有 program 操作，则软件清除 PG 位
    CLEAR_BIT(FLASH->CR, (FLASH_CR_EOPIE | FLASH_CR_PG));

    //9) 检查 FLASH_SR 寄存器的 EOP 标志位（当 program 操作已经成功，该位被置位），然后软件清零该位
    if (FLASH_SR_EOP == READ_BIT(FLASH->SR, FLASH_SR_EOP))
    {
      eResultFlag = SUCCESS;
      //清零 EOP 标志, 写 1，清零该位。
      SET_BIT(FLASH->SR, FLASH_SR_EOP);
    }
    else
    {
      eResultFlag = ERROR;
      break;
    }    
  }/* for (i = 0; i < wDataLength / 0x100; i++) */
  
  /* Verify Flash */
  dwOffsetAddr = 0;
  while (wDataLength)
  {
    if (HW32_REG(dwAddr + dwOffsetAddr) != HW32_REG(pucDataBuf + dwOffsetAddr))
    {
      eResultFlag = ERROR;
      break;
    }
    dwOffsetAddr += 4;
    wDataLength -= 4;
  }

  return eResultFlag;
}

/**
  * @brief  This function is used to write data in Option bytes.
  * @param  dwAddr The address where that data will be written.
  * @param  pucDataBuf The data to be written.
  * @param  ucDataLength The length of the data to be written.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Write Option bytes operation done
  *          - ERROR:   Write Option bytes operation failed or the value of one parameter is not ok
  */
ErrorStatus APP_WriteOption(uint32_t dwAddr, uint8_t *pucDataBuf, uint8_t ucDataLength)
{
  uint16_t i;
  uint32_t dwOPTR;
  uint32_t dwSDKR;
  uint32_t dwWRPR;
  ErrorStatus eResultFlag;

  dwOPTR = FLASH->OPTR;
  dwSDKR = FLASH->SDKR;
  dwWRPR = FLASH->WRPR;

  //2) 检查 BSY 位，确认没有正在进行的 Flash 操作

  //1) 用之前描述的步骤，清零 OPTLOCK 位
  //统一FLASH_Unlock入口，防止芯片跑飞，误操作FLASH
  if (READ_BIT(FLASH->CR, FLASH_CR_OPTLOCK) != 0x00U)
  {
    WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY1);
    WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY2);
  }

  for (i = 0; i < ucDataLength + 1; i++)
  {
    if ((OPTR_BASE + 0) == (dwAddr + i)) //RDP
    {
      dwOPTR &= ~(0x000000FF << 0);
      dwOPTR |= (pucDataBuf[i] << 0);
    }
    if ((OPTR_BASE + 1) == (dwAddr + i)) //BOR_EN,BOR_LEV[2:0],IWDG_SW,WWDG_SW,NRST_MODE,nBOOT1
    {
      dwOPTR &= ~(0x000000FF << 8);
      dwOPTR |= (pucDataBuf[i] << 8);
    }

    if ((SDKR_BASE+0) == (dwAddr+i))//SDK_STRT[4:0]
    {
      dwSDKR &= ~(0x000000FF<<0);
      dwSDKR |= (pucDataBuf[i]<<0);
    }
    if ((SDKR_BASE+1) == (dwAddr+i))//SDK_END[4:0]
    {
      dwSDKR &= ~(0x000000FF<<8);
      dwSDKR |= (pucDataBuf[i]<<8);
    }

    if ((WRPR_BASE + 0) == (dwAddr + i)) //WRP[7:0]
    {
      dwWRPR &= ~(0x000000FF << 0);
      dwWRPR |= (pucDataBuf[i] << 0);
    }
    if ((WRPR_BASE + 1) == (dwAddr + i)) //WRP[15:8]
    {
      dwWRPR &= ~(0x000000FF << 8);
      dwWRPR |= (pucDataBuf[i] << 8);
    }
  }

  //3) 向 option bytes 寄存器 FLASH_OPTR/FLASH_SDKR/FLASH_WRPR 写期望的值
  FLASH->OPTR = dwOPTR;
  FLASH->SDKR = dwSDKR;
  FLASH->WRPR = dwWRPR;

  SET_BIT(FLASH->CR, (FLASH_CR_EOPIE | FLASH_CR_OPTSTRT));
  __NOP();
  HW32_REG(0x40022080) = 0xFFFFFFFF;

  //4) 等待 BSY 位被清零
  while (FLASH_SR_BSY == READ_BIT(FLASH->SR, FLASH_SR_BSY));
  __NOP();

  //5) 等待 EOP 拉高，软件清零
  eResultFlag = (FLASH_SR_EOP == READ_BIT(FLASH->SR, FLASH_SR_EOP)) ? SUCCESS : ERROR;

  //清零 EOP 标志, 写 1，清零该位。
  SET_BIT(FLASH->SR, FLASH_SR_EOP);

  CLEAR_BIT(FLASH->CR, (FLASH_CR_EOPIE | FLASH_CR_OPTSTRT));

  //SET_BIT(FLASH->CR, FLASH_CR_OPTLOCK);//FLASH_OB_Lock
  //如果软件置位 Lock 位，则 OPTLOCK 位也被自动置位

  if (SUCCESS == eResultFlag)
  {
    APP_Bootloader_SendByte(ACK_BYTE);

    //当 FLASH_CR 寄存器中的 OBL_LAUNCH 位被置位, Option byte loading
    SET_BIT(FLASH->CR, FLASH_CR_OBL_LAUNCH);
  }

  return eResultFlag;
}

/**
  * @brief  This function is used to start FLASH mass erase operation.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Mass erase operation done
  *          - ERROR:   Mass erase operation failed or the value of one parameter is not ok
  */
ErrorStatus APP_MassErase(void)
{
	uint32_t dwAddr;
	uint32_t dwEndAddr;
	uint16_t wSectorNum;
	ErrorStatus eResultFlag;
	
	dwAddr = APP_ADDR;
	dwEndAddr = FLASH_BASE + (((HW8_REG(FLASHSIZE_BASE)&0x03)>>0)+1)*32*0x400;
	while (dwAddr < dwEndAddr) {
		wSectorNum = (dwAddr - FLASH_BASE)/FLASH_SECTOR_SIZE;
		eResultFlag = APP_SectorErase(&wSectorNum, 0);
		if (SUCCESS != eResultFlag) {
			break;
		}
		dwAddr += FLASH_SECTOR_SIZE;
	}
	
	return eResultFlag;
}

/**
  * @brief  This function is used to erase the specified FLASH pages.
  * @param  *pwDataBuf Pointer to the buffer that contains erase operation options.
  * @param  ucDataLength Size of the Data buffer.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Erase operation done
  *          - ERROR:   Erase operation failed or the value of one parameter is not ok
  */
ErrorStatus APP_PageErase(uint16_t *pwDataBuf, uint8_t ucDataLength, uint8_t ucPageCount)
{
  uint8_t i, j;
#ifdef CHECK_WRP
  uint8_t ucSector;
#endif
  uint32_t dwAddr;
  ErrorStatus eResultFlag = SUCCESS;

  //1) 检查 BSY 位，确认是否没有正在进行的 Flash 操作

  //2) 向 FLASH_KEYR 寄存器依次写 KEY1,KEY2，解除 FLASH_CR 寄存器保护
  //统一FLASH_Unlock入口，防止芯片跑飞，误操作FLASH

  for (i=0; i<ucDataLength+1; i++)
  {
    for (j = 0; j < ucPageCount; j++)
    {
      //3) 置位 FLASH_CR 寄存器的 PER 位
      SET_BIT(FLASH->CR, (FLASH_CR_EOPIE | FLASH_CR_PER));
      
      APP_WDG_Refresh();

      dwAddr = FLASH_BASE + (pwDataBuf[i] * ucPageCount + j) * FLASH_PAGE_SIZE;

      if ((((HW8_REG(FLASHSIZE_BASE)&0x03)>>0)+1)*32*0x400 <= (dwAddr-FLASH_BASE))
      {
        eResultFlag = SUCCESS;
        continue;
      }

#ifdef CHECK_WRP
      ucSector = (dwAddr - FLASH_BASE) / 0x1000;
      if (0 == READ_BIT(FLASH->WRPR, (1U << ucSector)))
      {
        eResultFlag = ERROR;
        break;
      }
#endif

#ifdef CHECK_SDK
      if (((FLASH->SDKR) & 0x1F) * 0x800 <= dwAddr
          && dwAddr < (((FLASH->SDKR >> 8) & 0x1F) + 1) * 0x800)
      {
        eResultFlag = ERROR;
        break;
      }
#endif

      //向该 page 写任意数据（必须 32bit 数据）
      HW32_REG(dwAddr) = 0xFFFFFFFF;

      //5) 等待 BSY 位被清零
      while (FLASH_SR_BSY == READ_BIT(FLASH->SR, FLASH_SR_BSY));
      
      CLEAR_BIT(FLASH->CR, (FLASH_CR_EOPIE | FLASH_CR_PER));

      //6) 检查 EOP 标志位被置位
      if (FLASH_SR_EOP == READ_BIT(FLASH->SR, FLASH_SR_EOP))
      {
        eResultFlag = SUCCESS;
        //7) 清零 EOP 标志, 写 1，清零该位。
        SET_BIT(FLASH->SR, FLASH_SR_EOP);
      }
      else
      {
        eResultFlag = ERROR;
        break;
      }      
      
    }/* for (j = 0; j < ucPageCount; j++) */
  }/* for (i = 0; i < ucDataLength + 1; i++) */

  return eResultFlag;
}

/**
  * @brief  This function is used to erase the specified FLASH sectors.
  * @param  *pwDataBuf Pointer to the buffer that contains erase operation options.
  * @param  ucDataLength Size of the Data buffer.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Erase operation done
  *          - ERROR:   Erase operation failed or the value of one parameter is not ok
  */
ErrorStatus APP_SectorErase(uint16_t *pwDataBuf, uint8_t ucDataLength)
{
  uint8_t i;
  ErrorStatus eResultFlag;

  //1) 检查 BSY 位，确认是否没有正在进行的 Flash 操作

  //2) 向 FLASH_KEYR 寄存器依次写 KEY1,KEY2，解除 FLASH_CR 寄存器保护
  //统一FLASH_Unlock入口，防止芯片跑飞，误操作FLASH

  for (i = 0; i < ucDataLength + 1; i++)
  {
    //3) 置位 FLASH_CR 寄存器的 SER 位
    SET_BIT(FLASH->CR, (FLASH_CR_EOPIE | FLASH_CR_SER));

    APP_WDG_Refresh();

#ifdef CHECK_WRP
    if (0 == READ_BIT(FLASH->WRPR, (1U << pwDataBuf[i])))
    {
      eResultFlag = ERROR;
      break;
    }
#endif

#ifdef CHECK_SDK
    if (((FLASH->SDKR) & 0x1F) * 0x800 <= (FLASH_BASE + pwDataBuf[i] * 0x1000)
        && (FLASH_BASE + pwDataBuf[i] * 0x1000) < (((FLASH->SDKR >> 8) & 0x1F) + 1) * 0x800)
    {
      eResultFlag = ERROR;
      break;
    }
#endif

    //4) 向该 sector 写任意数据
    HW32_REG(FLASH_BASE + pwDataBuf[i] * FLASH_SECTOR_SIZE) = 0xFFFFFFFF;

    //5) 等待 BSY 位被清零
    while (FLASH_SR_BSY == READ_BIT(FLASH->SR, FLASH_SR_BSY));
    
    CLEAR_BIT(FLASH->CR, (FLASH_CR_EOPIE | FLASH_CR_SER));

    //6) 检查 EOP 标志位被置位
    if (FLASH_SR_EOP == READ_BIT(FLASH->SR, FLASH_SR_EOP))
    {
      eResultFlag = SUCCESS;
      //7) 清零 EOP 标志, 写 1，清零该位。
      SET_BIT(FLASH->SR, FLASH_SR_EOP);
    }
    else
    {
      eResultFlag = ERROR;
      break;
    }
  }

  return eResultFlag;
}

#if 0
/**
  * @brief  This function is used to erase the specified FLASH blocks.
  * @param  *pwDataBuf Pointer to the buffer that contains erase operation options.
  * @param  ucDataLength Size of the Data buffer.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Erase operation done
  *          - ERROR:   Erase operation failed or the value of one parameter is not ok
  */
ErrorStatus APP_BlockErase(uint16_t *pwDataBuf, uint8_t ucDataLength)
{
  uint8_t i;
  ErrorStatus eResultFlag;

  //1) 检查 BSY 位，确认是否没有正在进行的 Flash 操作

  //2) 向 FLASH_KEYR 寄存器依次写 KEY1,KEY2，解除 FLASH_CR 寄存器保护
  //统一FLASH_Unlock入口，防止芯片跑飞，误操作FLASH

  //3) 置位 FLASH_CR 寄存器的 BER 位
  SET_BIT(FLASH->CR, (FLASH_CR_EOPIE | FLASH_CR_BER));

  for (i = 0; i < ucDataLength + 1; i++)
  {
    APP_WDG_Refresh();

#ifdef CHECK_WRP
    if (0 == READ_BIT(FLASH->WRPR, (1U << pwDataBuf[i])))
    {
      eResultFlag = ERROR;
      break;
    }
#endif

#ifdef CHECK_SDK
    if (((FLASH->SDKR) & 0x1F) * 0x800 <= (FLASH_BASE + pwDataBuf[i] * 0x1000)
        && (FLASH_BASE + pwDataBuf[i] * 0x1000) < (((FLASH->SDKR >> 8) & 0x1F) + 1) * 0x800)
    {
      eResultFlag = ERROR;
      break;
    }
#endif

    //4) 向该 block 写任意数据
    HW32_REG(FLASH_BASE + pwDataBuf[i] * FLASH_BLOCK_SIZE) = 0xFFFFFFFF;

    //5) 等待 BSY 位被清零
    while (FLASH_SR_BSY == READ_BIT(FLASH->SR, FLASH_SR_BSY));

    //6) 检查 EOP 标志位被置位
    if (FLASH_SR_EOP == READ_BIT(FLASH->SR, FLASH_SR_EOP))
    {
      eResultFlag = SUCCESS;
      //7) 清零 EOP 标志, 写 1，清零该位。
      SET_BIT(FLASH->SR, FLASH_SR_EOP);
    }
    else
    {
      eResultFlag = ERROR;
      break;
    }
  }

  CLEAR_BIT(FLASH->CR, (FLASH_CR_EOPIE | FLASH_CR_BER));

  return eResultFlag;
}
#endif
