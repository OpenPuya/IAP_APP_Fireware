/**
  ******************************************************************************
  * @file    flash.c
  * @author  Puya Application Team
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
#include "flash.h"
#include "usart.h"
#include "bootloader.h"
#include "wdg.h"

/**
  * @brief  This function is used to write data in FLASH memory.
  * @param  dwAddr The address where that data will be written.
  * @param  pucDataBuf The data to be written.
  * @param  ucDataLength The length of the data to be written.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Write FLASH operation done
  *          - ERROR:   Write FLASH operation failed or the value of one parameter is not ok
  */
ErrorStatus WriteFlash(uint32_t dwAddr, uint8_t* pucDataBuf, uint8_t ucDataLength)
{
  uint16_t i, j;
  uint16_t wDataLength;
  uint32_t dwOffsetAddr;
  ErrorStatus eResultFlag;

  wDataLength = ucDataLength + 1;

  if ((wDataLength&(FLASH_PAGE_SIZE-1))||(dwAddr&(FLASH_PAGE_SIZE-1)))
  {
    return ERROR;
  }

  //1) 检查 FLASH_SR 寄存器 BSY 位，确认没有正在进行的 flash 操作

  //2) 如果没有正在进行的 flash erase 或者 program 操作，则软件读出该 Page 的 32 个
  //word（如果该 page 已有数据存放，则进行该步骤，否则跳过该步骤）

  //3) 向 FLASH_KEYR 寄存器依次写 KEY1 和 KEY2，解除 FLASH_CR 寄存器的保护
  //统一FLASH_Unlock入口，防止芯片跑飞，误操作FLASH  

  dwOffsetAddr = 0;
  
  //5) 向目标地址进行 program 数据的操作，只接受 32bit 的 program
  for (i=0; i<wDataLength/FLASH_PAGE_SIZE; i++)
  {
    //4) 置位 FLASH_CR 寄存器的 PG 位
    SET_BIT(FLASH->CR, (FLASH_CR_EOPIE|FLASH_CR_PG));
    
    for (j=0; j<FLASH_PAGE_SIZE/4; j++)//0x20 = 32
    {
      HW32_REG(dwAddr+dwOffsetAddr) = HW32_REG(pucDataBuf+dwOffsetAddr);

      dwOffsetAddr += 4;

      if ((FLASH_PAGE_SIZE/4-2) == j)//0x20 - 2 = 30
      {
        //6) 在软件写第 31 个 word 后， 置位 FLASH_CR 寄存器的 PGSTRT
        SET_BIT(FLASH->CR, FLASH_CR_PGSTRT);
      }
    }

    //7) 写第 32 个 word 后
    //8) 等待 FLASH_SR 寄存器的 BSY 位被清零
    while (FLASH_SR_BSY == READ_BIT(FLASH->SR, FLASH_SR_BSY));

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
  }

  //10) 如果不再有 program 操作，则软件清除 PG 位
  CLEAR_BIT(FLASH->CR, (FLASH_CR_EOPIE|FLASH_CR_PG));

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
ErrorStatus WriteOption(uint32_t dwAddr, uint8_t* pucDataBuf, uint8_t ucDataLength)
{
  uint16_t i;
  uint32_t dwOPTR;
  uint32_t dwSDKR;
  uint32_t dwBTCR;
  uint32_t dwWRPR;
  ErrorStatus eResultFlag;

  dwOPTR = FLASH->OPTR;
  dwSDKR = FLASH->SDKR;
  dwBTCR = FLASH->BTCR;
  dwWRPR = FLASH->WRPR;

  //2) 检查 BSY 位，确认没有正在进行的 Flash 操作

  //1) 用之前描述的步骤，清零 OPTLOCK 位
  //统一FLASH_Unlock入口，防止芯片跑飞，误操作FLASH
  if (READ_BIT(FLASH->CR, FLASH_CR_OPTLOCK) != 0x00U)
  {
    WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY1);
    WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY2);
  }

  for (i=0; i<ucDataLength+1; i++)
  {
//    if ((OPTR_BASE+0) == (dwAddr+i))
//    {
//      dwOPTR &= ~(0x000000FF<<0);
//      dwOPTR |= (pucDataBuf[i]<<0);
//    }
    if ((OPTR_BASE+1) == (dwAddr+i))//BOR_EN,BOR_LEV[2:0],IWDG_SW,WWDG_SW,NRST_MODE,nBOOT1
    {
      dwOPTR &= ~(0x000000FF<<8);
      dwOPTR |= (pucDataBuf[i]<<8);
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
    
    if ((BTCR_BASE+0) == (dwAddr+i))//BOOT_SIZE[2:0]
    {
      dwBTCR &= ~(0x000000FF<<0);
      dwBTCR |= (pucDataBuf[i]<<0);
    }
    if ((BTCR_BASE+1) == (dwAddr+i))//nBOOT1[15],BOOT0[14]
    {
      dwBTCR &= ~(0x000000FF<<8);
      dwBTCR |= (pucDataBuf[i]<<8);
    }

    if ((WRPR_BASE+0) == (dwAddr+i))//WRP[7:0]
    {
      dwWRPR &= ~(0x000000FF<<0);
      dwWRPR |= (pucDataBuf[i]<<0);
    }
    if ((WRPR_BASE+1) == (dwAddr+i))//WRP[15:8]
    {
      dwWRPR &= ~(0x000000FF<<8);
      dwWRPR |= (pucDataBuf[i]<<8);
    }
  }

  //3) 向 option bytes 寄存器 FLASH_OPTR/FLASH_SDKR/FLASH_WRPR 写期望的值
  FLASH->OPTR = dwOPTR;
  FLASH->SDKR = dwSDKR;
  FLASH->BTCR = dwBTCR;
  FLASH->WRPR = dwWRPR;

  SET_BIT(FLASH->CR, (FLASH_CR_EOPIE|FLASH_CR_OPTSTRT));
  HW32_REG(0x40022080) = 0xFFFFFFFF;

  //4) 等待 BSY 位被清零
  while (FLASH_SR_BSY == READ_BIT(FLASH->SR, FLASH_SR_BSY));

  //5) 等待 EOP 拉高，软件清零
  eResultFlag = (FLASH_SR_EOP == READ_BIT(FLASH->SR, FLASH_SR_EOP)) ? SUCCESS : ERROR;

  //清零 EOP 标志, 写 1，清零该位。
  SET_BIT(FLASH->SR, FLASH_SR_EOP);

  CLEAR_BIT(FLASH->CR, (FLASH_CR_EOPIE|FLASH_CR_OPTSTRT));

  //SET_BIT(FLASH->CR, FLASH_CR_OPTLOCK);//FLASH_OB_Lock
  //如果软件置位 Lock 位，则 OPTLOCK 位也被自动置位

  if (SUCCESS == eResultFlag)
  {
    USART_SendByte(ACK_BYTE);

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
//ErrorStatus MassErase(void)
//{
//  ErrorStatus eResultFlag;

//  //1) 检查 BSY 位，确认是否没有正在进行的 Flash 操作

//  //2) 向 FLASH_KEYR 寄存器依次写 KEY1,KEY2，解除 FLASH_CR 寄存器保护
//  //统一FLASH_Unlock入口，防止芯片跑飞，误操作FLASH

//  //3) 置位 FLASH_CR 寄存器的 MER 位
//  SET_BIT(FLASH->CR, (FLASH_CR_EOPIE|FLASH_CR_MER));

//  //4) 向 flash 的任意空间写任意数据（32bit 数据）
//  HW32_REG(FLASH_BASE) = 0xFFFFFFFF;

//  //5) 等待 BSY 位被清零
//  while (FLASH_SR_BSY == READ_BIT(FLASH->SR, FLASH_SR_BSY));

//  //6) 检查 EOP 标志位被置位
//  eResultFlag = (FLASH_SR_EOP == READ_BIT(FLASH->SR, FLASH_SR_EOP)) ? SUCCESS : ERROR;

//  //7) 清零 EOP 标志, 写 1，清零该位。
//  SET_BIT(FLASH->SR, FLASH_SR_EOP);

//  CLEAR_BIT(FLASH->CR, (FLASH_CR_EOPIE|FLASH_CR_MER));

//  return eResultFlag;
//}

#if 0
/**
  * @brief  This function is used to erase the specified FLASH pages.
  * @param  *pwDataBuf Pointer to the buffer that contains erase operation options.
  * @param  ucDataLength Size of the Data buffer.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Erase operation done
  *          - ERROR:   Erase operation failed or the value of one parameter is not ok
  */
ErrorStatus PageErase(uint16_t* pwDataBuf, uint8_t ucDataLength)
{
  uint8_t i;

  uint32_t dwAddr;
  ErrorStatus eResultFlag = SUCCESS;

  //1) 检查 BSY 位，确认是否没有正在进行的 Flash 操作

  //2) 向 FLASH_KEYR 寄存器依次写 KEY1,KEY2，解除 FLASH_CR 寄存器保护
  //统一FLASH_Unlock入口，防止芯片跑飞，误操作FLASH

  for (i=0; i<ucDataLength+1; i++)
  {
    //3) 置位 FLASH_CR 寄存器的 PER 位
    SET_BIT(FLASH->CR, (FLASH_CR_EOPIE|FLASH_CR_PER));
    
    WDG_Refresh();
    
    dwAddr = FLASH_BASE+pwDataBuf[i]*FLASH_PAGE_SIZE;
    if (0xFFFF == pwDataBuf[i]) {
      dwAddr = OTP_BASE;
    }

    //向该 page 写任意数据（必须 32bit 数据）
    HW32_REG(dwAddr) = 0xFFFFFFFF;

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

  CLEAR_BIT(FLASH->CR, (FLASH_CR_EOPIE|FLASH_CR_PER));

  return eResultFlag;
}
#endif

/**
  * @brief  This function is used to erase the specified FLASH sectors.
  * @param  *pwDataBuf Pointer to the buffer that contains erase operation options.
  * @param  ucDataLength Size of the Data buffer.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Erase operation done
  *          - ERROR:   Erase operation failed or the value of one parameter is not ok
  */
ErrorStatus SectorErase(uint16_t* pwDataBuf, uint8_t ucDataLength)
{
  uint8_t i;
  ErrorStatus eResultFlag;

  //1) 检查 BSY 位，确认是否没有正在进行的 Flash 操作

  //2) 向 FLASH_KEYR 寄存器依次写 KEY1,KEY2，解除 FLASH_CR 寄存器保护
  //统一FLASH_Unlock入口，防止芯片跑飞，误操作FLASH

  for (i=0; i<ucDataLength+1; i++)
  {
    if (0 == pwDataBuf[i])
    {
      continue;
    }
    //3) 置位 FLASH_CR 寄存器的 SER 位
    SET_BIT(FLASH->CR, (FLASH_CR_EOPIE|FLASH_CR_SER));
    
    WDG_Refresh();

    //4) 向该 sector 写任意数据
    HW32_REG(FLASH_BASE+pwDataBuf[i]*FLASH_SECTOR_SIZE) = 0xFFFFFFFF;

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

  CLEAR_BIT(FLASH->CR, (FLASH_CR_EOPIE|FLASH_CR_SER));

  return eResultFlag;
}
