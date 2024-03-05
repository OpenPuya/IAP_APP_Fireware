/**
  ******************************************************************************
  * @file    bootloader.c
  * @author  Puya Application Team
  * @brief   Bootloader application entry point
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Puya Semiconductor.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bootloader.h"
#include "usart.h"
#include "flash.h"
#include "wdg.h"
#include "py32l020xx_ll_Start_Kit.h"

/* Private types ------------------------------------------------------------*/
typedef void (*Function_Pointer)(void);

/* Private define ------------------------------------------------------------*/
#define AREA_ERROR                        0x0U              /* Error Address Area */
#define FLASH_AREA                        0x1U              /* Flash Address Area */
#define RAM_AREA                          0x2U              /* RAM Address area */
#define OB_AREA                           0x3U              /* Option bytes Address area */
#define OTP_AREA                          0x4U              /* OTP Address area */
#define SYS_AREA                          0x5U              /* System memory area */
#define EB_AREA                           0x7U              /* Engi bytes Address area */

/* Private macro -------------------------------------------------------------*/
#if (defined (__CC_ARM)) || (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
const uint32_t BID_DATA __attribute__((at(BID_BASE))) = BID;
#elif defined(__ICCARM__)
const uint32_t BID_DATA @BID_BASE = BID;
#endif

const uint8_t BOOTLOADER_CMD[] = {
  CMD_GET_COMMAND, CMD_GET_ID,
  CMD_READ_MEMORY, CMD_GO, CMD_WRITE_MEMORY, CMD_EXT_ERASE_MEMORY
};

#ifdef CONST_OPTION
const uint32_t FLASH_OPTR_DATA __attribute__((at(OPTR_BASE))) = FLASH_OPTR;
const uint32_t FLASH_SDKR_DATA __attribute__((at(SDKR_BASE))) = FLASH_SDKR;
const uint32_t FLASH_BTCR_DATA __attribute__((at(BTCR_BASE))) = FLASH_BTCR;
const uint32_t FLASH_WRPR_DATA __attribute__((at(WRPR_BASE))) = FLASH_WRPR;
#endif

/* Private variables ---------------------------------------------------------*/
uint8_t guc_DataBuffer[0x180];

/* Private functions --------------------------------------------------------*/
void WriteFlashParameter(void);
ErrorStatus ReadMemory(void);
ErrorStatus Go(void);
ErrorStatus WriteMemory(void);
ErrorStatus ExtendedErase(void);
ErrorStatus WriteProtect(void);
uint8_t GetXOR(const uint8_t* pucDataBuf, uint16_t wDataLength, uint8_t ucBase);

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Initialize Bootloader.
  * @param  None.
  * @retval None.
  */
void Bootloader_Init(void)
{
#ifdef JUMP_TO_APP_BY_USER_BUTTON
  /* Configure user Button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  /* Check if the USER Button is pressed */
  if (BSP_PB_GetState(BUTTON_USER) == 0x00)
  {
    JumpToAddress(APP_ADDR);
  }
#endif
  
#ifdef JUMP_TO_APP_BY_TIME_OUT
  __IO uint32_t uwReadCount;
  
  uwReadCount = 0;
#endif
  
  __disable_irq();
  
  WDG_Init();
  
  USART_Init();

  while (0x7F != (uint8_t)(READ_BIT(USART1->DR, USART_DR_DR) & 0xFFU))
  {
#ifdef JUMP_TO_APP_BY_TIME_OUT
    if (uwReadCount++ > MAX_TIME_OUT)
    {
      JumpToAddress(APP_ADDR);
    }
#endif
    
    WDG_Refresh();
    
    SET_BIT(USART1->SR, USART_SR_ABRRQ);
  }
  
  CLEAR_BIT(USART1->CR3, USART_CR3_ABREN);
  
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD3, GPIO_PUPDR_PUPD3_0);//01: 上拉
  CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE3_0);  //10: 复用功能模式
  SET_BIT(GPIOA->AFR[0], GPIO_AFRL_AFSEL3_0);  //0001:AF1 USART1_TX(PA3) USART1_RX(PA4)  
  SET_BIT(USART1->CR1, USART_CR1_TE); //1： 传送使能

  USART_SendByte(ACK_BYTE);
}

/**
  * @brief  This function is used to select which protocol will be used when communicating with the host.
  * @param  None.
  * @retval None.
  */
void Bootloader_ProtocolDetection(void)
{
  uint8_t i;
  uint8_t ucCommand;
  ErrorStatus eStatus = SUCCESS;

#if 1//获取命令
  ucCommand = USART_ReadByte();
  /* Check the data integrity */
  if ((ucCommand ^ USART_ReadByte()) != 0xFF)
  {
    USART_SendByte(NACK_BYTE);
    return;
  }
#endif

  USART_SendByte(ACK_BYTE);

#if 1//FLASH_Unlock(统一入口，防止芯片跑飞，误操作FLASH)
  switch (ucCommand)
  {
  case CMD_WRITE_MEMORY:
  case CMD_EXT_ERASE_MEMORY:
    //检查 FLASH_SR 寄存器 BSY 位，确认没有正在进行的 flash 操作
    if (FLASH_SR_BSY == READ_BIT(FLASH->SR, FLASH_SR_BSY))
    {
      USART_SendByte(NACK_BYTE);
      return;
    }
    //向 FLASH_KEYR 寄存器依次写 KEY1 和 KEY2，解除 FLASH_CR 寄存器的保护
    if (FLASH_CR_LOCK == READ_BIT(FLASH->CR, FLASH_CR_LOCK))
    {
      WRITE_REG(FLASH->KEYR, FLASH_KEY1);
      WRITE_REG(FLASH->KEYR, FLASH_KEY2);
      
      WriteFlashParameter();
    }
    break;
  }
#endif

#if 1//执行命令  
  switch (ucCommand)
  {
  case CMD_GET_COMMAND:
    USART_SendByte(COUNTOF(BOOTLOADER_CMD));
    USART_SendByte(VERSION);
    for(i=0; i<COUNTOF(BOOTLOADER_CMD); i++)
    {
      USART_SendByte(BOOTLOADER_CMD[i]);
    }
    break;

  case CMD_GET_ID:
    USART_SendByte(0x01);
    USART_SendByte(0x00);
    USART_SendByte(0x64);
    break;

  case CMD_READ_MEMORY:
    eStatus = ReadMemory();
    if (SUCCESS == eStatus) {
      return;
    }
    break;

  case CMD_GO:
    eStatus = Go();
    break;

  case CMD_WRITE_MEMORY:
    eStatus = WriteMemory();
    break;

  case CMD_EXT_ERASE_MEMORY:
    eStatus = ExtendedErase();
    break;

  default:
    eStatus = ERROR;
    break;
  }
#endif

#if 1//FLASH_Lock
  switch (ucCommand)
  {
  case CMD_WRITE_MEMORY:
  case CMD_EXT_ERASE_MEMORY:
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);//FLASH_Lock
    break;
  }
#endif

  USART_SendByte((SUCCESS==eStatus) ? ACK_BYTE : NACK_BYTE);
}

/* Private functions --------------------------------------------------------*/

/**
  * @brief  Get the address and Check it is valid or not and returns the area type.
  * @param  pdwAddr The address to be got and checked.
  * @retval The address area: FLASH_AREA, RAM_AREA... if the address is valid
  *         or AREA_ERROR if the address is not valid.
  */
uint8_t GetAddressArea(uint32_t* pdwAddr)
{
  guc_DataBuffer[0] = USART_ReadByte();
  guc_DataBuffer[1] = USART_ReadByte();
  guc_DataBuffer[2] = USART_ReadByte();
  guc_DataBuffer[3] = USART_ReadByte();
  if (USART_ReadByte() != GetXOR(guc_DataBuffer, 0x04, 0x00))
  {
    return ERROR;
  }
  USART_SendByte(ACK_BYTE);
  *pdwAddr = (guc_DataBuffer[0]<<24)+(guc_DataBuffer[1]<<16)+(guc_DataBuffer[2]<<8)+guc_DataBuffer[3];

  if ((*pdwAddr >= FLASH_BASE) && (*pdwAddr < FLASH_END))
  {
    return FLASH_AREA;
  }
  
  if ((*pdwAddr >= OTP_BASE) && (*pdwAddr < (OTP_BASE+FLASH_PAGE_SIZE)))
  {
    return FLASH_AREA;
  }

  if ((*pdwAddr >= SRAM_BASE) && (*pdwAddr < SRAM_END))
  {
    return RAM_AREA;
  }

  return OB_AREA;
}

/**
 * @brief  This function is used to read memory from the device.
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: ReadMemory operation done
 *          - ERROR:   ReadMemory operation failed or the value of address is not ok
 */
ErrorStatus ReadMemory(void)
{  
  uint16_t i;
  uint32_t dwAddr;
  uint8_t ucAddrArea;
  uint8_t sdk_start;
  uint8_t sdk_end;

  ucAddrArea = GetAddressArea(&dwAddr);

  guc_DataBuffer[0] = USART_ReadByte();
  if ((guc_DataBuffer[0] ^ USART_ReadByte()) != 0xFF)
  {
    return ERROR;
  }
  USART_SendByte(ACK_BYTE);

  sdk_start = (READ_BIT(FLASH->SDKR, FLASH_SDKR_SDK_STRT) >> FLASH_SDKR_SDK_STRT_Pos);
  sdk_end = (READ_BIT(FLASH->SDKR, FLASH_SDKR_SDK_END) >> FLASH_SDKR_SDK_END_Pos);

  if ( (FLASH_AREA == ucAddrArea) && (sdk_start <= sdk_end) )
  {
    for (i=0; i<guc_DataBuffer[0]+1; i++)
    {
      USART_SendByte(HW8_REG(0+i));
    }
    return SUCCESS;
  }

  for (i=0; i<guc_DataBuffer[0]+1; i++)
  {
    USART_SendByte(HW8_REG(dwAddr+i));
  }

  return SUCCESS;
}

void JumpToAddress(uint32_t dwAddr)
{
  Function_Pointer jump_to_address;
  
  if (SRAM_BASE == (HW32_REG(dwAddr) & 0x2FFE0000))
  {
    __enable_irq();
    
    /*名称  说明                      地址
    -      保留（实际存的是MSP地址）0x00000000
    Reset  复位                     0x00000004
    */
    __set_MSP(HW32_REG(dwAddr));//Set Main Stack Pointer

    jump_to_address = (Function_Pointer)(HW32_REG(dwAddr + 4U));

    jump_to_address();
  }
}

/**
 * @brief  This function is used to jump to the user application.
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: Go operation done
 *          - ERROR:   Go operation failed or the value of address is not ok
 */
ErrorStatus Go(void)
{
  uint8_t ucMemArea;
  uint32_t dwAddr;  

  ucMemArea = GetAddressArea(&dwAddr);
  if ((ucMemArea != FLASH_AREA) && (ucMemArea != RAM_AREA))
  {
    return ERROR;
  }
  
  JumpToAddress(dwAddr);

  return SUCCESS;
}

/**
 * @brief  This function is used to write in to device memory.
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: WriteMemory operation done
 *          - ERROR:   WriteMemory operation failed or the value of address is not ok
 */
ErrorStatus WriteMemory(void)
{
  uint16_t i;
  uint8_t ucMemArea;
  uint8_t ucDataLength;
  uint32_t dwAddr;
  ErrorStatus eResultFlag;

  ucMemArea = GetAddressArea(&dwAddr);
  if ((ucMemArea != FLASH_AREA) && (ucMemArea != RAM_AREA) && (ucMemArea != OB_AREA))
  {
    return ERROR;
  }
 
  ucDataLength = USART_ReadByte();
  for (i=0; i<ucDataLength+1; i++)
  {
    guc_DataBuffer[i] = USART_ReadByte();
  }

  if (USART_ReadByte() != GetXOR(guc_DataBuffer, ucDataLength+1, ucDataLength))
  {
    return ERROR;
  }

  switch (ucMemArea)
  {
  case FLASH_AREA:
    eResultFlag = WriteFlash(dwAddr, guc_DataBuffer, ucDataLength);
    break;
  case RAM_AREA:
    for (i=0; i<ucDataLength+1; i++)
    {
      HW8_REG(dwAddr+i) = guc_DataBuffer[i];
    }
    eResultFlag = SUCCESS;
    break;
  case OB_AREA:
    eResultFlag = WriteOption(dwAddr, guc_DataBuffer, ucDataLength);
    break;
  }

  return eResultFlag;
}

/**
 * @brief  This function is used to erase a memory.
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: ExtendedErase operation done
 *          - ERROR:   ExtendedErase operation failed or the value of address is not ok
 */
ErrorStatus ExtendedErase(void)
{
  uint8_t i;
  uint8_t ucXOR;
  uint8_t ucDataTemp;
  uint8_t ucFuncFlag;
  uint8_t ucDataLength;
  ErrorStatus eResultFlag = ERROR;

  /* Read number of pages to be erased */
  ucFuncFlag = USART_ReadByte();
  ucDataLength = USART_ReadByte();

  /* Checksum initialization */
  ucXOR  = ucFuncFlag;
  ucXOR ^= ucDataLength;

  if (0xFF == ucFuncFlag)//0xFFFY, Y=F,E,D
  {
    //接收双字节的校验和
    if (USART_ReadByte() != ucXOR)
    {
      return ERROR;
    }
    
    ucDataLength = FLASH_SECTOR_NB - 1;
    
    for (i=0; i<ucDataLength+1; i++)
    {
      HW16_REG(guc_DataBuffer + 2*i) = i;
    }
    
    eResultFlag = SectorErase((uint16_t *)guc_DataBuffer, ucDataLength);
  }
  else
  {
    for (i=0; i<2*(ucDataLength+1); i++)
    {
      guc_DataBuffer[i] = USART_ReadByte();
    }
    ucXOR = GetXOR(guc_DataBuffer, 2*(ucDataLength+1), ucXOR);

    if (USART_ReadByte() != ucXOR)
    {
      return ERROR;
    }

    for (i=0; i<ucDataLength+1; i++)
    {
      ucDataTemp = guc_DataBuffer[2*i+0];
      guc_DataBuffer[2*i+0] = guc_DataBuffer[2*i+1];
      guc_DataBuffer[2*i+1] = ucDataTemp;
    }
    
    eResultFlag = SectorErase((uint16_t *)guc_DataBuffer, ucDataLength);
  }

  return eResultFlag;
}


/**
 * @brief  This function is used to enable write protect.
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: WriteProtect operation done
 *          - ERROR:   WriteProtect operation failed or the value of address is not ok
 */
ErrorStatus WriteProtect(void)
{
  uint16_t i;
  uint8_t ucDataLength;
  uint16_t ProtectedPages = 0xFFFF;

  ucDataLength = USART_ReadByte();
  for (i=0; i<ucDataLength+1; i++)
  {
    guc_DataBuffer[i] = USART_ReadByte();
  }

  if (USART_ReadByte() != GetXOR(guc_DataBuffer, ucDataLength+1, ucDataLength))
  {
    return ERROR;
  }

  for (i=0; i<ucDataLength+1; i++)
  {
    CLEAR_BIT(ProtectedPages, (1U<<guc_DataBuffer[i]));
  }

  guc_DataBuffer[0] = ProtectedPages&0xFF;
  guc_DataBuffer[1] = (ProtectedPages>>8)&0xFF;

  return WriteOption(WRPR_BASE, guc_DataBuffer, 0x01);
}

/**
 * @brief  This function is used to get XOR of the DataBuf.
 * @param *pucDataBuf Pointer to the DataBuf
 * @param wDataLength The length of the DataBuf
 * @param ucBase The base value of the DataBuf
 * @retval The XOR of the DataBuf
 */
uint8_t GetXOR(const uint8_t* pucDataBuf, uint16_t wDataLength, uint8_t ucBase)
{
  while(wDataLength--)
  {
    ucBase = ucBase ^*pucDataBuf++;
  }
  return ucBase;
}

#define FP_OFFSET 0
void WriteFlashParameter(void)
{
  FLASH->TS0 = ((HW32_REG(0x1FFF011C + FP_OFFSET * 0x14) >> 0) & 0x000001FF);
  FLASH->TS3 = ((HW32_REG(0x1FFF011C + FP_OFFSET * 0x14) >> 9) & 0x000001FF);
  FLASH->TS1 = ((HW32_REG(0x1FFF011C + FP_OFFSET * 0x14) >> 18) & 0x000003FF);
  FLASH->TS2P = ((HW32_REG(0x1FFF0120 + FP_OFFSET * 0x14) >> 0) & 0x000001FF);
  FLASH->TPS3 = ((HW32_REG(0x1FFF0120 + FP_OFFSET * 0x14) >> 16) & 0x00000FFF);
  FLASH->PERTPE = ((HW32_REG(0x1FFF0124 + FP_OFFSET * 0x14) >> 0) & 0x0003FFFF);
  FLASH->SMERTPE = ((HW32_REG(0x1FFF0128 + FP_OFFSET * 0x14) >> 0) & 0x0003FFFF);
  FLASH->PRGTPE = ((HW32_REG(0x1FFF012C + FP_OFFSET * 0x14) >> 0) & 0x0000FFFF);
  FLASH->PRETPE = ((HW32_REG(0x1FFF012C + FP_OFFSET * 0x14) >> 16) & 0x00003FFF);
}
