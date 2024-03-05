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

/* Private types ------------------------------------------------------------*/
typedef void (*Function_Pointer)(void);

/* Private define ------------------------------------------------------------*/
#define SYS_CLK                           24000000U
#define MAX_BAUDRATE                      (1000000+100)
#define MIN_BAUDRATE                      (1200-100)
#define DEFAULT_BAUDRATE                  115200            /* Default Baudrate */
#define MAX_BRR                           ((SYS_CLK+MAX_BAUDRATE/2)/MAX_BAUDRATE)
#define MIN_BRR                           ((SYS_CLK+MIN_BAUDRATE/2)/MIN_BAUDRATE)
#define DEFAULT_BRR                       ((SYS_CLK+DEFAULT_BAUDRATE/2)/DEFAULT_BAUDRATE)

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
  CMD_GET_COMMAND, CMD_GET_VERSION, CMD_GET_ID,
  CMD_READ_MEMORY, CMD_GO, CMD_WRITE_MEMORY, CMD_EXT_ERASE_MEMORY,
  CMD_WRITE_PROTECT, CMD_WRITE_UNPROTECT, CMD_READ_PROTECT, CMD_READ_UNPROTECT,
  CMD_GET_DEVICE_IDCODE
};

/* Private variables ---------------------------------------------------------*/
uint8_t guc_DataBuffer[0x180];

/* Private functions --------------------------------------------------------*/
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
  uint32_t dwRstPALevel[3][2];
  uint32_t wPAConfig;

#ifdef JUMP_TO_APP_BY_TIME_OUT
  __IO uint32_t uwReadCount;
  
  uwReadCount = 0;
#endif
  
  WDG_Init();

#if 1//轮询检查RX(PA3,PA10,PA15)是否接收到START信号(低电平)
  SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);

  CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODE3|GPIO_MODER_MODE10|GPIO_MODER_MODE15));

  wPAConfig = *((__IO uint32_t *)(0x1FFF0D90));
  if ((wPAConfig&(1<<3)) == 0) //当前芯片PA03未封出
  {
    SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD3_0); //增加上拉配置
  }
  if ((wPAConfig&(1<<10)) == 0) //当前芯片PA10未封出
  {
    SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD10_0); //增加上拉配置
  }
  if ((wPAConfig&(1<<15)) == 0) //当前芯片PA15未封出
  {
    SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD15_0); //增加上拉配置
  }

  dwRstPALevel[0][0] = READ_BIT(GPIOA->IDR, GPIO_IDR_ID3);
  dwRstPALevel[1][0] = READ_BIT(GPIOA->IDR, GPIO_IDR_ID10);
  dwRstPALevel[2][0] = READ_BIT(GPIOA->IDR, GPIO_IDR_ID15);

  while (1)
  {
#ifdef JUMP_TO_APP_BY_TIME_OUT
    if (uwReadCount++ > MAX_TIME_OUT)
    {
      JumpToAddress(APP_ADDR);
    }
#endif
    
    dwRstPALevel[0][1] = dwRstPALevel[0][0];
    dwRstPALevel[1][1] = dwRstPALevel[1][0];
    dwRstPALevel[2][1] = dwRstPALevel[2][0];
    dwRstPALevel[0][0] = READ_BIT(GPIOA->IDR, GPIO_IDR_ID3);
    dwRstPALevel[1][0] = READ_BIT(GPIOA->IDR, GPIO_IDR_ID10);
    dwRstPALevel[2][0] = READ_BIT(GPIOA->IDR, GPIO_IDR_ID15);
    WDG_Refresh();
    if (dwRstPALevel[0][1] && (!dwRstPALevel[0][0]))
    {
      WRITE_REG(GPIOA->MODER, 0xEBFFFFFF);
      CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODE2_0|GPIO_MODER_MODE3_0));//10: 复用功能模式
      SET_BIT(GPIOA->AFR[0], (GPIO_AFRL_AFSEL2_0|GPIO_AFRL_AFSEL3_0));//0001:AF1 USART1_TX(PA2) USART1_RX(PA3)
      break;
    }
    if (dwRstPALevel[1][1] && (!dwRstPALevel[1][0]))
    {
      WRITE_REG(GPIOA->MODER, 0xEBFFFFFF);
      CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODE9_0|GPIO_MODER_MODE10_0));//10: 复用功能模式
      SET_BIT(GPIOA->AFR[1], (GPIO_AFRH_AFSEL9_0|GPIO_AFRH_AFSEL10_0));//0001:AF1 USART1_TX(PA9) USART1_RX(PA10)
      break;
    }
    if (dwRstPALevel[2][1] && (!dwRstPALevel[2][0]))
    {
      WRITE_REG(GPIOA->MODER, 0xEBFFFFFF);
      CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODE14_0|GPIO_MODER_MODE15_0));//10: 复用功能模式
      SET_BIT(GPIOA->AFR[1], (GPIO_AFRH_AFSEL14_0|GPIO_AFRH_AFSEL15_0));//0001:AF1 USART1_TX(PA14) USART1_RX(PA15)
      break;
    }
  }
#endif

#if 1//USART初始化  
  SET_BIT(RCC->APBENR2, RCC_APBENR2_USART1EN);

  //USART_CR1_M  1： 1 start bit， 9 data bit， n stop bit
  //USART_CR1_PCE  1：奇偶校验使能
  //USART_CR1_PS  0：偶校验(EVEN)
  //USART_CR2_STOP 00： 1 stop bit
  SET_BIT(USART1->CR1, (USART_CR1_M|USART_CR1_PCE));

  WRITE_REG(USART1->BRR, DEFAULT_BRR);

  //USART_CR3_ABRMODE  00：从 start 位开始测量波特率
  //USART_CR3_ABREN  1：自动波特率使能
  MODIFY_REG(USART1->CR3, USART_CR3_ABRMODE, USART_CR3_ABREN);

  SET_BIT(USART1->CR1, USART_CR1_UE);//1： USART 使能
#endif

#if 1//USART自动波特率识别，若成功则发送ACK，失败则Reset
  while (USART_SR_ABRF != READ_BIT(USART1->SR, USART_SR_ABRF));

  if ( (USART_SR_ABRE == READ_BIT(USART1->SR, USART_SR_ABRE)) ||
       (MIN_BRR < USART1->BRR) || (USART1->BRR < MAX_BRR) )
  {
    NVIC_SystemReset();
  }

  __disable_irq();

  SET_BIT(USART1->SR, USART_SR_ABRRQ);
  CLEAR_BIT(USART1->CR3, USART_CR3_ABREN);//0：自动波特率禁止

  CLEAR_BIT(USART1->SR, (USART_SR_RXNE|USART_SR_TC));
  SET_BIT(USART1->CR1, (USART_CR1_TE|USART_CR1_RE));//1： 传送使能；1： 接收使能，开始检测 start 位；

  USART_SendByte(ACK_BYTE);
#endif
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
  case CMD_WRITE_PROTECT:
  case CMD_WRITE_UNPROTECT:
  case CMD_READ_PROTECT:
  case CMD_READ_UNPROTECT:
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
      
      /* 配置flash擦写时间参数 */
      __HAL_FLASH_TIME_REG_SET((*(__IO uint32_t *)(0x1FFF0F6C)),     \
                               (*(__IO uint32_t *)(0x1FFF0F6C+4)),   \
                               (*(__IO uint32_t *)(0x1FFF0F6C+8)),   \
                               (*(__IO uint32_t *)(0x1FFF0F6C+12)),  \
                               (*(__IO uint32_t *)(0x1FFF0F6C+16)));
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

  case CMD_GET_VERSION:
    USART_SendByte(VERSION);
    USART_SendByte(0x00);
    USART_SendByte(0x00);
    break;

  case CMD_GET_ID://RETURN ST IDCODE
    USART_SendByte(0x01);
    USART_SendByte(0x04);
    USART_SendByte(0x40);
    break;

  case CMD_GET_DEVICE_IDCODE:
    USART_SendByte(0x01);
    USART_SendByte(DBGMCU_IDCODE>>8);
    USART_SendByte(DBGMCU_IDCODE&0xFF);
    break;

  case CMD_READ_MEMORY:
    eStatus = ReadMemory();
    return;

  case CMD_GO:
    eStatus = Go();
    break;

  case CMD_WRITE_MEMORY:
    eStatus = WriteMemory();
    break;

  case CMD_EXT_ERASE_MEMORY:
    eStatus = ExtendedErase();
    break;

  case CMD_WRITE_PROTECT:
    eStatus = WriteProtect();
    break;

  case CMD_WRITE_UNPROTECT:
    guc_DataBuffer[0] = (FLASH_SDKR>>0)&0xFF;
    guc_DataBuffer[1] = (FLASH_SDKR>>8)&0xFF;
    guc_DataBuffer[4] = (FLASH_WRPR>>0)&0xFF;
    guc_DataBuffer[5] = (FLASH_WRPR>>8)&0xFF;
    eStatus = WriteOption(SDKR_BASE, guc_DataBuffer, 0x07);
    break;

  case CMD_READ_PROTECT:
    guc_DataBuffer[0] = FLASH_OPTR_RDP_LEVEL_1;
    eStatus = WriteOption(OPTR_BASE, guc_DataBuffer, 0x00);
    break;

  case CMD_READ_UNPROTECT:
    guc_DataBuffer[0] = (FLASH_OPTR>>0)&0xFF;
    guc_DataBuffer[1] = (FLASH_OPTR>>8)&0xFF;
    guc_DataBuffer[4] = (FLASH_SDKR>>0)&0xFF;
    guc_DataBuffer[5] = (FLASH_SDKR>>8)&0xFF;
    eStatus = WriteOption(OPTR_BASE, guc_DataBuffer, 0x07);
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
  case CMD_WRITE_PROTECT:
  case CMD_WRITE_UNPROTECT:
  case CMD_READ_PROTECT:
  case CMD_READ_UNPROTECT:
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
  uint32_t dwStartAddr;
  uint32_t dwEndAddr;

  guc_DataBuffer[0] = USART_ReadByte();
  guc_DataBuffer[1] = USART_ReadByte();
  guc_DataBuffer[2] = USART_ReadByte();
  guc_DataBuffer[3] = USART_ReadByte();
  if (USART_ReadByte() != GetXOR(guc_DataBuffer, 0x04, 0x00))
  {
    return AREA_ERROR;
  }
  USART_SendByte(ACK_BYTE);
  *pdwAddr = (guc_DataBuffer[0]<<24)+(guc_DataBuffer[1]<<16)+(guc_DataBuffer[2]<<8)+guc_DataBuffer[3];

  switch (*pdwAddr)
  {
  case STM32F0_FLASHSIZE_BASE:
  case STM32F1_FLASHSIZE_BASE:
    return OB_AREA;
  case ST_UID_BASE:
    *pdwAddr = UID_BASE;
    break;
  case ST_OPTION_BASE:
    *pdwAddr = OPTION_BASE;
    break;
  case ST_BID_BASE:
    *pdwAddr = BID_BASE;
    break;
  }

  dwStartAddr = FLASH_BASE;
  dwEndAddr = FLASH_BASE + (((HW8_REG(FLASHSIZE_BASE)&0x07)>>0)+1)*8*0x400;
  if ((*pdwAddr >= dwStartAddr) && (*pdwAddr < dwEndAddr))
  {
    return FLASH_AREA;
  }

  dwStartAddr = SRAM_BASE + 0x0200;
  dwEndAddr = SRAM_BASE + (((HW8_REG(FLASHSIZE_BASE)&0x30)>>4)+1)*2*0x400;
  if ((*pdwAddr >= dwStartAddr) && (*pdwAddr < dwEndAddr))
  {
    return RAM_AREA;
  }

  dwStartAddr = SYSTEM_BASE;
  dwEndAddr = SYSTEM_BASE+0x0E00;
  if ((*pdwAddr >= dwStartAddr) && (*pdwAddr < dwEndAddr))
  {
    return SYS_AREA;
  }

  dwStartAddr = SYSTEM_BASE+0x0E00;
  dwEndAddr = 0x1FFF1000;
  if ((*pdwAddr >= dwStartAddr) && (*pdwAddr < dwEndAddr))
  {
    return OB_AREA;
  }

  return AREA_ERROR;
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
  
  ucAddrArea = GetAddressArea(&dwAddr);

  if (AREA_ERROR == ucAddrArea)
  {
    return ERROR;
  }

  guc_DataBuffer[0] = USART_ReadByte();
  if ((guc_DataBuffer[0] ^ USART_ReadByte()) != 0xFF)
  {
    return ERROR;
  }
  USART_SendByte(ACK_BYTE);

  switch (dwAddr)
  {
  case STM32F0_FLASHSIZE_BASE:
  case STM32F1_FLASHSIZE_BASE:
    guc_DataBuffer[1] = (((HW8_REG(FLASHSIZE_BASE)&0x07)>>0)+1)*8;
    guc_DataBuffer[2] = 0x00;
    guc_DataBuffer[3] = (((HW8_REG(FLASHSIZE_BASE)&0x30)>>4)+1)*2;
    guc_DataBuffer[4] = 0x00;
    for (i=0; i<guc_DataBuffer[0]+1; i++)
    {
      USART_SendByte(guc_DataBuffer[i+1]);
    }
    return SUCCESS;
  }
  
  if ( (FLASH_AREA == ucAddrArea) && (FLASH_OPTR_RDP_LEVEL_0 != READ_BIT(FLASH->OPTR, FLASH_OPTR_RDP)) )
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

  APP_SystemClockConfig(LL_RCC_HSICALIBRATION_8MHz, 8000000);

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

    return MassErase();//0xFFFF = 批量擦除
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

    switch (ucFuncFlag)
    {
    case 0x00://兼容ST,按1KBytes擦除
      eResultFlag = PageErase((uint16_t *)guc_DataBuffer, ucDataLength, 8);
      break;
//    case 0x10://PageErase
//      eResultFlag = PageErase((uint16_t *)guc_DataBuffer, ucDataLength, 1);
//      break;
    case 0x20://SectorErase
      eResultFlag = SectorErase((uint16_t *)guc_DataBuffer, ucDataLength);
      break;
    default:
      eResultFlag = ERROR;
      break;
    }
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

/**
  * @brief  系统时钟配置函数
  * @param  Value This parameter can be one of the following values:
  *         @arg @ref LL_RCC_HSICALIBRATION_4MHz
  *         @arg @ref LL_RCC_HSICALIBRATION_8MHz
  *         @arg @ref LL_RCC_HSICALIBRATION_16MHz
  *         @arg @ref LL_RCC_HSICALIBRATION_22p12MHz
  *         @arg @ref LL_RCC_HSICALIBRATION_24MHz
  * @param  HCLKFrequency HCLK frequency in Hz (can be calculated thanks to RCC helper macro)
  * @retval 无
  */
void APP_SystemClockConfig(uint32_t Value, uint32_t HCLKFrequency)
{
  /* HSI使能及初始化 */
  LL_RCC_HSI_Enable();
  LL_RCC_HSI_SetCalibFreq(Value);
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }
  
  /* 设置AHB分频 */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* 配置HSISYS为系统时钟及初始化 */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  
  /* 设置APB1分频及初始化 */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* 更新系统时钟全局变量SystemCoreClock(也可以通过调用SystemCoreClockUpdate函数更新) */
  LL_SetSystemCoreClock(HCLKFrequency);
}
