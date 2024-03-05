/**
  ******************************************************************************
  * @file    app_bootloader.c
  * @author  MCU Application Team
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
/* 
S*32F072 datasheet
The boot loader is located in System Memory. It is used to reprogram the Flash memory by 
using USART on pins PA14/PA15(USART2 AF1), or PA9/PA10(USART1 AF1) or I2C on pins PB6/PB7 or through the USB 
DFU PA11/PA12 interface.

S*32F383 datasheet
The boot loader is located in the system memory. It is used to reprogram the Flash memory 
by using USART1 (PA9/PA10 AF2), USART2 (PD5/PD6 AF2) or I2C on pins PB6/PB7 or USB (PA11/PA12) through DFU 
(device firmware upgrade).

S*32F405 datasheet
The boot loader is located in system memory. It is used to reprogram the Flash memory by 
using USART1 (PA9/PA10), USART3 (PC10/PC11 or PB10/PB11), CAN2 (PB5/PB13), USB 
OTG FS in Device mode (PA11/PA12) through DFU (device firmware upgrade).

PY32F403
USART1(AF2:PA9/PA10), USART3(AF2:PB10/PB11), USART4(AF1:PC10/PC11), USART2(AF2:PD5/PD6)
I2C(PB6/PB7), USB(PA11/PA12)
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_bootloader.h"
#include "app_usart.h"
#include "app_i2c.h"
#include "app_flash.h"
#include "app_wdg.h"
#include "usb_config.h"

/* Private types ------------------------------------------------------------*/
typedef enum _INTERFACE
{
  INTERFACE_USART1 = 0,
  INTERFACE_USART2,
  INTERFACE_USART3,
  INTERFACE_USART4,
  INTERFACE_USB,
  INTERFACE_I2C,
} INTERFACE;

INTERFACE guc_InterfaceDetection;

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

const uint8_t BOOTLOADER_CMD[] =
{
  CMD_GET_COMMAND, CMD_GET_VERSION, CMD_GET_ID,
  CMD_READ_MEMORY, CMD_GO, CMD_WRITE_MEMORY, CMD_EXT_ERASE_MEMORY,
  CMD_WRITE_PROTECT, CMD_WRITE_UNPROTECT, CMD_READ_PROTECT, CMD_READ_UNPROTECT,
  CMD_GET_DEVICE_IDCODE
};

/* Private variables ---------------------------------------------------------*/
/* Private functions --------------------------------------------------------*/
ErrorStatus APP_ReadMemory(void);
ErrorStatus APP_Go(void);
ErrorStatus APP_WriteMemory(void);
ErrorStatus APP_ExtendedErase(void);
ErrorStatus APP_WriteProtect(void);
uint8_t APP_GetXOR(const uint8_t *pucDataBuf, uint16_t wDataLength, uint8_t ucBase);
static void APP_DeInitOtherInterface(void);
static void APP_DeInitInterface(void);

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Initialize Bootloader.
  * @param  None.
  * @retval None.
  */
void APP_Bootloader_Init(void)
{  
  while (1)
  {
    APP_WDG_Refresh();

    if (0x7F == (uint8_t)(READ_BIT(USART1->DR, USART_DR_DR) & 0xFFU))
    {      
      guc_InterfaceDetection = INTERFACE_USART1;
      CLEAR_BIT(USART1->CR3, USART_CR3_ABREN);
      break;
    }
    SET_BIT(USART1->SR, USART_SR_ABRRQ);
    
    if (0x7F == (uint8_t)(READ_BIT(USART2->DR, USART_DR_DR) & 0xFFU))
    {      
      guc_InterfaceDetection = INTERFACE_USART2;
      CLEAR_BIT(USART2->CR3, USART_CR3_ABREN);
      break;
    }
    SET_BIT(USART2->SR, USART_SR_ABRRQ);
    
    if (0x7F == (uint8_t)(READ_BIT(USART3->DR, USART_DR_DR) & 0xFFU))
    {      
      guc_InterfaceDetection = INTERFACE_USART3;
      CLEAR_BIT(USART3->CR3, USART_CR3_ABREN);
      break;
    }
    SET_BIT(USART3->SR, USART_SR_ABRRQ);
    
    if (0x7F == (uint8_t)(READ_BIT(USART4->DR, USART_DR_DR) & 0xFFU))
    {      
      guc_InterfaceDetection = INTERFACE_USART4;
      CLEAR_BIT(USART4->CR3, USART_CR3_ABREN);
      break;
    }
    SET_BIT(USART4->SR, USART_SR_ABRRQ);
    
    if (SUCCESS == APP_I2C_ShakeHandCheck())
    {
      guc_InterfaceDetection = INTERFACE_I2C;
      break;      
    }
    
    if (SUCCESS == hid_custom_dfu_shake_hands_check())
    {
      guc_InterfaceDetection = INTERFACE_USB;
      break;
    }
  }
  
  APP_DeInitOtherInterface();

  APP_Bootloader_SendByte(ACK_BYTE);
}

static void APP_DeInitOtherInterface(void)
{
  if (INTERFACE_USB != guc_InterfaceDetection)
  {
    __disable_irq();
  }
  
  switch (guc_InterfaceDetection)
  {
    case INTERFACE_USART1://USART1(AF2:PA9/PA10)    
      CLEAR_BIT(RCC->AHB2ENR, (RCC_AHB2ENR_IOPBEN|RCC_AHB2ENR_IOPCEN|RCC_AHB2ENR_IOPDEN));
      CLEAR_BIT(RCC->APB1ENR, (RCC_APB1ENR_USART2EN|RCC_APB1ENR_USART3EN|RCC_APB1ENR_USART4EN|RCC_APB1ENR_I2C1EN|RCC_APB1ENR_USBEN));
    
      SET_BIT(RCC->AHB2RSTR, (RCC_AHB2RSTR_IOPBRST|RCC_AHB2RSTR_IOPCRST|RCC_AHB2RSTR_IOPDRST));
      SET_BIT(RCC->APB1RSTR, (RCC_APB1RSTR_USART2RST|RCC_APB1RSTR_USART3RST|RCC_APB1RSTR_USART4RST|RCC_APB1RSTR_I2C1RST|RCC_APB1RSTR_USBRST));
      
      CLEAR_BIT(RCC->AHB2RSTR, (RCC_AHB2RSTR_IOPBRST|RCC_AHB2RSTR_IOPCRST|RCC_AHB2RSTR_IOPDRST));
      CLEAR_BIT(RCC->APB1RSTR, (RCC_APB1RSTR_USART2RST|RCC_APB1RSTR_USART3RST|RCC_APB1RSTR_USART4RST|RCC_APB1RSTR_I2C1RST|RCC_APB1RSTR_USBRST));
      break;
    
    case INTERFACE_USART2://USART2(AF2:PD5/PD6)
      CLEAR_BIT(RCC->AHB2ENR, (RCC_AHB2ENR_IOPAEN|RCC_AHB2ENR_IOPBEN|RCC_AHB2ENR_IOPCEN));
      CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
      CLEAR_BIT(RCC->APB1ENR, (RCC_APB1ENR_USART3EN|RCC_APB1ENR_USART4EN|RCC_APB1ENR_I2C1EN|RCC_APB1ENR_USBEN));
          
      SET_BIT(RCC->AHB2RSTR, (RCC_AHB2RSTR_IOPARST|RCC_AHB2RSTR_IOPBRST|RCC_AHB2RSTR_IOPCRST));
      SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
      SET_BIT(RCC->APB1RSTR, (RCC_APB1RSTR_USART3RST|RCC_APB1RSTR_USART4RST|RCC_APB1RSTR_I2C1RST|RCC_APB1RSTR_USBRST));
            
      CLEAR_BIT(RCC->AHB2RSTR, (RCC_AHB2RSTR_IOPARST|RCC_AHB2RSTR_IOPBRST|RCC_AHB2RSTR_IOPCRST));
      CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
      CLEAR_BIT(RCC->APB1RSTR, (RCC_APB1RSTR_USART3RST|RCC_APB1RSTR_USART4RST|RCC_APB1RSTR_I2C1RST|RCC_APB1RSTR_USBRST));
      break;
    
    case INTERFACE_USART3://USART3(AF2:PB10/PB11)
      CLEAR_BIT(RCC->AHB2ENR, (RCC_AHB2ENR_IOPAEN|RCC_AHB2ENR_IOPCEN|RCC_AHB2ENR_IOPDEN));
      CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
      CLEAR_BIT(RCC->APB1ENR, (RCC_APB1ENR_USART2EN|RCC_APB1ENR_USART4EN|RCC_APB1ENR_I2C1EN|RCC_APB1ENR_USBEN));
    
      SET_BIT(RCC->AHB2RSTR, (RCC_AHB2RSTR_IOPARST|RCC_AHB2RSTR_IOPCRST|RCC_AHB2RSTR_IOPDRST));
      SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
      SET_BIT(RCC->APB1RSTR, (RCC_APB1RSTR_USART2RST|RCC_APB1RSTR_USART4RST|RCC_APB1RSTR_I2C1RST|RCC_APB1RSTR_USBRST));
      
      CLEAR_BIT(RCC->AHB2RSTR, (RCC_AHB2RSTR_IOPARST|RCC_AHB2RSTR_IOPCRST|RCC_AHB2RSTR_IOPDRST));
      CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
      CLEAR_BIT(RCC->APB1RSTR, (RCC_APB1RSTR_USART2RST|RCC_APB1RSTR_USART4RST|RCC_APB1RSTR_I2C1RST|RCC_APB1RSTR_USBRST));
      break;
    
    case INTERFACE_USART4://USART4(AF1:PC10/PC11)
      CLEAR_BIT(RCC->AHB2ENR, (RCC_AHB2ENR_IOPAEN|RCC_AHB2ENR_IOPBEN|RCC_AHB2ENR_IOPDEN));
      CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
      CLEAR_BIT(RCC->APB1ENR, (RCC_APB1ENR_USART2EN|RCC_APB1ENR_USART3EN|RCC_APB1ENR_I2C1EN|RCC_APB1ENR_USBEN));
          
      SET_BIT(RCC->AHB2RSTR, (RCC_AHB2RSTR_IOPARST|RCC_AHB2RSTR_IOPBRST|RCC_AHB2RSTR_IOPDRST));
      SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
      SET_BIT(RCC->APB1RSTR, (RCC_APB1RSTR_USART2RST|RCC_APB1RSTR_USART3RST|RCC_APB1RSTR_I2C1RST|RCC_APB1RSTR_USBRST));
            
      CLEAR_BIT(RCC->AHB2RSTR, (RCC_AHB2RSTR_IOPARST|RCC_AHB2RSTR_IOPBRST|RCC_AHB2RSTR_IOPDRST));
      CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
      CLEAR_BIT(RCC->APB1RSTR, (RCC_APB1RSTR_USART2RST|RCC_APB1RSTR_USART3RST|RCC_APB1RSTR_I2C1RST|RCC_APB1RSTR_USBRST));
      break;
    
    case INTERFACE_I2C://I2C(AF1:PB6/PB7)
      CLEAR_BIT(RCC->AHB2ENR, (RCC_AHB2ENR_IOPAEN|RCC_AHB2ENR_IOPCEN|RCC_AHB2ENR_IOPDEN));
      CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
      CLEAR_BIT(RCC->APB1ENR, (RCC_APB1ENR_USART2EN|RCC_APB1ENR_USART3EN|RCC_APB1ENR_USART4EN|RCC_APB1ENR_USBEN));
          
      SET_BIT(RCC->AHB2RSTR, (RCC_AHB2RSTR_IOPARST|RCC_AHB2RSTR_IOPCRST|RCC_AHB2RSTR_IOPDRST));
      SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
      SET_BIT(RCC->APB1RSTR, (RCC_APB1RSTR_USART2RST|RCC_APB1RSTR_USART3RST|RCC_APB1RSTR_USART4RST|RCC_APB1RSTR_USBRST));
            
      CLEAR_BIT(RCC->AHB2RSTR, (RCC_AHB2RSTR_IOPARST|RCC_AHB2RSTR_IOPCRST|RCC_AHB2RSTR_IOPDRST));
      CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
      CLEAR_BIT(RCC->APB1RSTR, (RCC_APB1RSTR_USART2RST|RCC_APB1RSTR_USART3RST|RCC_APB1RSTR_USART4RST|RCC_APB1RSTR_USBRST));
      break;
    
    case INTERFACE_USB://USB(PA11/PA12)
      CLEAR_BIT(RCC->AHB2ENR, (RCC_AHB2ENR_IOPBEN|RCC_AHB2ENR_IOPCEN|RCC_AHB2ENR_IOPDEN));
      CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
      CLEAR_BIT(RCC->APB1ENR, (RCC_APB1ENR_USART2EN|RCC_APB1ENR_USART3EN|RCC_APB1ENR_USART4EN|RCC_APB1ENR_I2C1EN));
          
      SET_BIT(RCC->AHB2RSTR, (RCC_AHB2RSTR_IOPBRST|RCC_AHB2RSTR_IOPCRST|RCC_AHB2RSTR_IOPDRST));
      SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
      SET_BIT(RCC->APB1RSTR, (RCC_APB1RSTR_USART2RST|RCC_APB1RSTR_USART3RST|RCC_APB1RSTR_USART4RST|RCC_APB1RSTR_I2C1RST));
            
      CLEAR_BIT(RCC->AHB2RSTR, (RCC_AHB2RSTR_IOPBRST|RCC_AHB2RSTR_IOPCRST|RCC_AHB2RSTR_IOPDRST));
      CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
      CLEAR_BIT(RCC->APB1RSTR, (RCC_APB1RSTR_USART2RST|RCC_APB1RSTR_USART3RST|RCC_APB1RSTR_USART4RST|RCC_APB1RSTR_I2C1RST));
      break;
  }
}

static void APP_DeInitInterface(void)
{
  if (INTERFACE_USB != guc_InterfaceDetection)
  {
    __enable_irq();
  }
  
  switch (guc_InterfaceDetection)
  {
    case INTERFACE_USART1://USART1(AF2:PA9/PA10)
      CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_IOPAEN);
      CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
          
      SET_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_IOPARST);
      SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
            
      CLEAR_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_IOPARST);
      CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
      break;
    
    case INTERFACE_USART2://USART2(AF2:PD5/PD6)
      CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_IOPDEN);
      CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
          
      SET_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_IOPDRST);
      SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USART2RST);
            
      CLEAR_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_IOPDRST);
      CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USART2RST);
      break;
    
    case INTERFACE_USART3://USART3(AF2:PB10/PB11)
      CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_IOPBEN);
      CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);
          
      SET_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_IOPBRST);
      SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USART3RST);
            
      CLEAR_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_IOPBRST);
      CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USART3RST);
      break;
    
    case INTERFACE_USART4://USART4(AF1:PC10/PC11)
      CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_IOPCEN);
      CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART4EN);
          
      SET_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_IOPCRST);
      SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USART4RST);
            
      CLEAR_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_IOPCRST);
      CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USART4RST);
      break;
    
    case INTERFACE_I2C://I2C(AF1:PB6/PB7)
      CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_IOPBEN);
      CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
          
      SET_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_IOPBRST);
      SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_I2C1RST);
            
      CLEAR_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_IOPBRST);
      CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_I2C1RST);
      break;
    
    case INTERFACE_USB://USB(PA11/PA12)
      CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_IOPAEN);
      CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN);
          
      SET_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_IOPARST);
      SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USBRST);
            
      CLEAR_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_IOPARST);
      CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USBRST);
      break;
  }
  
  CLEAR_BIT(RCC->CFGR, RCC_CFGR_SW);
  while (0 != READ_BIT(RCC->CFGR, RCC_CFGR_SWS));
  
  CLEAR_BIT(RCC->CR, RCC_CR_PLLON);  
}

/**
  * @brief  This function is used to select which protocol will be used when communicating with the host.
  * @param  None.
  * @retval None.
  */
void APP_Bootloader_ProtocolDetection(void)
{
  uint8_t i;
  uint8_t ucCommand;
  ErrorStatus eStatus = SUCCESS;
  uint8_t ucDataBuffer[0x12];
  uint8_t ucTxDataBuffer[0x10];
  uint32_t uiTxDataLength = 0;

#if 1//获取命令
  APP_Bootloader_ReadData(ucDataBuffer, 2);
  ucCommand = ucDataBuffer[0];
  /* Check the data integrity */
  if ((ucCommand ^ ucDataBuffer[1]) != 0xFF)
  {
    APP_Bootloader_SendByte(NACK_BYTE);
    return;
  }
#endif

#if 1
  switch (ucCommand)
  {
  case CMD_WRITE_MEMORY:
  case CMD_EXT_ERASE_MEMORY:
  case CMD_READ_MEMORY:
  case CMD_GO:
  case CMD_WRITE_PROTECT:
  case CMD_WRITE_UNPROTECT:
  case CMD_READ_PROTECT:
  case CMD_GET_COMMAND:
  case CMD_GET_VERSION:
  case CMD_GET_ID:
  case CMD_GET_DEVICE_IDCODE:
  case CMD_READ_UNPROTECT:
    APP_Bootloader_SendByte(ACK_BYTE);
    break;
  }
#endif

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
      APP_Bootloader_SendByte(NACK_BYTE);
      return;
    }
    //向 FLASH_KEYR 寄存器依次写 KEY1 和 KEY2，解除 FLASH_CR 寄存器的保护
    if (FLASH_CR_LOCK == READ_BIT(FLASH->CR, FLASH_CR_LOCK))
    {
      WRITE_REG(FLASH->KEYR, FLASH_KEY1);
      WRITE_REG(FLASH->KEYR, FLASH_KEY2);
    }
    break;
  }
#endif

#if 1//执行命令 
  switch (ucCommand)
  {
  case CMD_GET_COMMAND:
    ucTxDataBuffer[uiTxDataLength++] = COUNTOF(BOOTLOADER_CMD);
    ucTxDataBuffer[uiTxDataLength++] = VERSION;
    for (i = 0; i < COUNTOF(BOOTLOADER_CMD); i++)
    {
      ucTxDataBuffer[uiTxDataLength++] = BOOTLOADER_CMD[i];
    }
    break;

  case CMD_GET_VERSION:
    ucTxDataBuffer[uiTxDataLength++] = VERSION;
    ucTxDataBuffer[uiTxDataLength++] = 0x00;
    ucTxDataBuffer[uiTxDataLength++] = 0x00;
    break;

  case CMD_GET_ID://RETURN ST IDCODE
    ucTxDataBuffer[uiTxDataLength++] = 0x01;
    ucTxDataBuffer[uiTxDataLength++] = PID>>8;;
    ucTxDataBuffer[uiTxDataLength++] = PID&0xFF;
    break;

  case CMD_GET_DEVICE_IDCODE:
    ucTxDataBuffer[uiTxDataLength++] = 0x01;
    ucTxDataBuffer[uiTxDataLength++] = DBGMCU_IDCODE>>8;
    ucTxDataBuffer[uiTxDataLength++] = DBGMCU_IDCODE&0xFF;
    break;

  case CMD_READ_MEMORY:
    eStatus = APP_ReadMemory();
    if (SUCCESS == eStatus) {
      return;
    }
    break;

  case CMD_GO:
    eStatus = APP_Go();
    break;

  case CMD_WRITE_MEMORY:
    eStatus = APP_WriteMemory();
    break;

  case CMD_EXT_ERASE_MEMORY:
    eStatus = APP_ExtendedErase();
    break;

  case CMD_WRITE_PROTECT:
    eStatus = APP_WriteProtect();
    break;

  case CMD_WRITE_UNPROTECT:
    ucDataBuffer[0] = (FLASH_WRPR >> 0) & 0xFF;
    ucDataBuffer[1] = (FLASH_WRPR >> 8) & 0xFF;
    eStatus = APP_WriteOption(WRPR_BASE, ucDataBuffer, 0x01);
    break;

  case CMD_READ_PROTECT:
    ucDataBuffer[0] = FLASH_OPTR_RDP_LEVEL_1;
    eStatus = APP_WriteOption(OPTR_BASE, ucDataBuffer, 0x00);
    break;

  case CMD_READ_UNPROTECT:
    ucDataBuffer[0] = FLASH_OPTR_RDP_LEVEL_0;
    eStatus = APP_WriteOption(OPTR_BASE, ucDataBuffer, 0x00);
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

  ucTxDataBuffer[uiTxDataLength++] = (SUCCESS==eStatus) ? ACK_BYTE : NACK_BYTE;
  APP_Bootloader_SendData(ucTxDataBuffer, uiTxDataLength);
}

void APP_Bootloader_SendByte(uint8_t ucDataBuf)
{
  APP_Bootloader_SendData(&ucDataBuf, 1);
}

void APP_Bootloader_SendData(uint8_t* pucDataBuf, uint16_t size)
{
  switch (guc_InterfaceDetection)
  {
  case INTERFACE_USART1:
    APP_USART_SendData(USART1, pucDataBuf, size);
    break;
  case INTERFACE_USART2:
    APP_USART_SendData(USART2, pucDataBuf, size);
    break;
  case INTERFACE_USART3:
    APP_USART_SendData(USART3, pucDataBuf, size);
    break;
  case INTERFACE_USART4:
    APP_USART_SendData(USART4, pucDataBuf, size);
    break;
  case INTERFACE_USB:
    hid_custom_dfu_send_data(pucDataBuf, size);
    break;
  case INTERFACE_I2C:
    APP_I2C_SendData(pucDataBuf, size);
    break;
  }
}

uint8_t APP_Bootloader_ReadByte(void)
{
  uint8_t ucData;
  APP_Bootloader_ReadData(&ucData, 1);
  return ucData;
}

void APP_Bootloader_ReadData(uint8_t *pucDataBuf, uint16_t size)
{
  switch (guc_InterfaceDetection)
  {
  case INTERFACE_USART1:
    APP_USART_ReadData(USART1, pucDataBuf, size);
    break;
  case INTERFACE_USART2:
    APP_USART_ReadData(USART2, pucDataBuf, size);
    break;
  case INTERFACE_USART3:
    APP_USART_ReadData(USART3, pucDataBuf, size);
    break;
  case INTERFACE_USART4:
    APP_USART_ReadData(USART4, pucDataBuf, size);
    break;
  case INTERFACE_USB:
    hid_custom_dfu_read_data(pucDataBuf, size);
    break;
  case INTERFACE_I2C:
    APP_I2C_ReadData(pucDataBuf, size);
    break;
  }
}

/* Private functions --------------------------------------------------------*/

/**
  * @brief  Get the address and Check it is valid or not and returns the area type.
  * @param  pdwAddr The address to be got and checked.
  * @retval The address area: FLASH_AREA, RAM_AREA... if the address is valid
  *         or AREA_ERROR if the address is not valid.
  */
uint8_t GetAddressArea(uint32_t *pdwAddr)
{
  uint32_t dwStartAddr;
  uint32_t dwEndAddr;
  uint8_t ucDataBuffer[0x05];

  APP_Bootloader_ReadData(ucDataBuffer, 0x05);
  if (ucDataBuffer[0x04] != APP_GetXOR(ucDataBuffer, 0x04, 0x00))
  {
    return AREA_ERROR;
  }
  APP_Bootloader_SendByte(ACK_BYTE);
  *pdwAddr = (ucDataBuffer[0] << 24) + (ucDataBuffer[1] << 16) + (ucDataBuffer[2] << 8) + ucDataBuffer[3];

  switch (*pdwAddr)
  {
//  case STM32F0_FLASHSIZE_BASE:
  case STM32F1_FLASHSIZE_BASE:
  case STM32F3_FLASHSIZE_BASE:
  case STM32F4_FLASHSIZE_BASE:
//    *pdwAddr = FLASHSIZE_BASE;
    return OB_AREA;
//  case STM32F0_UID_BASE:
  case STM32F1_UID_BASE:
  case STM32F3_UID_BASE:
  case STM32F4_UID_BASE:
    *pdwAddr = UID_BASE;
    break;
  case STM32F0_OB_BASE:
  case STM32F4_OB_BASE:
    *pdwAddr = OB_BASE;
    break;
  case STM32F0_BID_BASE:
  case STM32F4_BID_BASE:
    *pdwAddr = BID_BASE;
    break;
  }

  dwStartAddr = FLASH_BASE;
  dwEndAddr = FLASH_END;
  if ((*pdwAddr >= dwStartAddr) && (*pdwAddr < dwEndAddr))
  {
    return FLASH_AREA;
  }

  dwStartAddr = SRAM_BASE;
  dwEndAddr = SRAM_END;
  if ((*pdwAddr >= dwStartAddr) && (*pdwAddr < dwEndAddr))
  {
    return RAM_AREA;
  }

  dwStartAddr = SYSTEM_BASE;
  dwEndAddr = SYSTEM_BASE + 20 * 0x400;
  if ((*pdwAddr >= dwStartAddr) && (*pdwAddr < dwEndAddr))
  {
    return SYS_AREA;
  }

  dwStartAddr = OB_BASE;
  dwEndAddr = OB_BASE + 0x100;
  if ((*pdwAddr >= dwStartAddr) && (*pdwAddr < dwEndAddr))
  {
    return OB_AREA;
  }
  
  dwStartAddr = 0x1FFF5100;
  dwEndAddr = 0x1FFF5A00;
  if ((*pdwAddr >= dwStartAddr) && (*pdwAddr < dwEndAddr))
  {
    return OTP_AREA;
  }

  return AREA_ERROR;
}

/**
 * @brief  This function is used to read memory from the device.
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: ReadMemory operation done
 *          - ERROR:   ReadMemory operation failed or the value of address is not ok
 */
ErrorStatus APP_ReadMemory(void)
{
  uint32_t dwAddr;
  uint8_t ucDataBuffer[0x05];
  uint16_t wFlashSize;
  uint8_t ucAddrArea;
  
  ucAddrArea = GetAddressArea(&dwAddr);

  if (AREA_ERROR == ucAddrArea)
  {
    return ERROR;
  }

  APP_Bootloader_ReadData(ucDataBuffer, 0x02);
  if ((ucDataBuffer[0] ^ ucDataBuffer[1]) != 0xFF)
  {
    return ERROR;
  }
  APP_Bootloader_SendByte(ACK_BYTE);
  
  switch (dwAddr)
  {
//  case STM32F0_FLASHSIZE_BASE:
  case STM32F1_FLASHSIZE_BASE:
  case STM32F3_FLASHSIZE_BASE:
  case STM32F4_FLASHSIZE_BASE:
    wFlashSize = (((HW8_REG(FLASHSIZE_BASE) & 0x1F) >> 0) + 1) * 16;
    wFlashSize = min(wFlashSize, 384);
    ucDataBuffer[1] = ((wFlashSize >> 0) & 0xFF);
    ucDataBuffer[2] = ((wFlashSize >> 8) & 0xFF);
    ucDataBuffer[3] = (((HW8_REG(FLASHSIZE_BASE) & 0xE0) >> 5) + 1) * 8;
    ucDataBuffer[4] = 0x00;
    APP_Bootloader_SendData(&ucDataBuffer[1], ucDataBuffer[0] + 1);
    return SUCCESS;
  }
  
  if ( (FLASH_AREA == ucAddrArea) && (FLASH_OPTR_RDP_LEVEL_0 != READ_BIT(FLASH->OPTR, FLASH_OPTR_RDP)) )
  {
    APP_Bootloader_SendData((uint8_t*)0, ucDataBuffer[0] + 1);
    return SUCCESS;
  }

  APP_Bootloader_SendData((uint8_t*)dwAddr, ucDataBuffer[0] + 1);

  return SUCCESS;
}

typedef  void (*pFunction)(void);
void APP_Bootloader_Go(uint32_t dwAddr)
{
  uint32_t JumpAddress;
  pFunction Jump_To_Application;;

  if (((*(__IO uint32_t*)dwAddr) & 0x2FFE0000 ) == 0x20000000)
  {
    /* Jump to user application */
    JumpAddress = *(__IO uint32_t*) (dwAddr + 4);
    Jump_To_Application = (pFunction) JumpAddress;
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t*) dwAddr);
    Jump_To_Application();
  }
  
  NVIC_SystemReset();  
}

/**
 * @brief  This function is used to jump to the user application.
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: Go operation done
 *          - ERROR:   Go operation failed or the value of address is not ok
 */
ErrorStatus APP_Go(void)
{
  uint8_t ucMemArea;
  uint32_t dwAddr;

  ucMemArea = GetAddressArea(&dwAddr);
  if ((ucMemArea != FLASH_AREA) && (ucMemArea != RAM_AREA))
  {
    return ERROR;
  }
  
  APP_DeInitInterface();

  APP_Bootloader_Go(dwAddr);

  return SUCCESS;
}

/**
 * @brief  This function is used to write in to device memory.
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: WriteMemory operation done
 *          - ERROR:   WriteMemory operation failed or the value of address is not ok
 */
ErrorStatus APP_WriteMemory(void)
{
  uint16_t i;
  uint8_t ucMemArea;
  uint8_t ucDataLength;
  uint32_t dwAddr;
  ErrorStatus eResultFlag;
  uint8_t ucDataBuffer[0x201];

  ucMemArea = GetAddressArea(&dwAddr);
  if ((ucMemArea != FLASH_AREA) && (ucMemArea != RAM_AREA) && (ucMemArea != OB_AREA))
  {
    return ERROR;
  }

  ucDataLength = APP_Bootloader_ReadByte();
  APP_Bootloader_ReadData(ucDataBuffer, (ucDataLength + 1) + 1);

  if (ucDataBuffer[ucDataLength + 1] != APP_GetXOR(ucDataBuffer, ucDataLength + 1, ucDataLength))
  {
    return ERROR;
  }

  switch (ucMemArea)
  {
  case FLASH_AREA:
    eResultFlag = APP_WriteFlash(dwAddr, ucDataBuffer, ucDataLength);
    break;
  case RAM_AREA:
    for (i = 0; i < ucDataLength + 1; i++)
    {
      HW8_REG(dwAddr + i) = ucDataBuffer[i];
    }
    eResultFlag = SUCCESS;
    break;
  case OB_AREA:
    eResultFlag = APP_WriteOption(dwAddr, ucDataBuffer, ucDataLength);
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
ErrorStatus APP_ExtendedErase(void)
{
  uint8_t i;
  uint8_t ucXOR;
  uint8_t ucDataTemp;
  uint8_t ucFuncFlag;
  uint8_t ucDataLength;
  ErrorStatus eResultFlag = ERROR;
  uint8_t ucDataBuffer[0x40];

  /* Read number of pages to be erased */
  APP_Bootloader_ReadData(ucDataBuffer, 0x02);
  ucFuncFlag = ucDataBuffer[0];
  ucDataLength = ucDataBuffer[1];

  /* Checksum initialization */
  ucXOR  = ucFuncFlag;
  ucXOR ^= ucDataLength;

  if (0xFF == ucFuncFlag)//0xFFFY, Y=F,E,D
  {
    APP_Bootloader_ReadData(&ucDataBuffer[2], 1);
    //接收双字节的校验和
    if (ucDataBuffer[2] != ucXOR)
    {
      return ERROR;
    }

    return APP_MassErase();//0xFFFF = 批量擦除
  }
  else
  {
    APP_Bootloader_ReadData(ucDataBuffer, 2 * (ucDataLength + 1) + 1);
    ucXOR = APP_GetXOR(ucDataBuffer, 2 * (ucDataLength + 1), ucXOR);

    if (ucDataBuffer[2 * (ucDataLength + 1)] != ucXOR)
    {
      return ERROR;
    }

    for (i = 0; i < ucDataLength + 1; i++)
    {
      ucDataTemp = ucDataBuffer[2 * i + 0];
      ucDataBuffer[2 * i + 0] = ucDataBuffer[2 * i + 1];
      ucDataBuffer[2 * i + 1] = ucDataTemp;
    }

    switch (ucFuncFlag)
    {
    case 0x00://兼容ST,按1KBytes擦除
      eResultFlag = APP_PageErase((uint16_t *)ucDataBuffer, ucDataLength, 0x400/FLASH_PAGE_SIZE);
      break;
    case 0x10://PageErase
      eResultFlag = APP_PageErase((uint16_t *)ucDataBuffer, ucDataLength, 1);
      break;
    case 0x20://SectorErase
      eResultFlag = APP_SectorErase((uint16_t *)ucDataBuffer, ucDataLength);
      break;
    case 0x30://BlockErase
      eResultFlag = APP_BlockErase((uint16_t *)ucDataBuffer, ucDataLength);
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
ErrorStatus APP_WriteProtect(void)
{
  uint16_t i;
  uint8_t ucDataLength;
  uint16_t ProtectedPages = 0xFFFF;
  uint8_t ucDataBuffer[0x10];

  ucDataLength = APP_Bootloader_ReadByte();

  APP_Bootloader_ReadData(ucDataBuffer, (ucDataLength + 1) + 1);

  if (ucDataBuffer[ucDataLength + 1] != APP_GetXOR(ucDataBuffer, ucDataLength + 1, ucDataLength))
  {
    return ERROR;
  }

  for (i = 0; i < ucDataLength + 1; i++)
  {
    CLEAR_BIT(ProtectedPages, (1U << ucDataBuffer[i]));
  }

  ucDataBuffer[0] = ProtectedPages & 0xFF;
  ucDataBuffer[1] = (ProtectedPages >> 8) & 0xFF;

  return APP_WriteOption(WRPR_BASE, ucDataBuffer, 0x01);
}

/**
 * @brief  This function is used to get XOR of the DataBuf.
 * @param *pucDataBuf Pointer to the DataBuf
 * @param wDataLength The length of the DataBuf
 * @param ucBase The base value of the DataBuf
 * @retval The XOR of the DataBuf
 */
uint8_t APP_GetXOR(const uint8_t *pucDataBuf, uint16_t wDataLength, uint8_t ucBase)
{
  while (wDataLength--)
  {
    ucBase = ucBase ^*pucDataBuf++;
  }
  return ucBase;
}
