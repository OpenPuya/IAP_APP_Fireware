/**
  ******************************************************************************
  * @file    app_bootloader.h
  * @author  MCU Application Team
  * @brief   Header for app_bootloader.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 Puya Semiconductor.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_BOOTLOADER_H
#define __APP_BOOTLOADER_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* 
自举程序标识符（ ID） ： STM32器件自举程序的版本，以0xXY形式的单字节代码表示，
其中：
– X指定器件自举程序所用的嵌入式串行外设：
X = 1：使用一个USART
X = 2：使用两个USART
X = 3：使用USART、 CAN和DFU
X = 4：使用USART和DFU
X = 5：使用USART和I2C
X = 6：使用I2C
X = 7：使用USART、 CAN、 DFU和I2C
X = 8：使用I2C和SPI
X = 9：使用USART、 CAN（或FDCAN）、 DFU、 I2C和SPI
X = 10：使用USART、 DFU和I2C
X = 11：使用USART、 I2C和SPI
X = 12：使用USART和SPI
X = 13：使用USART、 DFU、 I2C和SPI
– Y指定器件的自举程序版本
下面以自举程序ID 0x10为例。这表示仅使用一个USART的器件自举程序的第一个
版本。
自举程序ID编程在器件系统存储器最后一个字节地址减1所对应的空间中，可通
过自举程序“Read memory”命令来读取，或者通过使用JTAG/SWD直接访问系统
存储器来读取。*/
#define BID_BASE                          (VECT_TAB_BASE + 0x00000BFCUL)
#define BID                               (0x23061800 + 0xA0)
#define PID                               0x0413
#define VERSION                           0x10

#define ERROR_COMMAND                     0xEC             /* Error command */
#define ACK_BYTE                          0x79             /* Acknowledge Byte ID */
#define NACK_BYTE                         0x1F             /* No Acknowledge Byte ID */
#define BUSY_BYTE                         0x76             /* Busy Byte */
#define SYNC_BYTE                         0xA5             /* synchronization byte */

#define CMD_GET_COMMAND                   0x00            /* Get commands command */
#define CMD_GET_VERSION                   0x01             /* Get Version command */
#define CMD_GET_ID                        0x02             /* Get ID command */
#define CMD_GET_DEVICE_IDCODE             0x03             /* Get DEVICE_IDCODE command */
#define CMD_READ_MEMORY                   0x11             /* Read Memory command */
#define CMD_WRITE_MEMORY                  0x31             /* Write Memory command */
#define CMD_GO                            0x21             /* GO command */
#define CMD_READ_PROTECT                  0x82             /* Readout Protect command */
#define CMD_READ_UNPROTECT                0x92             /* Readout Unprotect command */
#define CMD_EXT_ERASE_MEMORY              0x44             /* Erase Memory command */
#define CMD_WRITE_PROTECT                 0x63             /* Write Protect command */
#define CMD_WRITE_UNPROTECT               0x73             /* Write Unprotect command */

#define APP_ADDR        0x08004000
/* Exported constants --------------------------------------------------------*/
/* Exported variable ------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void APP_Bootloader_Init(void);
void APP_Bootloader_ProtocolDetection(void);

void APP_Bootloader_Go(uint32_t dwAddr);

void APP_Bootloader_SendByte(uint8_t ucDataBuf);
uint8_t APP_Bootloader_ReadByte(void);
void APP_Bootloader_SendData(uint8_t* pucDataBuf, uint16_t size);
void APP_Bootloader_ReadData(uint8_t* pucDataBuf, uint16_t size);

#endif /* __APP_BOOTLOADER_H */

/************************ (C) COPYRIGHT Puya Semiconductor *****END OF FILE****/
