/**
  ******************************************************************************
  * @file    Bootloader/Inc/protocol.h
  * @author  Puya Application Team
  * @brief   Header for protocol.c module
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
#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define APP_ADDR        0x08001000

#define JUMP_TO_APP_BY_USER_BUTTON
//#define JUMP_TO_APP_BY_TIME_OUT
#define MAX_TIME_OUT    1000000

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define BID_BASE                          (FLASH_BASE + 0x000000FCUL)
#define BID                               (0x23041900 + 0x10)
#define VERSION                           0x10

#define ERROR_COMMAND                     0xEC             /* Error command */
#define ACK_BYTE                          0x79             /* Acknowledge Byte ID */
#define NACK_BYTE                         0x1F             /* No Acknowledge Byte ID */
#define BUSY_BYTE                         0x76             /* Busy Byte */
#define SYNC_BYTE                         0xA5             /* synchronization byte */

#define CMD_GET_COMMAND                   0x00             /* Get commands command */
#define CMD_GET_ID                        0x02             /* Get ID command */
#define CMD_READ_MEMORY                   0x11             /* Read Memory command */
#define CMD_WRITE_MEMORY                  0x31             /* Write Memory command */
#define CMD_GO                            0x21             /* GO command */
#define CMD_EXT_ERASE_MEMORY              0x44             /* Erase Memory command */

/* Exported constants --------------------------------------------------------*/
/* Exported variable ------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Bootloader_Init(void);
void Bootloader_ProtocolDetection(void);
void JumpToAddress(uint32_t dwAddr);

#endif /* __BOOTLOADER_H */

/************************ (C) COPYRIGHT Puya Semiconductor *****END OF FILE****/
