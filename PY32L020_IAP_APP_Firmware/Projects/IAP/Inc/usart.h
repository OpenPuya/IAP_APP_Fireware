/**
  ******************************************************************************
  * @file    usart.h
  * @author  Puya Application Team
  * @brief   Header for usart.c module
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
#ifndef __USART_H
#define __USART_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void USART_Init(void);
void USART_SendByte(uint8_t ucDataBuf);
uint8_t USART_ReadByte(void);

#endif /* __USART_H */

/************************ (C) COPYRIGHT Puya Semiconductor *****END OF FILE****/
