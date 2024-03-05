/**
  ******************************************************************************
  * @file    app_usart.h
  * @author  MCU Application Team
  * @brief   Header for app_usart.c module
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
#ifndef __APP_USART_H
#define __APP_USART_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void APP_USART_Init(USART_TypeDef *Instance);
void APP_USART_SendData(USART_TypeDef *Instance, uint8_t* data, uint16_t size);
void APP_USART_ReadData(USART_TypeDef *Instance, uint8_t* data, uint16_t size);

#endif /* __APP_USART_H */

/************************ (C) COPYRIGHT Puya Semiconductor *****END OF FILE****/
