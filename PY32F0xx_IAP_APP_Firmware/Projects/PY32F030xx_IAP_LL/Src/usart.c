/**
  ******************************************************************************
  * @file    usart.c
  * @author  Puya Application Team
  * @brief   Contains USART HW configuration
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
#include "usart.h"
#include "wdg.h"

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  This function is used to send one byte through USART pipe.
  * @param  ucDataBuf The byte to be sent.
  * @retval None.
  */
void USART_SendByte(uint8_t ucDataBuf)
{
  USART1->SR;//CLEAR_BIT(USART1->SR, USART_SR_TC);
  USART1->DR = (ucDataBuf & (uint16_t)0x01FF);
  while (USART_SR_TC != READ_BIT(USART1->SR, USART_SR_TC))
  {
//    WDG_Refresh();
  }
}

/**
  * @brief  This function is used to read one byte from USART pipe.
  * @retval Returns the read byte.
  */
uint8_t USART_ReadByte(void)
{
  while (USART_SR_RXNE != READ_BIT(USART1->SR, USART_SR_RXNE))
  {
    WDG_Refresh();
  }
  return (uint8_t)(READ_BIT(USART1->DR, USART_DR_DR) & 0xFFU);
}

#ifdef _DEBUG
//重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
  /* 发送一个字节数据到串口DEBUG_USART */
  USART_SendByte(ch);

  return (ch);
}
#endif
