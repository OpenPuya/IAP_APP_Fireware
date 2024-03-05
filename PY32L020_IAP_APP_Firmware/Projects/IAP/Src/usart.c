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

#define DEFAULT_BAUDRATE                  115200            /* Default Baudrate */
#define DEFAULT_BRR                       ((SystemCoreClock+DEFAULT_BAUDRATE/2)/DEFAULT_BAUDRATE)

/* Exported functions --------------------------------------------------------*/
void USART_Init(void)
{
  SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);
  SET_BIT(RCC->APBENR2, RCC_APBENR2_USART1EN);
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD4, GPIO_PUPDR_PUPD4_0);//01: 上拉
  CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE4_0);  //10: 复用功能模式
  SET_BIT(GPIOA->AFR[0], GPIO_AFRL_AFSEL4_0);  //0001:AF1 USART1_TX(PA3) USART1_RX(PA4)
  
  //USART_CR1_M 1： 1 start bit， 9 data bit， n stop bit
  //USART_CR1_PCE 1：奇偶校验使能
  //USART_CR1_PS  0：偶校验(EVEN)
  //USART_CR2_STOP 00： 1 stop bit
  SET_BIT(USART1->CR1, (USART_CR1_M | USART_CR1_PCE));

  WRITE_REG(USART1->BRR, DEFAULT_BRR);

  SET_BIT(USART1->CR1, USART_CR1_UE);//1： USART 使能

  //USART_CR3_ABRMODE 00：从 start 位开始测量波特率
  //USART_CR3_ABREN 1：自动波特率使能
  MODIFY_REG(USART1->CR3, USART_CR3_ABRMODE, USART_CR3_ABREN);

  SET_BIT(USART1->CR1, USART_CR1_RE); //1： 接收使能
}

/**
  * @brief  This function is used to send one byte through USART pipe.
  * @param  ucDataBuf The byte to be sent.
  * @retval None.
  */
void USART_SendByte(uint8_t ucDataBuf)
{
  USART1->DR = (ucDataBuf & (uint16_t)0x01FF);
  
  while (USART_SR_TC != READ_BIT(USART1->SR, USART_SR_TC))
  {
    WDG_Refresh();
  }
  
  CLEAR_BIT(USART1->SR, USART_SR_TC);
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
