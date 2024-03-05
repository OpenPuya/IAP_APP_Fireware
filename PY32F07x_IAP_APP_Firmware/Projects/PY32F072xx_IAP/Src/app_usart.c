/**
  ******************************************************************************
  * @file    app_usart.c
  * @author  MCU Application Team
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
#include "app_usart.h"
#include "app_wdg.h"

#define DEFAULT_BAUDRATE                  115200            /* Default Baudrate */
#define DEFAULT_BRR                       ((SystemCoreClock+DEFAULT_BAUDRATE/2)/DEFAULT_BAUDRATE)

/* Exported functions --------------------------------------------------------*/
void APP_USART_InitRx(USART_TypeDef *Instance)
{
  if (USART1 == Instance)
  {
    SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD10, GPIO_PUPDR_PUPD10_0);//01: 上拉
    CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE10_0); //10: 复用功能模式
    SET_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL10_0);  //0001:AF1 USART1_TX(PA9) USART1_RX(PA10)
    SET_BIT(RCC->APBENR2, RCC_APBENR2_USART1EN);
  }
  
  if (USART2 == Instance)
  {
    SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD15, GPIO_PUPDR_PUPD15_0);//01: 上拉    
    CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE15_0); //10: 复用功能模式
    SET_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL15_0);  //0001:AF1 USART2_TX(PA14) USART2_RX(PA15)
    SET_BIT(RCC->APBENR1, RCC_APBENR1_USART2EN);
  }
  
  if (USART3 == Instance)
  {
    SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);
    MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD11, GPIO_PUPDR_PUPD11_0);//01: 上拉    
    CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE11_0); //10: 复用功能模式
    SET_BIT(GPIOB->AFR[1], GPIO_AFRH_AFSEL11_2);  //0100:AF4 USART3_TX(PB10) USART3_RX(PB11)
    SET_BIT(RCC->APBENR1, RCC_APBENR1_USART3EN);
  }
  
  if (USART4 == Instance)
  {
    SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOCEN);
    MODIFY_REG(GPIOC->PUPDR, GPIO_PUPDR_PUPD11, GPIO_PUPDR_PUPD11_0);//01: 上拉    
    CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODE11_0); //10: 复用功能模式
    CLEAR_BIT(GPIOC->AFR[1], GPIO_AFRH_AFSEL11);  //0000:AF0 USART4_TX(PC10) USART4_RX(PC11)
    SET_BIT(RCC->APBENR1, RCC_APBENR1_USART4EN);
  }

  //USART_CR1_M 1： 1 start bit， 9 data bit， n stop bit
  //USART_CR1_PCE 1：奇偶校验使能
  //USART_CR1_PS  0：偶校验(EVEN)
  //USART_CR2_STOP 00： 1 stop bit
  SET_BIT(Instance->CR1, (USART_CR1_M | USART_CR1_PCE));

  WRITE_REG(Instance->BRR, DEFAULT_BRR);

  SET_BIT(Instance->CR1, USART_CR1_UE);//1： USART 使能

  //USART_CR3_ABRMODE 00：从 start 位开始测量波特率
  //USART_CR3_ABREN 1：自动波特率使能
  MODIFY_REG(Instance->CR3, USART_CR3_ABRMODE, USART_CR3_ABREN);

  SET_BIT(Instance->CR1, USART_CR1_RE); //1： 接收使能
}

void APP_USART_InitTx(USART_TypeDef *Instance)
{
  CLEAR_BIT(Instance->CR3, USART_CR3_ABREN);
  
  if (USART1 == Instance)
  {
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD9, GPIO_PUPDR_PUPD9_0);//01: 上拉
    CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE9_0); //10: 复用功能模式
    SET_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL9_0);  //0001:AF1 USART1_TX(PA9) USART1_RX(PA10)
  }
  
  if (USART2 == Instance)
  {
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD14, GPIO_PUPDR_PUPD14_0);//01: 上拉
    CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE14_0); //10: 复用功能模式
    SET_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL14_0);  //0001:AF1 USART2_TX(PA14) USART2_RX(PA15)
  }
  
  if (USART3 == Instance)
  {
    MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD10, GPIO_PUPDR_PUPD10_0);//01: 上拉
    CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE10_0); //10: 复用功能模式
    SET_BIT(GPIOB->AFR[1], GPIO_AFRH_AFSEL10_2);  //0100:AF4 USART3_TX(PB10) USART3_RX(PB11)
  }
  
  if (USART4 == Instance)
  {
    MODIFY_REG(GPIOC->PUPDR, GPIO_PUPDR_PUPD10, GPIO_PUPDR_PUPD10_0);//01: 上拉
    CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODE10_0); //10: 复用功能模式
    CLEAR_BIT(GPIOC->AFR[1], GPIO_AFRH_AFSEL10);  //0000:AF0 USART4_TX(PC10) USART4_RX(PC11)
  }

  SET_BIT(Instance->CR1, USART_CR1_TE); //1： 传送使能
}

/**
  * @brief  This function is used to send one byte through USART pipe.
  * @param  data The data to be sent.
  * @param  size The data size to be sent.
  * @retval None.
  */
void APP_USART_SendData(USART_TypeDef *Instance, uint8_t* data, uint16_t size)
{
  while (size--)
  {
    Instance->DR = ((*data++) & (uint16_t)0x01FF);
    
    while (USART_SR_TXE != READ_BIT(Instance->SR, USART_SR_TXE))
    {
      APP_WDG_Refresh();
    }
  }
  
  while (USART_SR_TC != READ_BIT(Instance->SR, USART_SR_TC))
  {
    APP_WDG_Refresh();
  }
  
  CLEAR_BIT(Instance->SR, USART_SR_TC);
}

/**
  * @brief  This function is used to read one byte from USART pipe.
  * @param  data The data to be read.
  * @param  size The data size to be read.
  */
void APP_USART_ReadData(USART_TypeDef *Instance, uint8_t* data, uint16_t size)
{
  while (size--)
  {
    while (USART_SR_RXNE != READ_BIT(Instance->SR, USART_SR_RXNE))
    {
      APP_WDG_Refresh();
    }
    
    *data++ = (uint8_t)(READ_BIT(Instance->DR, USART_DR_DR) & 0xFFU);    
  }
}
