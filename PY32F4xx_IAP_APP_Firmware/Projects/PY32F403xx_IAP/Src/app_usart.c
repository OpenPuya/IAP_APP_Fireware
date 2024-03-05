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

#define MAX_BAUDRATE                      (1000000+100)
#define MIN_BAUDRATE                      (1200-100)
#define DEFAULT_BAUDRATE                  115200            /* Default Baudrate */
#define MAX_BRR                           ((SystemCoreClock+MAX_BAUDRATE/2)/MAX_BAUDRATE)
#define MIN_BRR                           ((SystemCoreClock+MIN_BAUDRATE/2)/MIN_BAUDRATE)
#define DEFAULT_BRR                       ((SystemCoreClock+DEFAULT_BAUDRATE/2)/DEFAULT_BAUDRATE)

/* Exported functions --------------------------------------------------------*/
void APP_USART_Init(USART_TypeDef *Instance)
{
  switch ((uint32_t)Instance)
  {
    case (uint32_t)USART1:      
      SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_IOPAEN);
      SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
      MODIFY_REG(GPIOA->PUPDR, (GPIO_PUPDR_PUPD9|GPIO_PUPDR_PUPD10), (GPIO_PUPDR_PUPD9_0|GPIO_PUPDR_PUPD10_0));//01: 上拉
      CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODE9_0 | GPIO_MODER_MODE10_0)); //10: 复用功能模式
      SET_BIT(GPIOA->AFR[1], (GPIO_AFRH_AFSEL9_1 | GPIO_AFRH_AFSEL10_1));  //0010:AF2 USART1_TX(PA9) USART1_RX(PA10)      
      break;
    
    case (uint32_t)USART2:
      SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_IOPDEN);
      SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
      MODIFY_REG(GPIOD->PUPDR, (GPIO_PUPDR_PUPD5|GPIO_PUPDR_PUPD6), (GPIO_PUPDR_PUPD5_0|GPIO_PUPDR_PUPD6_0));//01: 上拉
      CLEAR_BIT(GPIOD->MODER, (GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0)); //10: 复用功能模式
      SET_BIT(GPIOD->AFR[0], (GPIO_AFRL_AFSEL5_1 | GPIO_AFRL_AFSEL6_1));  //0010:AF2 USART2_TX(PD5) USART2_RX(PD6)
      break;
    
    case (uint32_t)USART3:
      SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_IOPBEN);
      SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);
      MODIFY_REG(GPIOB->PUPDR, (GPIO_PUPDR_PUPD10|GPIO_PUPDR_PUPD11), (GPIO_PUPDR_PUPD10_0|GPIO_PUPDR_PUPD11_0));//01: 上拉
      CLEAR_BIT(GPIOB->MODER, (GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0)); //10: 复用功能模式
      SET_BIT(GPIOB->AFR[1], (GPIO_AFRH_AFSEL10_1 | GPIO_AFRH_AFSEL11_1));  //0010:AF2 USART3_TX(PB10) USART3_RX(PB11)
      break;
    
    case (uint32_t)USART4:
      SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_IOPCEN);
      SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART4EN);
      MODIFY_REG(GPIOC->PUPDR, (GPIO_PUPDR_PUPD10|GPIO_PUPDR_PUPD11), (GPIO_PUPDR_PUPD10_0|GPIO_PUPDR_PUPD11_0));//01: 上拉
      CLEAR_BIT(GPIOC->MODER, (GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0)); //10: 复用功能模式
      SET_BIT(GPIOC->AFR[1], (GPIO_AFRH_AFSEL10_0 | GPIO_AFRH_AFSEL11_0));  //0001:AF1 USART4_TX(PC10) USART4_RX(PC11)
      break;
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
  MODIFY_REG(Instance->CR3, USART_CR3_ABRMOD, USART_CR3_ABREN);

  SET_BIT(Instance->CR1, (USART_CR1_TE | USART_CR1_RE)); //1： 传送使能；1： 接收使能
}

/**
  * @brief  This function is used to send data through USART pipe.
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
  * @brief  This function is used to read data from USART pipe.
  * @param  data The data to be read.
  * @param  size The data size to be read.
  * @retval None.
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
