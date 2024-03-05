/**
  ******************************************************************************
  * @file    app_i2c.c
  * @author  MCU Application Team
  * @brief   Contains I2C HW configuration
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
#include "app_i2c.h"
#include "app_wdg.h"

/* Private define ------------------------------------------------------------*/
#define I2Cx            I2C1
#define I2Cx_SLAVE_ADDR 0x7E
/* Private variables ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void APP_I2C_Init(void)
{
  SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_IOPBEN);
  CLEAR_BIT(GPIOB->MODER, (GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0)); //10: 复用功能模式
  SET_BIT(GPIOB->AFR[0], (GPIO_AFRL_AFSEL6_0 | GPIO_AFRL_AFSEL7_0));  //0001:AF1 I2C_SCL(PB6) I2C_SDA(PB7)
  SET_BIT(GPIOB->OTYPER, (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7));//1: 开漏输出
  MODIFY_REG(GPIOB->PUPDR, (GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7), (GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0));
  
  /* 启用 I2C1 的外设时钟 */
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
  
  /* 配置从机地址：*/
  MODIFY_REG(I2Cx->OAR1, I2C_OAR1_ADD1_7, I2Cx_SLAVE_ADDR);
 
  /* 使能 I2C1 */
  SET_BIT(I2Cx->CR1, I2C_CR1_PE);
}

/**
  * @brief  This function is used to send data through I2C pipe.
  * @param  data The data to be sent.
  * @param  size The data size to be sent.
  * @retval None.
  */
void APP_I2C_SendData(uint8_t *data, uint16_t size)
{
  __IO uint32_t tmpreg = 0x00U;
  
  /* Disable Pos */
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_POS);
  
  /* Enable Address Acknowledge */
  SET_BIT(I2Cx->CR1, I2C_CR1_ACK);

  /* Wait until ADDR flag is set */
  while(READ_BIT(I2Cx->SR1, I2C_SR1_ADDR) != I2C_SR1_ADDR)
  {
    APP_WDG_Refresh();
  }
  
  /* Clear ADDR flag */
  tmpreg = I2Cx->SR1;
  tmpreg = I2Cx->SR2;

  while (size)
  {
    /* Wait until TXE flag is set */
    while (READ_BIT(I2Cx->SR1, I2C_SR1_TXE) != (I2C_SR1_TXE))
    {
      APP_WDG_Refresh();
    }
    
    /* Write data to DR */
    I2Cx->DR = *data++;
    size--;

    if(READ_BIT(I2Cx->SR1, I2C_SR1_BTF) == (I2C_SR1_BTF))
    {
      /* Write data to DR */
      I2Cx->DR = *data++;
      size--;
    }
  }
  
  /* Wait until AF flag is set */ 
  while (READ_BIT(I2Cx->SR1, I2C_SR1_AF) != I2C_SR1_AF)
  {
    APP_WDG_Refresh();
  }

  /* Clear AF flag */
  CLEAR_BIT(I2Cx->SR1, I2C_SR1_AF);
  
  /* Disable Address Acknowledge */
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_ACK);
}

/**
  * @brief  This function is used to read data from I2C pipe.
  * @param  data The data to be read.
  * @param  size The data size to be read.
  * @retval None.
  */
void APP_I2C_ReadData(uint8_t *data, uint16_t size)
{
  __IO uint32_t tmpreg = 0x00U;
  
  /* Disable Pos */
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_POS);
  
  /* Enable Address Acknowledge */
  SET_BIT(I2Cx->CR1, I2C_CR1_ACK);

  /* Wait until ADDR flag is set */
  while (READ_BIT(I2Cx->SR1, I2C_SR1_ADDR) != I2C_SR1_ADDR)
  {
    APP_WDG_Refresh();
  }
  
  /* Clear ADDR flag */
  tmpreg = I2Cx->SR1;
  tmpreg = I2Cx->SR2;

  while (size)
  {
    /* Wait until RXNE flag is set */
    while (READ_BIT(I2Cx->SR1, I2C_SR1_RXNE) != (I2C_SR1_RXNE))
    {
      APP_WDG_Refresh();
    }
    
    /* Read data from DR */
    *data++ = (uint8_t)I2Cx->DR;
    size--;

    if(READ_BIT(I2Cx->SR1, I2C_SR1_BTF) == (I2C_SR1_BTF))
    {
      /* Read data from DR */
      *data++ = (uint8_t)I2Cx->DR;
      size--;
    }
  }
  
  /* Wait until STOP flag is set */
  while (READ_BIT(I2Cx->SR1, I2C_SR1_STOPF) != I2C_SR1_STOPF)
  {
    APP_WDG_Refresh();
  }

  /* Clear STOP flag */
  tmpreg = I2Cx->SR1;
  SET_BIT(I2Cx->CR1, I2C_CR1_PE);
  
  /* Disable Address Acknowledge */
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_ACK);
}

uint8_t APP_I2C_ShakeHandCheck(void)
{
  __IO uint32_t tmpreg = 0x00U;
  
  /* Disable Pos */
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_POS);
  
  /* Enable Address Acknowledge */
  SET_BIT(I2Cx->CR1, I2C_CR1_ACK);

  /* Wait until ADDR flag is set */
  if (READ_BIT(I2Cx->SR1, I2C_SR1_ADDR) != I2C_SR1_ADDR)
  {
    return ERROR;
  }
  
  /* Clear ADDR flag */
  tmpreg = I2Cx->SR1;
  tmpreg = I2Cx->SR2;
  
  /* Wait until RXNE flag is set */
  while (READ_BIT(I2Cx->SR1, I2C_SR1_RXNE) != (I2C_SR1_RXNE))
  {
    APP_WDG_Refresh();
  }
  
  /* Read data from DR */
  if (0x7F != (uint8_t)I2Cx->DR)
  {
    return ERROR;
  }

  if(READ_BIT(I2Cx->SR1, I2C_SR1_BTF) == (I2C_SR1_BTF))
  {
    /* Read data from DR */
    tmpreg = (uint8_t)I2Cx->DR;
  }
  
  /* Wait until STOP flag is set */
  while (READ_BIT(I2Cx->SR1, I2C_SR1_STOPF) != I2C_SR1_STOPF)
  {
    /* Read data from DR */
    tmpreg = (uint8_t)I2Cx->DR;
    APP_WDG_Refresh();
  }

  /* Clear STOP flag */
  tmpreg = I2Cx->SR1;
  SET_BIT(I2Cx->CR1, I2C_CR1_PE);
  
  /* Disable Address Acknowledge */
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_ACK);
  
  return SUCCESS;
}
