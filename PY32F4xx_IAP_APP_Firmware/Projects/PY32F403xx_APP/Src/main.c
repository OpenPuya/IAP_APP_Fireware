/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ConfigureExti(void);

/**
  * @brief  应用程序入口函数.
  * @retval int
  */
int main(void)
{
  /* 初始化所有外设，Flash接口，SysTick */
  HAL_Init();
  
  /* 系统时钟配置 */
  APP_SystemClockConfig();
  
  /* 初始化LED GPIO */
  BSP_LED_Init(LED_GREEN);
  
  for (uint32_t i=0; i<20; i++)
  {
    BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(100);
  }
  
  /* 配置外部中断 */
  APP_ConfigureExti();

  while (1)
  {

  }
}

/**
  * @brief  系统时钟配置函数
  * @param  无
  * @retval 无
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef  OscInitstruct = {0};
  RCC_ClkInitTypeDef  ClkInitstruct = {0};
  
  OscInitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE | 
                                  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSI48M;
  OscInitstruct.HSEState        = RCC_HSE_OFF;                              /* 关闭HSE */
/* OscInitstruct.HSEFreq         = RCC_HSE_16_32MHz; */                     /* 选择HSE频率16~32M */
  OscInitstruct.HSI48MState     = RCC_HSI48M_OFF;                           /* 关闭HSI48M */
  OscInitstruct.HSIState        = RCC_HSI_ON;                               /* 开启HSI */
  OscInitstruct.LSEState        = RCC_LSE_OFF;                              /* 关闭LSE */
/* OscInitstruct.LSEDriver       = RCC_LSEDRIVE_HIGH; */                    /* 驱动能力等级：高 */
  OscInitstruct.LSIState        = RCC_LSI_OFF;                              /* 关闭LSI */
  OscInitstruct.PLL.PLLState    = RCC_PLL_OFF;                              /* 关闭PLL */
/* OscInitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE; */                    /* PLL时钟源选择HSE */
/* OscInitstruct.PLL.PLLMUL      = RCC_PLL_MUL6; */                         /* PLL时钟源6倍频 */
  /* 配置振荡器 */
  if(HAL_RCC_OscConfig(&OscInitstruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  ClkInitstruct.ClockType       = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  ClkInitstruct.SYSCLKSource    = RCC_SYSCLKSOURCE_HSI;                 /* 系统时钟选择HSI */
  ClkInitstruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;                      /* AHB时钟1分频 */
  ClkInitstruct.APB1CLKDivider  = RCC_HCLK_DIV1;                        /* APB1时钟1分频 */
  ClkInitstruct.APB2CLKDivider  = RCC_HCLK_DIV2;                        /* APB2时钟2分频 */
  /* 配置时钟 */
  if(HAL_RCC_ClockConfig(&ClkInitstruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  配置EXTI
  * @param  无
  * @retval 无
  */
static void APP_ConfigureExti(void)
{
  /* 配置引脚 */
  GPIO_InitTypeDef  GPIO_InitStruct;
  USER_BUTTON_GPIO_CLK_ENABLE();                 /* 使能按键 GPIO时钟 */
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;  /* GPIO模式为下降沿中断 */
  GPIO_InitStruct.Pull  = GPIO_PULLUP;           /* 上拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  /* 速度为高速 */
  GPIO_InitStruct.Pin = USER_BUTTON_PIN;
  HAL_GPIO_Init(USER_BUTTON_GPIO_PORT, &GPIO_InitStruct);

  /* 使能EXTI中断 */
  HAL_NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);
  /* 配置中断优先级 */
  HAL_NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, 0, 0);
}

/**
  * @brief  错误执行函数
  * @param  无
  * @retval 无
  */
void APP_ErrorHandler(void)
{
  /* 无限循环 */
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  输出产生断言错误的源文件名及行号
  * @param  file：源文件名指针
  * @param  line：发生断言错误的行号
  * @retval 无
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* 用户可以根据需要添加自己的打印信息,
     例如: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* 无限循环 */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
