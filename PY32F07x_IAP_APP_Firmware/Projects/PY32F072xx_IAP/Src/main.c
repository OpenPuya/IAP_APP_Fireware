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
#include "usb_config.h"
#include "app_bootloader.h"
#include "app_wdg.h"
#include "app_usart.h"
#include "app_i2c.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_USBInit(void);

/**
  * @brief  应用程序入口函数.
  * @retval int
  */
int main(void)
{
  /* 初始化IWDG和WWDG */
  APP_WDG_Init();
  
  /* 初始化所有外设，Flash接口，SysTick */
  HAL_Init();  
  
  /* Configure user Button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  /* Check if the USER Button is pressed */
  if (BSP_PB_GetState(BUTTON_USER) == 0x00)
  {
    APP_Bootloader_Go(APP_ADDR);
  }
  
  /* 系统时钟配置 */
  APP_SystemClockConfig();

  /* 初始化USART1外设 */
  APP_USART_InitRx(USART1);
  
  /* 初始化USART2外设 */
  APP_USART_InitRx(USART2);
  
  /* 初始化USART3外设 */
  APP_USART_InitRx(USART3);
  
  /* 初始化USART4外设 */
  APP_USART_InitRx(USART4);
  
  /* 初始化I2C外设 */
  APP_I2C_Init();

  /* 初始化USB外设 */
  APP_USBInit();
  
  APP_Bootloader_Init();

  /* 无限循环 */
  while (1)
  {
    APP_Bootloader_ProtocolDetection();
  }
}

/**
  * @brief  USB外设初始化函数
  * @param  无
  * @retval 无
  */
static void APP_USBInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  __HAL_RCC_USB_CLK_ENABLE();
  
  __HAL_RCC_GPIOA_CLK_ENABLE();

  hid_custom_dfu_init();

  /* 使能USB中断 */
  NVIC_EnableIRQ(USBD_IRQn);
}

/**
  * @brief  系统时钟配置函数
  * @param  无
  * @retval 无
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 振荡器配置 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;         /* 选择振荡器HSI */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                           /* 开启HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                           /* HSI 1分频 */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;  /* 配置HSI时钟24MHz */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                       /* 开启PLL */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;               /* PLL输入源为HSI */
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;                       /* PLL 2倍频输出 */
  /* 配置振荡器 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    NVIC_SystemReset();
  }

  /* 时钟源配置 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* 选择配置时钟 HCLK,SYSCLK,PCLK1 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; /* 选择PLL作为系统时钟 */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        /* AHB时钟 1分频 */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         /* APB时钟 1分频 */
  /* 配置时钟源 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    NVIC_SystemReset();
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
