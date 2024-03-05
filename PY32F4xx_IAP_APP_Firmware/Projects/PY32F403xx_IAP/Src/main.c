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
  APP_WDG_Init();
  
  /* 初始化所有外设，Flash接口，SysTick */
  HAL_Init();
  
  /* 初始化用户按键 */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  /* 检查用户按键是否有按下 */
  if (BSP_PB_GetState(BUTTON_USER) == 0x00)
  {
    APP_Bootloader_Go(APP_ADDR);
  }
  
  /* 系统时钟配置 */
  APP_SystemClockConfig();
  
  /* 初始化USART1外设 */
  APP_USART_Init(USART1);
  
  /* 初始化USART2外设 */
  APP_USART_Init(USART2);
  
  /* 初始化USART3外设 */
  APP_USART_Init(USART3);
  
  /* 初始化USART4外设 */
  APP_USART_Init(USART4);
  
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

  SET_BIT(RCC->CFGR1, RCC_CFGR1_USBSELHSI48);
//  SET_BIT(RCC->CFGR, RCC_CFGR_USBPRE_0);
  
  __HAL_RCC_USB_CLK_ENABLE();

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
  RCC_OscInitTypeDef  OscInitstruct = {0};
  RCC_ClkInitTypeDef  ClkInitstruct = {0};

  OscInitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48M;
  OscInitstruct.HSEState        = RCC_HSE_OFF;                             /* 关闭HSE */
  OscInitstruct.HSEFreq         = RCC_HSE_16_32MHz;                        /* 选择HSE频率16~32M */
  OscInitstruct.HSI48MState     = RCC_HSI48M_ON;                           /* 开启HSI48M */
  OscInitstruct.HSI48MCalibrationValue = RCC_HSI48MCALIBRATION_DEFAULT;    /* HSI48M默认校准值 */
  OscInitstruct.HSIState        = RCC_HSI_ON;                              /* 开启HSI */
  OscInitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;          /* HSI默认校准值 */
  OscInitstruct.PLL.PLLState    = RCC_PLL_ON;                              /* 开启PLL */
  OscInitstruct.PLL.PLLSource   = 0x00000000U;                             /* PLL时钟源选择HSI */
  OscInitstruct.PLL.PLLMUL      = RCC_PLL_MUL6;                            /* PLL时钟源3倍频 */
  /* 配置振荡器 */
  if(HAL_RCC_OscConfig(&OscInitstruct) != HAL_OK)
  {
    NVIC_SystemReset();
  }

  ClkInitstruct.ClockType       = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  ClkInitstruct.SYSCLKSource    = RCC_SYSCLKSOURCE_PLLCLK;              /* 系统时钟选择PLL */
  ClkInitstruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;                      /* AHB时钟1分频 */
  ClkInitstruct.APB1CLKDivider  = RCC_HCLK_DIV1;                        /* APB1时钟1分频 */
  ClkInitstruct.APB2CLKDivider  = RCC_HCLK_DIV1;                        /* APB2时钟1分频 */
  /* 配置时钟 */
  if(HAL_RCC_ClockConfig(&ClkInitstruct, FLASH_LATENCY_1) != HAL_OK)
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
