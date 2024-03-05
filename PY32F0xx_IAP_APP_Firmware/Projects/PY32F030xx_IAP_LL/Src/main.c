/**
  ******************************************************************************
  * @file    main.c
  * @author  Puya Application Team
  * @version V1.0
  * @brief   Main program body
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
#include "bootloader.h"
#include "flash.h"
#include "py32f030xx_ll_Start_Kit.h"

/**
  * @brief  The application entry point.
  * @param  None
  * @retval None
  */
int main(void)
{
#ifdef JUMP_TO_APP_BY_USER_BUTTON
  /* Configure user Button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  /* Check if the USER Button is pressed */
  if (BSP_PB_GetState(BUTTON_USER) == 0x00)
  {
    JumpToAddress(APP_ADDR);
  }
#endif
  
  APP_SystemClockConfig(LL_RCC_HSICALIBRATION_24MHz, 24000000);

  Bootloader_Init();

  /* Infinite loop */
  while (1)
  {
    Bootloader_ProtocolDetection();
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  }
}
#endif
/**
  * @}
  */


/************************ (C) COPYRIGHT Puya Semiconductor *****END OF FILE****/
