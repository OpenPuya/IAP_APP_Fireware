/**
  ******************************************************************************
  * @file    main.c
  * @author  Puya Application Team
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

/**
  * @brief  The application entry point.
  * @param  None
  * @retval None
  */
int main(void)
{  
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
