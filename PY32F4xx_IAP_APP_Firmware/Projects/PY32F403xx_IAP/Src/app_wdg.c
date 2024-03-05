/**
  ******************************************************************************
  * @file    app_wdg.c
  * @author  MCU Application Team
  * @brief   Contains WDG HW configuration
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
#include "app_wdg.h"

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize WWDG and IWDG.
  * @param  None.
  * @retval None.
  */
void APP_WDG_Init(void)
{
  if (FLASH_OPTR_IWDG_SW != READ_BIT(FLASH->OPTR, FLASH_OPTR_IWDG_SW))
  {
    //将预分频器设置为最大值
    //写 IWDG_PR 寄存器，配置预分频值 111
    WRITE_REG(IWDG->KR, 0x00005555U);
    WRITE_REG(IWDG->PR, IWDG_PR_PR);
    WRITE_REG(IWDG->RLR, 0x00000FFFU);
  }
  
  APP_WDG_Refresh();
}

/**
  * @brief  Refresh WWDG and IWDG.
  * @param  None.
  * @retval None.
  */
void APP_WDG_Refresh(void)
{
  //软件必须以一定的时间间隔向该寄存器写入 0xAAAA，否则，当计数器计数到 0 时，看门狗会产生复位。
  WRITE_REG(IWDG->KR, 0x0000AAAAU);
}
