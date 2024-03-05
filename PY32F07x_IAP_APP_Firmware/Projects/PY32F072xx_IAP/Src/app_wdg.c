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
  /* 检查硬件WWDG是否打开 */
  if (FLASH_OPTR_WWDG_SW != READ_BIT(FLASH->OPTR, FLASH_OPTR_WWDG_SW))
  {
    /* 将WWDG预分频器的时基设置为最大(11：CK 计数器时钟（PCLK 除以 4096）除以8) */
    SET_BIT(WWDG->CFR, WWDG_CFR_WDGTB);
  }
  
  /* 检查硬件IWDG是否打开 */
  if (FLASH_OPTR_IWDG_SW != READ_BIT(FLASH->OPTR, FLASH_OPTR_IWDG_SW))
  {
    /* 将预分频器设置为最大值, 写 IWDG_PR 寄存器，配置预分频值 111 */
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
  /* 7 位计数器（MSB 至 LSB）。该寄存器用来存储看门狗的计数值。
  每（4096x2WDGTB）个 PCLK 周期减 1。当计数值从40h 变为 3Fh 时（T[6]变为 0），产生看门狗复位。*/
  SET_BIT(WWDG->CR, WWDG_CR_T);

  /* 软件必须以一定的时间间隔向该寄存器写入 0xAAAA，否则，当计数器计数到 0 时，看门狗会产生复位。*/
  WRITE_REG(IWDG->KR, 0x0000AAAAU);
}
