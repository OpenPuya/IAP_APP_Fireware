/**
  ******************************************************************************
  * @file    app_i2c.h
  * @author  MCU Application Team
  * @brief   Header for app_i2c.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Puya Semiconductor.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_I2C_H
#define __APP_I2C_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void APP_I2C_Init(void);
void APP_I2C_SendData(uint8_t *data, uint16_t size);
void APP_I2C_ReadData(uint8_t *data, uint16_t size);
uint8_t APP_I2C_ShakeHandCheck(void);

#endif /* __APP_I2C_H */

/************************ (C) COPYRIGHT Puya Semiconductor *****END OF FILE****/
