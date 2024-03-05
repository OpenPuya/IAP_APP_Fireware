/**
  ******************************************************************************
  * @file    py32f4xx_ll_sdio.c
  * @author  MCU Application Team
  * @brief   SDIO HAL module driver.
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
#include "py32f4xx_hal.h"

/** @addtogroup PY32F4XX_HAL_Driver
  * @{
  */

/** @defgroup SDIO SDIO
* @brief SDIO LL module driver
  * @{
  */
  
#ifdef HAL_SD_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  DeInitialize the SDIO peripheral.
  * @retval None
  */
void SIDO_DeInit(void)
{
  __HAL_RCC_SDIO_FORCE_RESET();
  __HAL_RCC_SDIO_RELEASE_RESET();
}

/**
  * @brief  Initializes the SDIO peripheral according to the specified
  *         parameters in the SDIO_InitStruct.
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  SDIO_InitStruct : pointer to a SDIO_InitTypeDef structure
  *         that contains the configuration information for the SDIO peripheral.
  * @retval None
  */
void SDIO_Init(SDIO_TypeDef *SDIOx, SDIO_InitTypeDef* SDIO_InitStruct)
{
  uint32_t tmpreg;
  /* Check the parameters */
  assert_param(IS_SDIO_CLOCK_POWER_SAVE(SDIO_InitStruct->ClockPowerSave));
  assert_param(IS_SDIO_BUS_WIDE(SDIO_InitStruct->BusWide));
  assert_param(IS_SDIO_SMP_CLOCK(SDIO_InitStruct->SMPClock));
  assert_param(IS_SDIO_SMP_STATE(SDIO_InitStruct->SMPState));
  assert_param(IS_SDIO_CLOCK_Output(SDIO_InitStruct->CLockOutput));

  /*---------------------------- SDIO Card Width Configuration ---------------*/
  tmpreg = SDIOx->CLKCR;
  tmpreg &= ~(SDIO_CLKCR_WIDBUS_Msk);
  tmpreg |= SDIO_InitStruct->BusWide;
  SDIOx->CLKCR = tmpreg;
  
  /*---------------------------- SDIO CLK PWR SAVE Configuration -------------*/
  tmpreg = SDIOx->CLKCR;
  tmpreg &= ~(SDIO_CLKCR_PWRSAV_Msk);
  tmpreg |= SDIO_InitStruct->ClockPowerSave;
  SDIOx->CLKCR = tmpreg;
  
  /*---------------------------- SDIO CLK SMP Configuration ------------------*/
  tmpreg = SDIOx->CLKCR;
  tmpreg &= ~(SDIO_CLKCR_SMPCLKSEL_Msk | SDIO_CLKCR_SMPEN);
  tmpreg |= ((SDIO_InitStruct->SMPClock)| (SDIO_InitStruct->SMPState));
  SDIOx->CLKCR = tmpreg;

  /*---------------------------- SDIO CLK OUTPUT Configuration ---------------*/
  tmpreg = SDIOx->CLKCR;
  tmpreg &= ~(SDIO_CLKCR_CKSEL_Msk);
  tmpreg |= SDIO_InitStruct->CLockOutput;
  SDIOx->CLKCR = tmpreg;
  
  /* Set Clock Divider and Enable */
  SDIO_SetClock(SDIOx,SDIO_InitStruct->ClockDiv);
}

/**
  * @brief  Fills each SDIO_InitStruct member with its default value.
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  SDIO_InitStruct: pointer to an SDIO_InitTypeDef structure which
  *         will be initialized.
  * @retval None
  */
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct)
{
  /* SDIO_InitStruct members default value */
  SDIO_InitStruct->ClockDiv       = 0x00;
  SDIO_InitStruct->ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  SDIO_InitStruct->BusWide        = SDIO_BUS_WIDE_1B;
  SDIO_InitStruct->SMPClock       = SDIO_SMP_CLOCK_SDCLK_DEFAULT;
  SDIO_InitStruct->SMPState       = SDIO_SMP_ENABLE;
  SDIO_InitStruct->CLockOutput    = SDIO_CLOCK_OUTPUT_OFFSET180;
}

/**
  * @brief  The SDIO loads each of these registers only when the start_cmd bit and the
  *         Update_clk_regs_only bit in the CMD register are set. When a command is successfully loaded, the
  *         SDIO Controller clears this bit, unless the SDIO Controller already has another command in the
  *         queue, at which point it gives an HLE (Hardware Locked Error);
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  SDIO_InitStruct: pointer to an SDIO_InitTypeDef structure which
  *         will be initialized.
  *         SDIO_ClkDiv: clock dividor
  * @retval None
  */
void SDIO_SetClock(SDIO_TypeDef *SDIOx,uint32_t SDIO_ClkDiv)
{
  uint32_t tmpreg;
  tmpreg = SDIOx->CLKCR;
  /* 1. Confirm that no card is engaged in any transaction; if there is a transaction, wait until it finishes. */
  while(SDIO_IsCardBusy(SDIOx));
  /* 2. Stop all clocks by writing xxxx0000 to the CLKENA register. Set the start_cmd,
  *   Update_clk_regs_only, and wait_prvdata_complete bits in the CMD register. Wait until start_cmd is
  *   cleared or an HLE is set; in case of an HLE, repeat the command.
  */
  
  SDIO_ClockCmd(SDIOx,DISABLE);
  /* 3. Program the CLKDIV and CLKSRC registers, as required. Set the start_cmd, Update_clk_regs_only,
  *    and wait_prvdata_complete bits in the CMD register. Wait until start_cmd is cleared or an HLE is
  *    set; in case of an HLE, repeat the command.
  */
  tmpreg &= ~(SDIO_CLKCR_CLKDIV_Msk);
  tmpreg |= (uint32_t)(SDIO_ClkDiv & SDIO_CLKCR_CLKDIV_Msk);
  
  /*4. Re-enable all clocks by programming the CLKENA register. Set the start_cmd,
  *   Update_clk_regs_only, and wait_prvdata_complete bits in the CMD register. Wait until start_cmd is
  *   cleared or an HLE is set; in case of an HLE, repeat the command.
  */
  SDIOx->CLKCR = tmpreg;
  SDIO_ClockCmd(SDIOx,ENABLE);
}

/**
  * @brief  Enables or disables the SDIO Clock.
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  NewState: new state of the SDIO Clock.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SDIO_ClockCmd(SDIO_TypeDef *SDIOx,FunctionalState NewState)
{
    uint32_t tmpreg;

    /* Check the parameters */
    assert_param(IS_FUNCTIONAL_STATE(NewState));
    tmpreg = SDIOx->CLKCR;
    
    if(!NewState)
    {
        tmpreg &= ~(SDIO_CLKCR_CLKEN_Msk);
    }
    else
    {
        tmpreg |= (SDIO_CLKCR_CLKEN_Msk);
    }
    SDIOx->CLKCR = tmpreg;


    /* Wait until start_cmd is cleared or an HLE is set; in case of an HLE, repeat the command */
    SDIOx->CMD |= (SDIO_CMD_STARTCMD_Msk | SDIO_CMD_REGSYNC_Msk |SDIO_CMD_WAITPEND_Msk);    
    while(SDIOx->CMD & SDIO_CMD_STARTCMD_Msk);

}

/**
  * @brief  If CTYPE[16] = 1, the card at port0 is in 8-bit mode. Note that the CTYPE[0] value is ignored; it is
  *         recommended to keep this set to 0.
  *         If CTYPE[16] = 0, the card at port0 is in either 1-bit or 4-bit mode, depending upon the value of
  *         CTYPE[0]; that is, if CTYPE[0] = 1 - 4-bit, CTYPE[0] = 0 - 1-bit.
  * @param  SDIOx:Pointer to SDIO register base.
  * @param  SDIO_Width: This parameter can be one of the following values:
  *                     SDIO_BUS_WIDE_1B
  *                     SDIO_BUS_WIDE_4B
  *                     SDIO_BUS_WIDE_8B
* @retval None
  */
void SDIO_SetCardBusWidth(SDIO_TypeDef *SDIOx,uint32_t SDIO_Width)
{
    uint32_t tmpreg;
    /* Check the parameters */
    assert_param(IS_SDIO_BUS_WIDE(WIDE));

    /*---------------------------- SDIO CLKTYPE Configuration ------------------------*/
    tmpreg = SDIOx->CLKCR;
    tmpreg &= ~(SDIO_CLKCR_WIDBUS_Msk);
    tmpreg |= SDIO_Width;

    SDIOx->CLKCR = tmpreg;
}

/**
  * @brief  Sets the power status of the controller.
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  SDIO_PowerState: new state of the Power state.
  *          This parameter can be one of the following values:
  *            @arg SDIO_POWER_STATE_OFF: SDIO Power OFF
  *            @arg SDIO_POWER_STATE_ON : SDIO Power ON
  * @retval None
  */
void SDIO_SetPowerState(SDIO_TypeDef *SDIOx,uint32_t SDIO_PowerState)
{
  /* Check the parameters */
  assert_param(IS_SDIO_POWER_STATE(SDIO_PowerState));

  SDIOx->POWER = SDIO_PowerState;
}

/**
  * @brief  Gets the power status of the controller.
  * @param  SDIOx: Pointer to SDIO register base.
  * @retval Power status of the controller. The returned value can be one of the
  *         following values:
  *            - 0x00: Power OFF
  *            - 0x01: Power ON
  */
uint32_t SDIO_GetPowerState(SDIO_TypeDef *SDIOx)
{
    uint32_t tmpreg = 0;
    tmpreg = SDIOx->POWER;
        
    tmpreg &= SDIO_POWER_PWRCTRL_Msk;
    

  return (uint32_t)(tmpreg >> SDIO_POWER_PWRCTRL_Pos);
}

/**
  * @brief  Send stop command at end of data transfer or Not
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  NewState :   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SDIO_IsAutoStopCmd(SDIO_TypeDef *SDIOx,FunctionalState NewState)
{
  uint32_t tmpreg = 0;
  tmpreg = SDIOx->CMD;
      
  tmpreg &= ~SDIO_CMD_AUTOSTOP_Msk;
  
  if(NewState)
  {
    tmpreg |= SDIO_CMD_AUTOSTOP;
  }
  
  SDIOx->CMD = tmpreg;
}

/**
  * @brief  Initializes the SDIO Command according to the specified 
  * @param  SDIOx: Pointer to SDIO register base.
  *         parameters in the SDIO_CmdInitStruct and send the command.
  * @param  SDIO_CmdInitStruct : pointer to a SDIO_CmdInitTypeDef 
  *         structure that contains the configuration information for the SDIO 
  *         command.
  * @retval None
  */
void SDIO_SendCommand(SDIO_TypeDef *SDIOx,SDIO_CmdInitTypeDef *SDIO_CmdInitStruct)
{
  uint32_t tmpreg = 0;

  /* Set the SDIO Argument value */
  SDIOx->ARG = SDIO_CmdInitStruct->Argument;
  /* Get the SDIO CMD value */
  tmpreg = SDIOx->CMD;
    
  /* data_expected 0 0-No data transfer expected (read/write) 1-Data transfer expected (read/write) clear reg sync */
  tmpreg &= ~SDIO_CMD_REGSYNC_Msk;
  tmpreg &= ~(SDIO_CMD_STARTCMD_Msk| \
              SDIO_CMD_DEXPECT_Msk | \
              SDIO_CMD_AUTOINIT_Msk| \
              SDIO_CMD_WAITPEND_Msk| \
              SDIO_CMD_WAITRESP_Msk| \
              SDIO_CMD_RESPLEN_Msk | \
              SDIO_CMD_CHECKRESPCRC_Msk);
  
  tmpreg &= ~SDIO_CMD_CMDINDEX_Msk;

  tmpreg |= (uint32_t)(SDIO_CmdInitStruct->Response     |
                       SDIO_CmdInitStruct->Response_CRC |
                       SDIO_CmdInitStruct->DataTransfer |
                       SDIO_CmdInitStruct->Wait         |
                       SDIO_CmdInitStruct->InitSignal   |
                       SDIO_CmdInitStruct->CmdIndex);

  SDIOx->CMD = tmpreg;

  /* bit 31 start_cmd should write after all other CMD register bits have been wrote */
  tmpreg |=  SDIO_CmdInitStruct->CPSM;
  SDIOx->CMD = tmpreg;
}

/**
  * @brief  Fills each SDIO_CmdInitStruct member with its default value.
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  SDIO_CmdInitStruct: pointer to an SDIO_CmdInitTypeDef 
  *         structure which will be initialized.
  * @retval None
  */
void SDIO_CmdStructInit(SDIO_CmdInitTypeDef* SDIO_CmdInitStruct)
{
  /* SDIO_CmdInitStruct members default value */
  SDIO_CmdInitStruct->Argument = 0x00;
  SDIO_CmdInitStruct->CmdIndex = 0x00;
  SDIO_CmdInitStruct->Response = SDIO_RESPONSE_NO;
  SDIO_CmdInitStruct->Wait     = SDIO_WAIT_NO;
  SDIO_CmdInitStruct->CPSM     = SDIO_CPSM_DISABLE;
}

/**
  * @brief  Returns response received from the card for the last command.
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  SDIO_RESP: Specifies the SDIO response register. 
  *          This parameter can be one of the following values:
  *            @arg SDIO_RESP1: Response Register 1
  *            @arg SDIO_RESP2: Response Register 2
  *            @arg SDIO_RESP3: Response Register 3
  *            @arg SDIO_RESP4: Response Register 4
  * @retval The Corresponding response register value.
  */
uint32_t SDIO_GetResponse(SDIO_TypeDef *SDIOx,uint32_t SDIO_RESP)
{
    __IO uint32_t tmp = 0;

    /* Check the parameters */
    assert_param(IS_SDIO_RESP(SDIO_RESP));

    tmp = (uint32_t)(&(SDIOx->RESP0)) + SDIO_RESP;

    return (*(__IO uint32_t *) tmp);
}
/**
  * @brief  Initializes the SDIO data path according to the specified 
  *         parameters in the SDIO_DataInitStruct.
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  SDIO_DataInitStruct : pointer to a SDIO_DataInitTypeDef structure 
  *         that contains the configuration information for the SDIO command.
  * @retval None
  */
void SDIO_DataConfig(SDIO_TypeDef *SDIOx,SDIO_DataInitTypeDef* SDIO_DataInitStruct)
{
    uint32_t tmpreg = 0;

    /* Set the SDIO Data TimeOut value */
    SDIOx->TMOUT = SDIO_DataInitStruct->DataTimeOut << 8 | 0x64;

    /* Set the SDIO DataLength value */
    SDIOx->DLEN = SDIO_DataInitStruct->DataLength;

    /* Set the SDIO BlockSize value */
    SDIOx->BLKSIZ = SDIO_DataInitStruct->DataBlockSize;

    tmpreg = SDIOx->CMD;
    tmpreg &= ~(SDIO_CMD_DTMODE_Msk | SDIO_CMD_DIR_Msk);

    tmpreg |= SDIO_DataInitStruct->TransferDir | SDIO_DataInitStruct->TransferMode;

    SDIOx->CMD = tmpreg;
}
/**
  * @breif FIFO threshold watermark level when transmitting data to card.
  *        When FIFO data count is less than or equal to this number,
  *        DMA/FIFO request is raised. If Interrupt is enabled, then interrupt
  *        occurs. During end of packet, request or interrupt is generated,
  *        regardless of threshold programming.
  *        In non-DMA mode, when transmit FIFO threshold (TXDR)
  *        interrupt is enabled, then interrupt is generated instead of DMA
  *        request. During end of packet, on last interrupt, host is
  *        responsible for filling FIFO with only required remaining bytes (not
  *        before FIFO is full or after CIU completes data transfers, because
  *        FIFO may not be empty).
  *        In DMA mode, at end of packet, if last transfer is less than burst
  *        size, DMA controller does single cycles until required bytes are
  *        transferred.
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  SDIO_WaterMark: FIFO threshold watermark level when receiving data to card.
            This parameter can be one or a combination of the following values:
  *         SDIO_FIFO_WMARK_1
  *         SDIO_FIFO_WMARK_2
  *         ...

  *         SDIO_FIFO_WMARK_16
  * @retval None

  **/
void SDIO_SetTxWaterMark(SDIO_TypeDef *SDIOx,uint16_t SDIO_WaterMark)
{
    uint32_t tmpreg = 0;

    tmpreg = SDIOx->FIFOTH;

    tmpreg &= ~(SDIO_FIFOTH_TXWMARK_Msk);
    tmpreg |= ((SDIO_WaterMark << SDIO_FIFOTH_TXWMARK_Pos) & SDIO_FIFOTH_TXWMARK_Msk);
    
    SDIOx->FIFOTH = tmpreg;
}

/**
  * @breif FIFO threshold watermark level when receiving data to card.
  *        When FIFO data count reaches greater than this number,
  *        DMA/FIFO request is raised. During end of packet, request is
  *        generated regardless of threshold programming in order to
  *        complete any remaining data.
  *        In non-DMA mode, when receiver FIFO threshold (RXDR)
  *        interrupt is enabled, then interrupt is generated instead of DMA request.
  *        During end of packet, interrupt is not generated if threshold
  *        programming is larger than any remaining data. It is responsibility
  *        of host to read remaining bytes on seeing Data Transfer Done interrupt.
  *        In DMA mode, at end of packet, even if remaining bytes are less
  *        than threshold, DMA request does single transfers to flush out any
  *        remaining bytes before Data Transfer Done interrupt is set.
  * @param  SDIOx:Pointer to SDIO register base.
  * @param  SDIO_WaterMark: FIFO threshold watermark level when receiving data to card.
            This parameter can be one or a combination of the following values:
  *         SDIO_FIFO_WMARK_1
  *         SDIO_FIFO_WMARK_2
  *         ...

  *         SDIO_FIFO_WMARK_16
  * @retval None
  *
  **/
void SDIO_SetRxWaterMark(SDIO_TypeDef *SDIOx,uint16_t SDIO_WaterMark)
{
    uint32_t tmpreg = 0;

    tmpreg = SDIOx->FIFOTH;

    tmpreg &= ~(SDIO_FIFOTH_RXWMARK_Msk);
    tmpreg |= ((SDIO_WaterMark << SDIO_FIFOTH_RXWMARK_Pos) & SDIO_FIFOTH_RXWMARK_Msk);

    SDIOx->FIFOTH = tmpreg;
}
/**
  * @brief  Returns the number of words left to be written to or read from FIFO.
  * @param  SDIOx: Pointer to SDIO register base.
  * @retval Remaining number of words.
  */
uint16_t SDIO_GetFifoCount(SDIO_TypeDef *SDIOx)
{
  uint16_t fifo_cnt = 0;
  uint32_t tmpreg = 0;

  tmpreg = SDIOx->STATUS;

  tmpreg &= SDIO_STATUS_FIFOCNT_Msk;
  fifo_cnt = (uint16_t)(tmpreg >> SDIO_STATUS_FIFOCNT_Pos);

  return (fifo_cnt);
}

/**
  * @brief  Returns Command FSM states
  * @param  SDIOx: Pointer to SDIO register base.
  * @retval Command FSM states
  */
uint16_t SDIO_GetCmdState(SDIO_TypeDef *SDIOx)
{
  uint16_t cmdState = 0;
  uint32_t tmpreg = 0;

  tmpreg = SDIOx->STATUS;
  tmpreg &= SDIO_STATUS_CMDFSM_Msk;

  cmdState = (uint16_t)(tmpreg >> SDIO_STATUS_CMDFSM_Pos);

  return (cmdState);
}

/**
  *@breif Is card data busy
  **/
uint32_t SDIO_IsCardBusy(SDIO_TypeDef *SDIOx)
{
  uint32_t tmpreg = 0;
    
  tmpreg = SDIOx->STATUS;
  tmpreg &= SDIO_STATUS_CARDBSY_Msk;
    
  return(tmpreg >> SDIO_STATUS_CARDBSY_Pos);
}
/**
  * @brief  Returns FIFO is empty status
  * @param  SDIOx: Pointer to SDIO register base.
  * @retval FIFO is empty or not
  */
uint32_t SDIO_IsFifoEmpty(SDIO_TypeDef *SDIOx)
{
  uint32_t tmpreg = 0;
    
  tmpreg = SDIOx->STATUS;
  tmpreg &= SDIO_STATUS_FIFOE_Msk;
    
  return(tmpreg >> SDIO_STATUS_FIFOE_Pos);

}
/**
  * @brief  Returns FIFO is full status
  * @param  SDIOx: Pointer to SDIO register base.
  * @retval FIFO is full or not
  */
uint32_t SDIO_IsFifoFull(SDIO_TypeDef *SDIOx)
{
  uint32_t tmpreg = 0;
    
  tmpreg = SDIOx->STATUS;
  tmpreg &= SDIO_STATUS_FIFOF_Msk;
    
  return(tmpreg >> SDIO_STATUS_FIFOF_Pos);
}

/**
  * @brief  Returns Index of previous response, including any auto-stop sent by core
  * @param  SDIOx: Pointer to SDIO register base.
  * @retval Index of previous response
  */
uint8_t SDIO_GetRspIdx(SDIO_TypeDef *SDIOx)
{
  uint32_t tmpreg = 0;
    
  tmpreg = SDIOx->RESPCMD;
  tmpreg &= SDIO_RESPCMD_RESPCMD_Msk;
    
  return (uint8_t)(tmpreg >> SDIO_RESPCMD_RESPCMD_Pos);
}

/**
  * @brief  Fills each SDIO_DataInitStruct member with its default value.
  * @param  SDIO_DataInitStruct: pointer to an SDIO_DataInitTypeDef structure 
  *         which will be initialized.
  * @retval None
  */
void SDIO_DataStructInit(SDIO_DataInitTypeDef* SDIO_DataInitStruct)
{
  /* SDIO_DataInitStruct members default value */
  SDIO_DataInitStruct->DataTimeOut   = 0xFFFFFFFF;
  SDIO_DataInitStruct->DataLength    = 0x00;
  SDIO_DataInitStruct->DataBlockSize = SDIO_DATABLOCKSIZE_512B;
  SDIO_DataInitStruct->TransferDir   = SDIO_TRANSFERDIR_TO_CARD;
  SDIO_DataInitStruct->TransferMode  = SDIO_TRANSFERMODE_BLOCK;
}

/**
  * @brief  Read one data word from Rx FIFO.
  * @param  SDIOx: Pointer to SDIO register base.
  * @retval Data received
  */
uint32_t SDIO_ReadData(SDIO_TypeDef * SDIOx)
{
  return (SDIOx->FIFODATA);
}

/**
  * @brief  Write one data word to Tx FIFO.
  * @param  SDIOx:Pointer to SDIO register base.
  * @param  Data: 32-bit data word to write.
  * @retval None
  */
void SDIO_WriteData(SDIO_TypeDef * SDIOx,uint32_t Data)
{
  SDIOx->FIFODATA = Data;
}

/**
  * @brief  Enables or disables the SDIO DMA request.
  * @param  SDIOx:Pointer to SDIO register base.
  * @param  NewState: new state of the selected SDIO DMA request.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SDIO_DMACmd(SDIO_TypeDef * SDIOx,FunctionalState NewState)
{
  uint32_t tmpreg;

  tmpreg = SDIOx->CTRL;

  tmpreg &= ~(SDIO_CTRL_DMAEN_Msk);

  if(NewState)
  {
    tmpreg |= SDIO_CTRL_DMAEN;
  }

  SDIOx->CTRL = tmpreg;
}

/**
  * @brief  Enables or disables the SDIO Global interrupt.
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  NewState: new state of the selected SDIO DMA request.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SDIO_ITCmd(SDIO_TypeDef * SDIOx,FunctionalState NewState)
{
  uint32_t tmpreg;

  tmpreg = SDIOx->CTRL;

  tmpreg &= ~(SDIO_CTRL_INTEN_Msk);

  if(NewState)
  {
    SDIOx->CTRL = tmpreg | SDIO_CTRL_INTEN_Msk;
  }
  
  SDIOx->CTRL = tmpreg;
}
/**
  * @brief  Enable the SDIO's interrupt .
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  SDIO_IT: specifies the interrupt pending bit to clear. 
  *          This parameter can be one or a combination of the following values:
  *            @arg SDIO_IT_CD: Card detect (CD)
  *            @arg SDIO_IT_RE: Response error (RE)
  *            @arg SDIO_IT_CMDD: Command done (CD)
  *            @arg SDIO_IT_DTO:  Data transfer over (DTO)
  *            @arg SDIO_IT_TXDR: Transmit FIFO data request (TXDR)
  *            @arg SDIO_IT_RXDR: Receive FIFO data request (RXDR)
  *            @arg SDIO_IT_RCRC: Response CRC error (RCRC)
  *            @arg SDIO_IT_DCRC: Data CRC error (DCRC)
  *            @arg SDIO_IT_RTO:  Response timeout (RTO)/Boot Ack Received (BAR)
  *            @arg SDIO_IT_DRTO: Data read timeout (DRTO)/Boot Data Start (BDS)
  *            @arg SDIO_IT_HTO:  Data starvation-by-host timeout (HTO) /Volt_switch_int
  *            @arg SDIO_IT_FRUN: FIFO underrun/overrun error (FRUN)
  *            @arg SDIO_IT_HLE:  Hardware locked write error (HLE)
  *            @arg SDIO_IT_SBE:  Start-bit error (SBE)
  *            @arg SDIO_IT_ACD:  Auto command done (ACD)
  *            @arg SDIO_IT_EBE:  End-bit error (read)/write no CRC (EBE)
  *            @arg SDIO_IT_FROM_CARD: Interrupt from SDIO card
  * @retval None
  */
void SDIO_ITConfig(SDIO_TypeDef * SDIOx,uint32_t SDIO_IT, FunctionalState NewState)
{        
  /* Check the parameters */
  assert_param(IS_SDIO_IT(SDIO_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState)
  {
      /* Enable the SDIO interrupts */
      SDIOx->INTMASK |= SDIO_IT;
  }
  else
  {
      /* Disable the SDIO interrupts */
      SDIOx->INTMASK &= ~SDIO_IT;
  }
}

/**
  * @brief  Clears the SDIO's interrupt pending bits.
  * @param  SDIOx: Pointer to SDIO register base.
  * @param  SDIO_IT: specifies the interrupt pending bit to clear. 
  *          This parameter can be one or a combination of the following values:
  *            @arg SDIO_IT_CD: Card detect (CD)
  *            @arg SDIO_IT_RE: Response error (RE)
  *            @arg SDIO_IT_CMDD: Command done (CD)
  *            @arg SDIO_IT_DTO:  Data transfer over (DTO)
  *            @arg SDIO_IT_TXDR: Transmit FIFO data request (TXDR)
  *            @arg SDIO_IT_RXDR: Receive FIFO data request (RXDR)
  *            @arg SDIO_IT_RCRC: Response CRC error (RCRC)
  *            @arg SDIO_IT_DCRC: Data CRC error (DCRC)
  *            @arg SDIO_IT_RTO:  Response timeout (RTO)/Boot Ack Received (BAR)
  *            @arg SDIO_IT_DRTO: Data read timeout (DRTO)/Boot Data Start (BDS)
  *            @arg SDIO_IT_HTO:  Data starvation-by-host timeout (HTO) /Volt_switch_int
  *            @arg SDIO_IT_FRUN: FIFO underrun/overrun error (FRUN)
  *            @arg SDIO_IT_HLE:  Hardware locked write error (HLE)
  *            @arg SDIO_IT_SBE:  Start-bit error (SBE)
  *            @arg SDIO_IT_ACD:  Auto command done (ACD)
  *            @arg SDIO_IT_EBE:  End-bit error (read)/write no CRC (EBE)
  *            @arg SDIO_IT_FROM_CARD: Interrupt from SDIO card
  * @retval None
  */
void SDIO_ClearITStatus(SDIO_TypeDef * SDIOx,uint32_t SDIO_IT)
{
  /* Check the parameters */
  assert_param(IS_SDIO_CLEAR_FLAG(SDIO_FLAG));

  SDIOx->INTSTS |= SDIO_IT;
}
/**
  * @brief  Clears the SDIO's interrupt pending bits.
  * @param  SDIOx:Pointer to SDIO register base.
  * @param  SDIO_IT: specifies the interrupt pending bit to clear. 
  *          This parameter can be one or a combination of the following values:
  *            @arg SDIO_IT_CD: Card detect (CD)
  *            @arg SDIO_IT_RE: Response error (RE)
  *            @arg SDIO_IT_CMDD: Command done (CD)
  *            @arg SDIO_IT_DTO:  Data transfer over (DTO)
  *            @arg SDIO_IT_TXDR: Transmit FIFO data request (TXDR)
  *            @arg SDIO_IT_RXDR: Receive FIFO data request (RXDR)
  *            @arg SDIO_IT_RCRC: Response CRC error (RCRC)
  *            @arg SDIO_IT_DCRC: Data CRC error (DCRC)
  *            @arg SDIO_IT_RTO:  Response timeout (RTO)/Boot Ack Received (BAR)
  *            @arg SDIO_IT_DRTO: Data read timeout (DRTO)/Boot Data Start (BDS)
  *            @arg SDIO_IT_HTO:  Data starvation-by-host timeout (HTO) /Volt_switch_int
  *            @arg SDIO_IT_FRUN: FIFO underrun/overrun error (FRUN)
  *            @arg SDIO_IT_HLE:  Hardware locked write error (HLE)
  *            @arg SDIO_IT_SBE:  Start-bit error (SBE)
  *            @arg SDIO_IT_ACD:  Auto command done (ACD)
  *            @arg SDIO_IT_EBE:  End-bit error (read)/write no CRC (EBE)
  *            @arg SDIO_IT_FROM_CARD: Interrupt from SDIO card
  * @retval None
  */
ITStatus SDIO_GetITStatus(SDIO_TypeDef * SDIOx,uint32_t SDIO_IT)
{
  ITStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_SDIO_GET_IT(SDIO_IT));
  if ((SDIOx->INTSTS & SDIO_IT) != (uint32_t)RESET)
  {
      bitstatus = SET;
  }
  else
  {
      bitstatus = RESET;
  }
  return bitstatus;
}

#endif /* HAL_SDIO_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
