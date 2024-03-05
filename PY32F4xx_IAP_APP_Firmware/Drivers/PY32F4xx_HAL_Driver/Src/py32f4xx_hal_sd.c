/**
  ******************************************************************************
  * @file    py32f4xx_hal_sd.c
  * @author  MCU Application Team
  * @brief   SD card HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Secure Digital (SD) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
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

/** @defgroup SD SD
* @brief SD HAL module driver
  * @{
  */
  
#ifdef HAL_SD_MODULE_ENABLED

/* Private macro -------------------------------------------------------------*/
#define SD_CLK_DIV (5)
#define TX_FIFO_DEPTH (SDIO_FIFO_WMARK_8)
#define RX_FIFO_DEPTH (SDIO_FIFO_WMARK_7)
#define SD_DMA_MODE             

/* Private variables ---------------------------------------------------------*/
static uint8_t  CardType = SDIO_STD_CAPACITY_SD_CARD_V1_1; 

static uint32_t CSD_Tab[4],CID_Tab[4];  

static uint16_t RCA=0;         /* Relative Card Address */ 
static uint8_t DMA_MODE=0;     /* DMA Mode */ 

static uint32_t *pReadBuf = NULL,*pWriteBuf = NULL;
static uint32_t gReadCnt = 0;
static uint32_t gTransferCnt = 0,gWriteCnt = 0;
SD_Error gError = HAL_SD_ERROR_NONE;

__IO uint32_t StopCondition = 0;
__IO FlagStatus TransferError = SET;
__IO FlagStatus DMAEndOfTransfer = SET;

SDIO_InitTypeDef     SDIO_InitStructure;
SDIO_CmdInitTypeDef  SDIO_CmdInitStructure;
SDIO_DataInitTypeDef SDIO_DataInitStructure;  

DMA_HandleTypeDef SD_DMA_ReadBlockHandle;
DMA_HandleTypeDef SD_DMA_WriteBlockHandle;

SD_CardInfo SDCardInfo;

/* Private function prototypes -----------------------------------------------*/
SD_Error CmdResp1Error(void);
SD_Error CmdResp2Error(void);
SD_Error CmdResp3Error(void);
SD_Error CmdResp6Error(uint16_t *prca);
SD_Error CmdResp7Error(void);
SD_Error SD_EnWideBus(uint32_t enx);
SD_Error SD_GetCardStatus(uint32_t *pstatus);
void SD_WaitCardOperation(void);
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo);

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Send the Select Deselect command and check the response.
  * @param  addr: Address of the card to be selected.
  * @retval SD Card error state
  */
SD_Error HAL_SD_SelectDeselect(uint32_t addr)
{
  /* Send CMD7 SDMMC_SEL_DESEL_CARD */
  SDIO_CmdInitStructure.Argument      = addr;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_SEL_DESEL_CARD;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
  return CmdResp1Error();   
}

/**
  * @brief  Enables wide bus operation for the requested card if supported by
  *         card.
  * @param  WideMode: Specifies the SD card wide bus mode
  *          This parameter can be one of the following values:
  *            @arg SDIO_BUS_WIDE_1B: 1-bit data transfer
  *            @arg SDIO_BUS_WIDE_4B: 4-bit data transfer
  * @retval HAL status
  */
SD_Error HAL_SD_EnableWideBusOperation(uint32_t WideMode)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  errorstatus = SD_EnWideBus(WideMode);
  
  if(HAL_SD_ERROR_NONE == errorstatus)
  {
    SDIO_SetCardBusWidth(SDIO,WideMode);
  }

 return errorstatus; 
}

/**
  * @brief  Enquires cards about their operating voltage and configures clock
  *         controls.
  * @retval error state
  */
SD_Error HAL_SD_PowerON(void)
{ 
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  uint32_t response=0,count=0,validvoltage=0;
  
  /* Initialize SDIO peripheral interface */
  SDIO_InitStructure.ClockDiv       = SDIO_INIT_CLK_DIV_DEFAULT; 
  SDIO_InitStructure.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE; 
  SDIO_InitStructure.BusWide        = SDIO_BUS_WIDE_1B;      
  SDIO_Init(SDIO,&SDIO_InitStructure);
  
  /* Set Power State to ON */
  SDIO_SetPowerState(SDIO,SDIO_POWER_STATE_ON);
  
  /* CMD0: GO_IDLE_STATE */
  SDIO_CmdInitStructure.InitSignal   = SDIO_INITSIGNAL_ENABLE;
  SDIO_CmdInitStructure.Argument     = 0x0;
  SDIO_CmdInitStructure.CmdIndex     = SDIO_CMD_GO_IDLE_STATE; 
  SDIO_CmdInitStructure.DataTransfer = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_NO;  
  SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_CRC_DISABLE;
  SDIO_CmdInitStructure.Wait         = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM         = SDIO_CPSM_ENABLE;  
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);  
  for(;;)
  {
    if(SDIO_GetITStatus(SDIO,SDIO_IT_CMDD) ==SET)
      break;
  }
  SDIO_ClearITStatus(SDIO,SDIO_IT_CMDD);
  
  /* CMD8: SEND_IF_COND: Command available only on V2.0 cards */
  SDIO_CmdInitStructure.InitSignal   = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Argument     = SD_CHECK_PATTERN; 
  SDIO_CmdInitStructure.CmdIndex     = SDIO_CMD_HS_SEND_EXT_CSD;  
  SDIO_CmdInitStructure.DataTransfer = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_SHORT; 
  SDIO_CmdInitStructure.Wait         = SDIO_WAIT_NO; 
  SDIO_CmdInitStructure.CPSM         = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);  
  
  errorstatus = CmdResp7Error();
  if(errorstatus == HAL_SD_ERROR_NONE)
  {
    CardType=SDIO_STD_CAPACITY_SD_CARD_V2_0;
  }
  else
  {
    return errorstatus;
  }

   /* SEND CMD55 APP_CMD with RCA as 0 */
  while((!validvoltage)&&(count<SD_MAX_VOLT_TRIAL))
  { 
    SDIO_CmdInitStructure.InitSignal   = SDIO_INITSIGNAL_DISABLE;  
    SDIO_CmdInitStructure.Argument     = 0x00;
    SDIO_CmdInitStructure.CmdIndex     = SDIO_CMD_APP_CMD;   
    SDIO_CmdInitStructure.DataTransfer = SDIO_DATATRANSFER_DISABLE;
    SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_SHORT;
    SDIO_CmdInitStructure.Wait         = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSM         = SDIO_CPSM_ENABLE;
    SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);   
        
    errorstatus = CmdResp1Error();    
        
    if(errorstatus != HAL_SD_ERROR_NONE)
      return errorstatus;    
  
    /* SD CARD */
    /* Send ACMD41 SD_APP_OP_COND with Argument 0x40100000 */
    SDIO_CmdInitStructure.InitSignal   = SDIO_INITSIGNAL_DISABLE;
    SDIO_CmdInitStructure.Argument     = 0x40100000;
    SDIO_CmdInitStructure.CmdIndex     = SDIO_CMD_SD_APP_OP_COND;
    SDIO_CmdInitStructure.DataTransfer = SDIO_DATATRANSFER_DISABLE;
    SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_SHORT; 
    SDIO_CmdInitStructure.Wait         = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSM         = SDIO_CPSM_ENABLE;
    SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
      
    errorstatus=CmdResp3Error();      
      
    if(errorstatus!= HAL_SD_ERROR_NONE)
      return errorstatus;    
    response = SDIO->RESP0;
    
    /* Get operating voltage*/
    validvoltage = (((response>>31)==1)?1:0);   
    count++;
  }
  if(count >= SD_MAX_VOLT_TRIAL)
  {
    errorstatus = HAL_SD_ERROR_DE;
    return errorstatus;
  }
  if(response &= SD_HIGH_CAPACITY)
  {
    CardType = SDIO_HIGH_CAPACITY_SD_CARD;
  }
  return errorstatus;
}

SD_Error HAL_SD_InitializeCards(void)
{
  SD_Error errorstatus=HAL_SD_ERROR_NONE;
  uint16_t rca = 0x01;
  
  /* Check the power State */
  if (SDIO_GetPowerState(SDIO) == SDIO_POWER_STATE_OFF) 
  {
    errorstatus = HAL_SD_REQUEST_NOT_APPLICABLE;
    return(errorstatus);
  }

  if(CardType != SDIO_SECURE_DIGITAL_IO_CARD)
  {
    /*!< Send CMD2 ALL_SEND_CID */
    SDIO_CmdInitStructure.InitSignal   = SDIO_INITSIGNAL_DISABLE;
    SDIO_CmdInitStructure.Argument     = 0x0;
    SDIO_CmdInitStructure.CmdIndex     = SDIO_CMD_ALL_SEND_CID;
    SDIO_CmdInitStructure.DataTransfer = SDIO_DATATRANSFER_DISABLE;
    SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_LONG;
    SDIO_CmdInitStructure.Wait         = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.Response_CRC = SDIO_RESPONSE_CRC_ENABLE;
    SDIO_CmdInitStructure.CPSM         = SDIO_CPSM_ENABLE;
    SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
    
    errorstatus = CmdResp2Error();
    
    if(errorstatus != HAL_SD_ERROR_NONE)
    {
      return errorstatus;
    }
    CID_Tab[0]=SDIO->RESP3;
    CID_Tab[1]=SDIO->RESP2;
    CID_Tab[2]=SDIO->RESP1;
    CID_Tab[3]=SDIO->RESP0;
  }
  
  if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType)||(SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) \
    ||(SDIO_SECURE_DIGITAL_IO_COMBO_CARD == CardType)||(SDIO_HIGH_CAPACITY_SD_CARD == CardType))
  {
    /* Send CMD3 SET_REL_ADDR with argument 0 */
    /* SD Card publishes its RCA. */
    SDIO_CmdInitStructure.InitSignal   = SDIO_INITSIGNAL_DISABLE;
    SDIO_CmdInitStructure.Argument     = 0x00;
    SDIO_CmdInitStructure.CmdIndex     = SDIO_CMD_SET_REL_ADDR; 
    SDIO_CmdInitStructure.DataTransfer = SDIO_DATATRANSFER_DISABLE;
    SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_SHORT;
    SDIO_CmdInitStructure.Wait         = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSM         = SDIO_CPSM_ENABLE;
    SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure); 
    
    errorstatus=CmdResp6Error(&rca);
    
    if(errorstatus != HAL_SD_ERROR_NONE)
      return errorstatus;
    
    RCA = rca;
  }
  #ifndef EDA_SIM 
  if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)
  {
    /*!< Send CMD9 SEND_CSD with argument as card's RCA */
    SDIO_CmdInitStructure.InitSignal    = SDIO_INITSIGNAL_DISABLE;
    SDIO_CmdInitStructure.Argument      = (uint32_t)(rca << 16);
    SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_SEND_CSD;
    SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_LONG;
    SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.Response_CRC  = SDIO_RESPONSE_CRC_ENABLE;
    SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_DISABLE;
    SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
    SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
    errorstatus = CmdResp2Error();
    if(errorstatus != HAL_SD_ERROR_NONE)
      return errorstatus;
  
    CSD_Tab[0] = SDIO->RESP3;
    CSD_Tab[1] = SDIO->RESP2;
    CSD_Tab[2] = SDIO->RESP1;
    CSD_Tab[3] = SDIO->RESP0;
  }
  #endif
  return HAL_SD_ERROR_NONE;
} 

/**
  * @brief  Initializes the SD according to the specified parameters in the
            SD_HandleTypeDef and create the associated handle.
  * @retval HAL status
  */
SD_Error HAL_SD_Init(void)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  uint8_t clkdiv=0;
  
  /* Disable OD PULLUP */
  __SDIO_OD_PULLUP_DISABLE();
  
  /* Enquires cards about their operating voltage and configures clock controls. */
  errorstatus = HAL_SD_PowerON();
  if(errorstatus != HAL_SD_ERROR_NONE)
  {
    return errorstatus;
  }
  
  /* SD Card publishes its RCA. */
  errorstatus = HAL_SD_InitializeCards();
  if(errorstatus != HAL_SD_ERROR_NONE)
  {
    return errorstatus;
  }
  
  /* Get card information */
  errorstatus = SD_GetCardInfo(&SDCardInfo);
  if(errorstatus != HAL_SD_ERROR_NONE)
  {
    return errorstatus;
  }
  
  /* Send the Select Deselect command and check the response. */
  errorstatus = HAL_SD_SelectDeselect(RCA<<16);
  
  if(errorstatus == HAL_SD_ERROR_NONE)
  {
#ifdef WIDE_BUS
    errorstatus = HAL_SD_EnableWideBusOperation(SDIO_BUS_WIDE_4B);
#else
    errorstatus = HAL_SD_EnableWideBusOperation(SDIO_BUS_WIDE_1B);
#endif
    if(errorstatus != HAL_SD_ERROR_NONE)
      return errorstatus;
  }
  /* set clock speed */
#if 1
  if((errorstatus == HAL_SD_ERROR_NONE)||(SDIO_MULTIMEDIA_CARD == CardType))
  {
        /*!< Card Version V1.1/V2.0 */
    if(SDCardInfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1||SDCardInfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0)
      clkdiv = 2;
    else
      clkdiv = SD_CLK_DIV;
  
        /*!< Configure the SDIO peripheral */
    SDIO_SetClock(SDIO,clkdiv);
  }
#endif
  
  /* Fifo Configure TX_WMark = 8, RX_WMark = 7 */
  SDIO_SetTxWaterMark(SDIO,TX_FIFO_DEPTH);
  SDIO_SetRxWaterMark(SDIO,RX_FIFO_DEPTH);
  
#if defined(SD_POLLING_MODE)
#elif defined(SD_DMA_MODE)
  SDIO_DMACmd(SDIO,ENABLE);
  HAL_SD_DMA_ReadBlockInit(&SD_DMA_ReadBlockHandle);
  HAL_SD_DMA_WriteBlockInit(&SD_DMA_WriteBlockHandle);
#elif defined(SD_IT_MODE)
  NVIC_EnableIRQ(SDIO_IRQn);
#endif

  return errorstatus;
}

/**
  * @brief  Allows to erase memory area specified for the given card.
  * @param  startaddr: the start address.
  * @param  endaddr: the end address.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error HAL_SD_Erase(uint32_t startaddr, uint32_t endaddr)
{
    SD_Error errorstatus = HAL_SD_ERROR_NONE;
    uint32_t R1;

#ifndef EDA_SIM
    /*!< Check if the card command class supports erase command */
    if (((CSD_Tab[1] >> 20) & SD_CCCC_ERASE) == 0)
    {
      errorstatus = HAL_SD_REQUEST_NOT_APPLICABLE;
      return (errorstatus);
    }
#endif

  /* Check Card Status */
  if(HAL_SD_ERROR_NONE != SD_GetCardStatus(&R1))
  {
    return errorstatus;
  }
  else if(R1 & SD_CARD_LOCKED)
  {
    errorstatus = HAL_SD_LOCK_UNLOCK_FAILED;
    return errorstatus;
  }
  
  /*!< According to sd-card spec 1.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33) */
  if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
  {
    /*!< Send CMD32 SD_ERASE_GRP_START with argument as addr  */
    SDIO_CmdInitStructure.Argument = startaddr;
    SDIO_CmdInitStructure.CmdIndex = SDIO_CMD_SD_ERASE_GRP_START;
    SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
    SDIO_CmdInitStructure.Wait     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSM     = SDIO_CPSM_ENABLE;
    SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
    errorstatus = CmdResp1Error();
    if (errorstatus != HAL_SD_ERROR_NONE)
    {
      return (errorstatus);
    }
  
    /*!< Send CMD33 SD_ERASE_GRP_END with argument as addr  */
    SDIO_CmdInitStructure.Argument = endaddr;
    SDIO_CmdInitStructure.CmdIndex = SDIO_CMD_SD_ERASE_GRP_END;
    SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
    SDIO_CmdInitStructure.Wait     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSM     = SDIO_CPSM_ENABLE;
    SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
    errorstatus = CmdResp1Error();
    if (errorstatus != HAL_SD_ERROR_NONE)
    {
      return (errorstatus);
    }
  }
  
  /*!< Send CMD38 ERASE */
  SDIO_CmdInitStructure.Argument = 0;
  SDIO_CmdInitStructure.CmdIndex = SDIO_CMD_ERASE;
  SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait     = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM     = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
  errorstatus = CmdResp1Error();
  
  if (errorstatus != HAL_SD_ERROR_NONE)
  {
      return (errorstatus);
  }
  
  /* wait card erase operation complete */
  while(SDIO->STATUS & SDIO_STATUS_CARDBSY);

  return (errorstatus);
}

/**
  * @brief  Aborts an ongoing data transfer.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
SD_Error HAL_SD_StopTransfer(void)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;

  /*!< Send CMD12 STOP_TRANSMISSION  */
  SDIO_CmdInitStructure.Argument = 0;
  SDIO_CmdInitStructure.CmdIndex = SDIO_CMD_STOP_TRANSMISSION;
  SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error();

  return(errorstatus);
}


/**
  * @brief  Reads block(s) from a specified address in a card. The Data transfer
  *         is managed by polling mode.
  * @param  pData: pointer to the buffer that will contain the received data
  * @param  BlockAdd: Block Address from where data is to be read
  * @param  Blocksize: Size of SD blocks to read
  * @retval SD Error status
  */
SD_Error HAL_SD_ReadBlock(uint32_t *pData,uint32_t BlockAdd,uint16_t Blocksize)
{   
  SD_Error errorstatus=HAL_SD_ERROR_NONE;
  
  uint32_t count=0;
  uint32_t timeout=SDIO_DATATIMEOUT;   
  
  /* send CMD16 config block size*/
  SDIO_CmdInitStructure.InitSignal   = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Argument     = Blocksize;
  SDIO_CmdInitStructure.CmdIndex     = SDIO_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.DataTransfer = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait         = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM         = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
    
  errorstatus=CmdResp1Error();
    
  if(errorstatus!=HAL_SD_ERROR_NONE)
  {
    return errorstatus;
  }
      
  SDIO_DataInitStructure.DataBlockSize = Blocksize; 
  SDIO_DataInitStructure.DataLength    = Blocksize ;
  SDIO_DataInitStructure.DataTimeOut   = SDIO_DATATIMEOUT ;
  SDIO_DataInitStructure.TransferDir   = SDIO_TRANSFERDIR_TO_SDIO;
  SDIO_DataInitStructure.TransferMode  = SDIO_TRANSFERMODE_BLOCK;
  SDIO_DataConfig(SDIO,&SDIO_DataInitStructure);
  
  /* send CMD17 read data */
  SDIO_CmdInitStructure.InitSignal   = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Argument     = BlockAdd;
  SDIO_CmdInitStructure.CmdIndex     = SDIO_CMD_READ_SINGLE_BLOCK;
  SDIO_CmdInitStructure.DataTransfer = SDIO_DATATRANSFER_ENABLE;
  SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait         = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM         = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
  errorstatus=CmdResp1Error();
  
  if(errorstatus!=HAL_SD_ERROR_NONE)
  {
    return errorstatus;
  }
  
  if(DMA_MODE == 0)
  {
    /* Poll on SDIO flags */
    while(!(SDIO->INTSTS&((1<<3)|(1<<9)|(1<<10)|(1<<7)|(1<<13)|(1<<15)|(1<<11))))
    {
      if(SDIO_GetITStatus(SDIO,SDIO_IT_RXDR) != RESET)      
      {
        SDIO_ClearITStatus(SDIO,SDIO_IT_RXDR);
        for(count=0;count<8;count++)  
        {
          *(pData+count)=SDIO->FIFODATA;
        }
        pData+=8;  
        timeout=0X7FFFFF; 
      }
      else 
      {
        if(timeout==0)
          return HAL_SD_ERROR_DRTO;
        timeout--;
      }
    } 
    if(SDIO_GetITStatus(SDIO,SDIO_IT_DRTO) != RESET) 
    {             
      SDIO_ClearITStatus(SDIO,SDIO_IT_DRTO);  
      return HAL_SD_ERROR_DRTO;
    }
    else if(SDIO_GetITStatus(SDIO,SDIO_IT_DCRC) != RESET) 
    {
      SDIO_ClearITStatus(SDIO,SDIO_IT_DCRC);  
      return HAL_SD_ERROR_DCRC;     
    }
    else if(SDIO_GetITStatus(SDIO,SDIO_IT_FRUN) != RESET) 
    {
      SDIO_ClearITStatus(SDIO,SDIO_IT_FRUN);
      return HAL_SD_ERROR_FRUN;   
    }
    else if(SDIO_GetITStatus(SDIO,SDIO_IT_SBE) != RESET) 
    {
      SDIO_ClearITStatus(SDIO,SDIO_IT_SBE);
      return HAL_SD_ERROR_SBE;   
    }
    else if(SDIO_GetITStatus(SDIO,SDIO_IT_EBE) != RESET) 
    {
      SDIO_ClearITStatus(SDIO,SDIO_IT_EBE);
      return HAL_SD_ERROR_EBE;   
    }      
  
    SDIO_ClearITStatus(SDIO,SDIO_STATIC_FLAGS);
  }
  else
  {
  }
  
  return errorstatus; 
}

/**
  * @brief  Allows to write block(s) to a specified address in a card. The Data
  *         transfer is managed by polling mode.
  * @param  pData: pointer to the buffer that will contain the data to transmit
  * @param  BlockAdd: Block Address where data will be written
  * @param  DataLength: Specifies the number of data bytes to be transferred
  * @retval SD Error status
  */
SD_Error HAL_SD_WriteBlock(uint32_t *pData,uint32_t BlockAdd,  uint16_t DataLength)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  
  uint32_t timeout = 0,bytestransferred = 0;
  
  uint32_t count = 0,restwords = 0;
  
  uint32_t tlen = DataLength;
  
  /* send CMD16 config block size*/
  SDIO_CmdInitStructure.InitSignal   = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Argument     = DataLength;
  SDIO_CmdInitStructure.CmdIndex     = SDIO_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.DataTransfer = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait         = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM         = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure); 
    
  errorstatus=CmdResp1Error(); 
    
  if(errorstatus!= HAL_SD_ERROR_NONE)
  {
    return errorstatus;  
  }
                  
  SDIO_DataInitStructure.DataBlockSize = DataLength; 
  SDIO_DataInitStructure.DataLength    = DataLength ;
  SDIO_DataInitStructure.DataTimeOut   = SDIO_DATATIMEOUT ;
  SDIO_DataInitStructure.TransferDir   = SDIO_TRANSFERDIR_TO_CARD;
  SDIO_DataInitStructure.TransferMode  = SDIO_TRANSFERMODE_BLOCK;
  SDIO_DataConfig(SDIO,&SDIO_DataInitStructure);
  
  /* send CMD24 write data */
  SDIO_CmdInitStructure.InitSignal   = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Argument     = BlockAdd;
  SDIO_CmdInitStructure.CmdIndex     = SDIO_CMD_WRITE_SINGLE_BLOCK;
  SDIO_CmdInitStructure.DataTransfer = SDIO_DATATRANSFER_ENABLE;
  SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait         = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM         = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure); 
  
  errorstatus=CmdResp1Error();
  
  if(errorstatus!= HAL_SD_ERROR_NONE)
    return errorstatus;     
  
  timeout=SDIO_DATATIMEOUT;
  
    if(DMA_MODE == 0)
  {
    /* Poll on SDIO flags */
    while(!(SDIO->INTSTS&((1<<3)|(1<<9)|(1<<10)|(1<<7)|(1<<13)|(1<<15)|(1<<11))))
    {
      if(SDIO_GetITStatus(SDIO,SDIO_IT_TXDR) != RESET)       
      {
        if((tlen-bytestransferred)< 32)
        {
          restwords=((tlen-bytestransferred)%4==0)?((tlen-bytestransferred)/4):((tlen-bytestransferred)/4+1);
          
          for(count=0;count<restwords;count++,pData++,bytestransferred+=4)
          {
            SDIO->FIFODATA=*pData;
          }
        }
        else
        {
          for(count=0;count<8;count++)
          {
            SDIO->FIFODATA=*(pData+count);
          }
          pData+=8;
          bytestransferred+=32;
        }
        timeout=0X3FFFFFFF;
      }
      else
      {
        if(timeout==0)
          return HAL_SD_ERROR_DRTO;
        
        timeout--;
      }
    } 
    /* wait Data Transfer Over */
    while(SDIO_GetITStatus(SDIO,SDIO_IT_DTO) == RESET);
    /* wait Card Idle */
    while(SDIO_IsCardBusy(SDIO));
  
    if(SDIO_GetITStatus(SDIO,SDIO_IT_DRTO) != RESET) 
    {             
      SDIO_ClearITStatus(SDIO,SDIO_IT_DRTO); 
      return HAL_SD_ERROR_DRTO;
    }
    else if(SDIO_GetITStatus(SDIO,SDIO_IT_DCRC) != RESET) 
    {
      SDIO_ClearITStatus(SDIO,SDIO_IT_DCRC);   
      return HAL_SD_ERROR_DCRC;     
    }
    else if(SDIO_GetITStatus(SDIO,SDIO_IT_FRUN) != RESET) 
    {
      SDIO_ClearITStatus(SDIO,SDIO_IT_FRUN); 
      return HAL_SD_ERROR_FRUN;   
      }
    else if(SDIO_GetITStatus(SDIO,SDIO_IT_SBE) != RESET)  
    {
      SDIO_ClearITStatus(SDIO,SDIO_IT_SBE);
      return HAL_SD_ERROR_SBE;   
    }
    else if(SDIO_GetITStatus(SDIO,SDIO_IT_EBE) != RESET)  
    {
      SDIO_ClearITStatus(SDIO,SDIO_IT_EBE);
      return HAL_SD_ERROR_EBE;   
    }      
      SDIO_ClearITStatus(SDIO,SDIO_STATIC_FLAGS);
    }
  else
  {
  }
  
  return errorstatus;
}

/**
  * @brief  Reads block(s) from a specified address in a card. The Data transfer
  *         is managed by polling mode.
  * @param  pData: pointer to the buffer that will contain the received data
  * @param  BlockAdd: Block Address from where data is to be read
  * @param  Blocksize: Size of SD blocks to read
  * @param  NumberOfBlocks: Number of SD blocks to read
  * @retval SD Error status
  */
SD_Error SD_ReadMultiBlocks(uint8_t *pData, uint32_t BlockAdd, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;

  uint32_t *tempbuff = (uint32_t*)pData;
  uint32_t idx = 0;

  /*!< Set Block Size for Card */
  SDIO_CmdInitStructure.Argument      = (uint32_t) BlockSize;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_CmdInitStructure.Response_CRC  = SDIO_RESPONSE_CRC_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_DISABLE;

  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error();

  if (HAL_SD_ERROR_NONE != errorstatus)
  {
      return(errorstatus);
  }

  SDIO_DataInitStructure.DataTimeOut   = 0xFFFFFFFF;
  SDIO_DataInitStructure.DataLength    = NumberOfBlocks * BlockSize;
  SDIO_DataInitStructure.DataBlockSize = 512;
  SDIO_DataInitStructure.TransferDir   = SDIO_TRANSFERDIR_TO_SDIO;
  SDIO_DataInitStructure.TransferMode  = SDIO_TRANSFERMODE_BLOCK;
  SDIO_DataConfig(SDIO,&SDIO_DataInitStructure);

  /*!< Send CMD18 READ_MULT_BLOCK with argument data address */
  SDIO_CmdInitStructure.Argument      = (uint32_t)BlockAdd;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_READ_MULT_BLOCK;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_CmdInitStructure.Response_CRC  = SDIO_RESPONSE_CRC_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_ENABLE;
  /* AutoStop Enable */
  SDIO_IsAutoStopCmd(SDIO,ENABLE);
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error();

  if (errorstatus != HAL_SD_ERROR_NONE)
  {
      return(errorstatus);
  }
  
  while(!SDIO_GetITStatus(SDIO,SDIO_INTSTS_SBE | SDIO_INTSTS_EBE | SDIO_INTSTS_DCRC | SDIO_INTSTS_DRTO_BDS))
  {
     /* RX FIFO WaterMark */
    if(SDIO->INTSTS & SDIO_INTSTS_RXDR)
    {
      while(!(SDIO->STATUS & SDIO_STATUS_FIFOE))
      {
        tempbuff[idx++] = SDIO_ReadData(SDIO);
      }

      SDIO_ClearITStatus(SDIO,SDIO_INTSTS_RXDR);
    }
    if(SDIO->INTSTS & SDIO_INTSTS_DTO)
    {
      while(!(SDIO->STATUS & SDIO_STATUS_FIFOE))
      {
        tempbuff[idx++] = SDIO_ReadData(SDIO);
      }
      SDIO_ClearITStatus(SDIO,SDIO_INTSTS_DTO);
      break;
    }
  }
  
  if(SDIO_GetITStatus(SDIO,SDIO_INTSTS_SBE))
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_SBE);
    return HAL_SD_ERROR_SBE;
  }
  if(SDIO_GetITStatus(SDIO,SDIO_INTSTS_EBE))
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_EBE);
    return HAL_SD_ERROR_EBE;
  }
  if(SDIO_GetITStatus(SDIO,SDIO_INTSTS_DCRC))
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_DCRC);
    return HAL_SD_ERROR_DCRC;
  }
  if(SDIO_GetITStatus(SDIO,SDIO_INTSTS_DRTO_BDS))
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_DRTO_BDS);
    return HAL_SD_ERROR_DRTO;
  }

    return(errorstatus);
}

/**
  * @brief  Allows to write block(s) to a specified address in a card. The Data
  *         transfer is managed by polling mode.
  * @param  pData: pointer to the buffer that will contain the data to transmit
  * @param  BlockAdd: Block Address where data will be written
  * @param  NumberOfBlocks: Number of SD blocks to read.
  * @retval SD Error status
  */SD_Error SD_WriteMultiBlocks(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  
  uint32_t tlen = BlockSize * NumberOfBlocks;
  
  uint32_t idx = 0;
  uint32_t bytestransferred=0;
  uint32_t count=0,restwords=0;
  uint32_t *tempbuff = (uint32_t *)writebuff;

  /* Set Block Size for Card */
  SDIO_CmdInitStructure.Argument      = (uint32_t) BlockSize;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_DISABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error();
 
  if (HAL_SD_ERROR_NONE != errorstatus)
  {
      return(errorstatus);
  }
 
  SDIO_DataInitStructure.DataTimeOut    = 0xFFFFFFFF;
  SDIO_DataInitStructure.DataLength     = NumberOfBlocks * BlockSize;
  SDIO_DataInitStructure.DataBlockSize  = 512;
  SDIO_DataInitStructure.TransferDir    = SDIO_TRANSFERDIR_TO_CARD;
  SDIO_DataInitStructure.TransferMode   = SDIO_TRANSFERMODE_BLOCK;
  SDIO_DataConfig(SDIO,&SDIO_DataInitStructure);

  /* Auto-stop after all bytes transfer */
  SDIO_IsAutoStopCmd(SDIO,ENABLE);
  /*!< Send CMD25 WRITE_MULT_BLOCK with argument data address */
  SDIO_CmdInitStructure.Argument      = (uint32_t)WriteAddr;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_WRITE_MULT_BLOCK;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
  errorstatus = CmdResp1Error();
  if (HAL_SD_ERROR_NONE != errorstatus)
  {
      return(errorstatus);
  }
  /*!< In case of single data block transfer no need of stop command at all */
  while(!SDIO_GetITStatus(SDIO,SDIO_INTSTS_DCRC | SDIO_INTSTS_DRTO_BDS | SDIO_INTSTS_EBE))
  {
    if(SDIO_GetITStatus(SDIO,SDIO_INTSTS_TXDR))
    {
      /*not enough 32 bytes*/
      if((tlen - bytestransferred) < 32)
      {
        restwords=((tlen-bytestransferred)%4==0)?((tlen-bytestransferred)/4):((tlen-bytestransferred)/4+1);
        for(count = 0; count < restwords; count++)
        {
          SDIO_WriteData(SDIO,tempbuff[idx++]);
          bytestransferred += 4;
        }
      }
      else
      {
        for(count = 0; count < 8; count++)
        {
          /* Fifo full */
          while(SDIO->STATUS & SDIO_STATUS_FIFOF)
          {}
          SDIO_WriteData(SDIO,tempbuff[idx++]);
        }
        bytestransferred += 32;
      }
      SDIO_ClearITStatus(SDIO,SDIO_INTSTS_TXDR);
    }
    if(SDIO_GetITStatus(SDIO,SDIO_INTSTS_DTO))
    {
      SDIO_ClearITStatus(SDIO,SDIO_INTSTS_DTO);
      break;
    }
  }
  
  while(SDIO->STATUS & SDIO_STATUS_CARDBSY_Msk);
  
  if(SDIO_GetITStatus(SDIO,SDIO_INTSTS_SBE))
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_SBE);
    return HAL_SD_ERROR_SBE;
  }
  if(SDIO_GetITStatus(SDIO,SDIO_INTSTS_EBE))
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_EBE);
    return HAL_SD_ERROR_EBE;
  }
  if(SDIO_GetITStatus(SDIO,SDIO_INTSTS_DCRC))
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_DCRC);
    return HAL_SD_ERROR_RCRC;
  }
  if(SDIO_GetITStatus(SDIO,SDIO_INTSTS_DRTO_BDS))
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_DRTO_BDS);
    return HAL_SD_ERROR_DRTO;
  }
  return(errorstatus);
}


/**
  * @brief Rx Transfer completed callbacks
  * @param hdma: Pointer to DMA handle
  * @retval None
  */
void HAL_SD_DMA_RxCpltCallback(DMA_HandleTypeDef *hdma)
{
  DMAEndOfTransfer = SET;
}

/**
  * @brief Rx Transfer error callbacks
  * @param hdma: Pointer to DMA handle
  * @retval None
  */
void HAL_SD_DMA_RxErrorCallback(DMA_HandleTypeDef *hdma)
{
  TransferError = SET;
}

/**
  * @brief Tx Transfer completed callbacks
  * @param hdma: Pointer to DMA handle
  * @retval None
  */
void HAL_SD_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma)
{
  DMAEndOfTransfer = SET;
}

/**
  * @brief Tx Transfer error callbacks
  * @param hdma: Pointer to DMA handle
  * @retval None
  */
void HAL_SD_DMA_TxErrorCallback(DMA_HandleTypeDef *hdma)
{
  TransferError = SET;
}

/**
  * @brief Initialize block read as DMA
  * @param hdma_sd: Pointer to DMA handle
  * @retval None
  */
void HAL_SD_DMA_ReadBlockInit(DMA_HandleTypeDef  *hdma_sd)
{
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  
  /* DMA2_Channel3 for SD WriteBlock */
  SYSCFG->CFGR[4] &= ~SYSCFG_CFGR5_DMA10_MAP_Msk;
  SYSCFG->CFGR[4] |= (0x43 << SYSCFG_CFGR5_DMA10_MAP_Pos);
  
  /*##-1- Configure the DMA ##########################################*/
  /* Set the parameters to be configured for DACx_DMA_STREAM */
  hdma_sd->Instance                 = DMA2_Channel3;
  hdma_sd->Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_sd->Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_sd->Init.MemInc              = DMA_MINC_ENABLE;
  hdma_sd->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_sd->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma_sd->Init.Mode                = DMA_NORMAL;
  hdma_sd->Init.Priority            = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(hdma_sd);
  
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  
  hdma_sd->XferCpltCallback = HAL_SD_DMA_RxCpltCallback;
  hdma_sd->XferErrorCallback = HAL_SD_DMA_RxErrorCallback;
}

/**
  * @brief Initialize block Write as DMA
  * @param hdma_sd: Pointer to DMA handle
  * @retval None
  */
void HAL_SD_DMA_WriteBlockInit(DMA_HandleTypeDef  *hdma_sd)
{
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  
  /* DMA2_Channel4 for SD ReadBlock */
  SYSCFG->CFGR[4] &= ~SYSCFG_CFGR5_DMA11_MAP_Msk;
  SYSCFG->CFGR[4] |= (0x43 << SYSCFG_CFGR5_DMA11_MAP_Pos);
  
  /*##-1- Configure the DMA ##########################################*/
  /* Set the parameters to be configured for DACx_DMA_STREAM */
  hdma_sd->Instance = DMA2_Channel4;
  hdma_sd->Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_sd->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_sd->Init.MemInc = DMA_MINC_ENABLE;
  hdma_sd->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_sd->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_sd->Init.Mode = DMA_NORMAL;
  hdma_sd->Init.Priority = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(hdma_sd);
  
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  
  hdma_sd->XferCpltCallback = HAL_SD_DMA_TxCpltCallback;
  hdma_sd->XferErrorCallback = HAL_SD_DMA_TxErrorCallback;
}

/**
  * @brief  Reads block(s) from a specified address in a card. The Data transfer
  *         is managed by DMA mode.
  * @note   You could also check the DMA transfer process through the SD Rx
  *         interrupt event.
  * @param  hdma_sdio: Pointer to DMA handle
  * @param  pData: Pointer to the buffer that will contain the received data
  * @param  BlockAdd: Block Address from where data is to be read
  * @param  BlockSize: Block size.
  * @retval SDIO Error status
  */
SD_Error HAL_SD_ReadBlockDMA (DMA_HandleTypeDef  *hdma_sdio,uint8_t *pData,uint32_t BlockAdd,uint16_t BlockSize)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  uint16_t dma_cnt = BlockSize >> 2;
  
  if(pData == NULL)
    return HAL_SD_ERROR_INVALID_PARAMETER;

  /* Send CMD16 set Block Size for Card */
  SDIO_CmdInitStructure.Argument      = BlockSize;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.InitSignal    = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Response_CRC  = SDIO_RESPONSE_CRC_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
 
  if(CmdResp1Error() != HAL_SD_ERROR_NONE)
  {
    return errorstatus;
  }
 
  SDIO_DataInitStructure.DataBlockSize  = BlockSize; 
  SDIO_DataInitStructure.DataLength     = BlockSize ;
  SDIO_DataInitStructure.DataTimeOut    = SDIO_DATATIMEOUT ;
  SDIO_DataInitStructure.TransferDir    = SDIO_TRANSFERDIR_TO_SDIO;
  SDIO_DataInitStructure.TransferMode   = SDIO_TRANSFERMODE_BLOCK;
  SDIO_DataConfig(SDIO,&SDIO_DataInitStructure);
  
  /*!< Send CMD17 READ_SINGLE_BLOCK */
  SDIO_CmdInitStructure.Argument      = BlockAdd;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_READ_SINGLE_BLOCK;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.InitSignal    = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Response_CRC  = SDIO_RESPONSE_CRC_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_ENABLE;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
  if(CmdResp1Error() != HAL_SD_ERROR_NONE)
    return errorstatus;
  
  /* Enable DMA */
  HAL_DMA_Start_IT(hdma_sdio, (uint32_t)&(SDIO->FIFODATA),(uint32_t)pData, dma_cnt);
  
  /* Wait DMA Complete */
  while(TransferError == RESET)
  {
    if(DMAEndOfTransfer)
      break;
  }
  TransferError = RESET;
  DMAEndOfTransfer = RESET;
  
  if(SDIO->INTSTS & SDIO_INTSTS_SBE)
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_SBE);
    return HAL_SD_ERROR_SBE;
  }
  if(SDIO->INTSTS & SDIO_INTSTS_EBE)
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_EBE);
    return HAL_SD_ERROR_EBE;
  }
  if(SDIO->INTSTS & SDIO_INTSTS_DCRC)
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_DCRC);
    return HAL_SD_ERROR_DCRC;
  }
  if(SDIO->INTSTS & SDIO_INTSTS_RTO_BAR)
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_RTO_BAR);
    return HAL_SD_ERROR_DRTO;
  }
  SDIO_ClearITStatus(SDIO,SDIO_STATIC_FLAGS);
  
  return errorstatus;
}

/**
  * @brief  Write block(s) from a specified address in a card. The Data transfer
  *         is managed by DMA mode.
  * @note   You could also check the DMA transfer process through the SD Tx
  *         interrupt event.
  * @param  hdma_sdio: Pointer to DMA handle
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  BlockAdd: Block Address from where data is to be write
  * @param  BlockSize: Block size.
  * @retval SDIO Error status
  */
SD_Error HAL_SD_WriteBlockDMA(DMA_HandleTypeDef  *hdma_sdio,uint8_t *pData, uint32_t BlockAdd,  uint16_t BlockSize)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  uint8_t cardstate = 0;
  uint32_t R1;
  
  uint16_t dma_cnt = BlockSize >> 2;
  
  /* wait card operation complete */
  while(SDIO->STATUS & SDIO_STATUS_CARDBSY)
  {
  }
  
  /* Set Block Size for Card */
  SDIO_CmdInitStructure.Argument      = BlockSize;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.InitSignal    = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Response_CRC  = SDIO_RESPONSE_CRC_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);

  if(CmdResp1Error() != HAL_SD_ERROR_NONE)
    return errorstatus;
  
  SDIO_DataInitStructure.DataBlockSize = BlockSize;
  SDIO_DataInitStructure.DataLength   = BlockSize ;
  SDIO_DataInitStructure.DataTimeOut  = SDIO_DATATIMEOUT ;
  SDIO_DataInitStructure.TransferDir  = SDIO_TRANSFERDIR_TO_CARD;
  SDIO_DataInitStructure.TransferMode = SDIO_TRANSFERMODE_BLOCK;
  SDIO_DataConfig(SDIO,&SDIO_DataInitStructure);
  
  /*!< Send CMD24 WRITE_SINGLE_BLOCK */
  SDIO_CmdInitStructure.Argument      = BlockAdd;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_WRITE_SINGLE_BLOCK;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.InitSignal    = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Response_CRC  = SDIO_RESPONSE_CRC_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_ENABLE;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
  if(CmdResp1Error() != HAL_SD_ERROR_NONE)
    return errorstatus;
  
  /* Enable DMA */
  HAL_DMA_Start_IT(hdma_sdio, (uint32_t)pData, (uint32_t)&(SDIO->FIFODATA),dma_cnt);
  
  /* Wait DMA Complete */
  while(TransferError == RESET)
  {
    if(DMAEndOfTransfer)
      break;
  }
  TransferError = RESET;
  DMAEndOfTransfer = RESET;
  
  /* wait card write complete */
  do
  {
    /*!< Wait till the card is in programming state */
    errorstatus = SD_GetCardStatus(&R1);
    /* get CARD CURRENT_STATE */
    cardstate = ((R1 >> 9) & 0x0000000F);
  
  }while ((errorstatus == HAL_SD_ERROR_NONE) && \
          ((SD_CARD_PROGRAMMING == cardstate) || (SD_CARD_RECEIVING == cardstate)));
  
  /* wait card operation complete */
  while(SDIO->STATUS & SDIO_STATUS_CARDBSY);
  
  if(SDIO->INTSTS & SDIO_INTSTS_RTO_BAR)
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_RTO_BAR);
    return HAL_SD_ERROR_DRTO;
  }
  if(SDIO->INTSTS & SDIO_INTSTS_DCRC)
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_DCRC);
    return HAL_SD_ERROR_DCRC;
  }
  if(SDIO->INTSTS & SDIO_INTSTS_EBE) 
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_EBE);
    return HAL_SD_ERROR_EBE;
  }
  
  SDIO_ClearITStatus(SDIO,SDIO_STATIC_FLAGS);
  return errorstatus;

}

/**
  * @brief  Reads block(s) from a specified address in a card. The Data transfer
  *         is managed in interrupt mode.
  * @note   You could also check the IT transfer process through the SD Rx
  *         interrupt event.
  * @param  pData: Pointer to the buffer that will contain the received data
  * @param  BlockAdd: Block Address from where data is to be read
  * @param  BlockSize: Block size.
  * @retval SD Error status
  */
SD_Error HAL_SD_ReadBlockIT(uint8_t *pData,uint32_t BlockAdd,uint16_t BlockSize)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  
  if(pData == NULL)
    return HAL_SD_ERROR_INVALID_PARAMETER;
  
  pReadBuf = (uint32_t *)pData;
  gTransferCnt = (BlockSize >> 2);
  /* Software should look for data error interrupts; that is, bits 7, 9, 13, and 15 of the RINTSTS register */
  SDIO_ITConfig(SDIO,SDIO_IT_RXDR | SDIO_IT_DTO | \
                SDIO_INTSTS_SBE | SDIO_INTSTS_EBE | \
                SDIO_INTSTS_DCRC | SDIO_INTSTS_DRTO_BDS | \
                SDIO_INTSTS_FRUN, ENABLE);
  SDIO_ITCmd(SDIO,ENABLE);

  /* Send CMD16 set Block Size for Card */
  SDIO_CmdInitStructure.Argument      = BlockSize;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.InitSignal    = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Response_CRC  = SDIO_RESPONSE_CRC_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);

  if(CmdResp1Error() != HAL_SD_ERROR_NONE)
  {
    return errorstatus;
  }

  SDIO_DataInitStructure.DataBlockSize = BlockSize; 
  SDIO_DataInitStructure.DataLength    = BlockSize ;
  SDIO_DataInitStructure.DataTimeOut   = SDIO_DATATIMEOUT ;
  SDIO_DataInitStructure.TransferDir   = SDIO_TRANSFERDIR_TO_SDIO;
  SDIO_DataInitStructure.TransferMode  = SDIO_TRANSFERMODE_BLOCK;
  SDIO_DataConfig(SDIO,&SDIO_DataInitStructure);
  
  /*!< Send CMD17 READ_SINGLE_BLOCK */
  SDIO_CmdInitStructure.Argument      = BlockAdd;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_READ_SINGLE_BLOCK;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.InitSignal    = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Response_CRC  = SDIO_RESPONSE_CRC_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_ENABLE;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  /* Auto Stop */
  SDIO_IsAutoStopCmd(SDIO,ENABLE);
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
  if(CmdResp1Error() != HAL_SD_ERROR_NONE)
    return errorstatus;
  
  /* Wait IT receive Complete */
  while(gReadCnt < (BlockSize >> 2))
  {
    if(HAL_SD_ERROR_NONE != gError)
      break;
  }
  
  SDIO_ITCmd(SDIO,DISABLE);
  gTransferCnt = 0;
  
  errorstatus = gError;
  
  return errorstatus;
}

/**
  * @brief  Writes block(s) to a specified address in a card. The Data transfer
  *         is managed in interrupt mode.
  * @note   You could also check the IT transfer process through the SD Tx
  *         interrupt event.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  BlockAdd: Block Address where data will be written
  * @param  BlockSize: Block size.
  * @retval HAL status
  */
SD_Error HAL_SD_WriteBlockIT(uint8_t *pData,uint32_t BlockAdd,  uint16_t BlockSize)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  
  pWriteBuf = (uint32_t *)pData;
  gWriteCnt = (BlockSize >> 2);
  /* Software should look for data error interrupts; that is, for bits 7, 9, and 15 of the RINTSTS register.  */
  SDIO_ITConfig(SDIO,SDIO_IT_TXDR | SDIO_INTSTS_EBE | SDIO_INTSTS_DCRC | SDIO_INTSTS_DRTO_BDS | SDIO_INTSTS_FRUN, ENABLE);
  SDIO_ITCmd(SDIO,ENABLE);
  
  /* wait card operation complete */
  while(SDIO->STATUS & SDIO_STATUS_CARDBSY)
  {
  }
  
  /* Set Block Size for Card */
  SDIO_CmdInitStructure.Argument      = BlockSize;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.InitSignal    = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Response_CRC  = SDIO_RESPONSE_CRC_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
    
  if(CmdResp1Error() != HAL_SD_ERROR_NONE)
    return errorstatus;
  
  SDIO_DataInitStructure.DataBlockSize = BlockSize;
  SDIO_DataInitStructure.DataLength   = BlockSize ;
  SDIO_DataInitStructure.DataTimeOut  = SDIO_DATATIMEOUT ;
  SDIO_DataInitStructure.TransferDir  = SDIO_TRANSFERDIR_TO_CARD;
  SDIO_DataInitStructure.TransferMode = SDIO_TRANSFERMODE_BLOCK;
  SDIO_DataConfig(SDIO,&SDIO_DataInitStructure);
  
  /*!< Send CMD24 WRITE_SINGLE_BLOCK */
  SDIO_CmdInitStructure.Argument      = BlockAdd;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_WRITE_SINGLE_BLOCK;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.InitSignal    = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Response_CRC  = SDIO_RESPONSE_CRC_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_IsAutoStopCmd(SDIO,DISABLE);
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
  if(CmdResp1Error() != HAL_SD_ERROR_NONE)
    return errorstatus;
  
  /* Wait IT receive Complete */
  while(gTransferCnt < gWriteCnt)
  {
    if(HAL_SD_ERROR_NONE != gError)
      break;
  }
  SDIO_ITCmd(SDIO,DISABLE);
  gTransferCnt = 0;
  
  errorstatus = gError;
  
  /* wait card operation complete */
  SD_WaitCardOperation();
  
  return errorstatus;
}

/**
  * @brief  This function handles SD card interrupt request.
  * @retval None
  */
void HAL_SD_IRQHandler()
{
  /* RX FIFO WaterMark */
  if(SDIO->INTSTS & SDIO_INTSTS_RXDR)
  {
    while(!(SDIO->STATUS & SDIO_STATUS_FIFOE))
    {
      pReadBuf[gReadCnt++] = SDIO_ReadData(SDIO);
    }
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_RXDR);
  }
  if(SDIO->INTSTS & SDIO_INTSTS_DTO)
  {
    while(!(SDIO->STATUS & SDIO_STATUS_FIFOE))
    {
      pReadBuf[gReadCnt++] = SDIO_ReadData(SDIO);
    }
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_DTO);
  }
  if(SDIO_GetITStatus(SDIO,SDIO_INTSTS_TXDR))
  {
    if(gTransferCnt < gWriteCnt)
    {
      /* write 8 bytes */
      for(uint8_t count=0; count < TX_FIFO_DEPTH; count++)
      {
        SDIO_WriteData(SDIO,pWriteBuf[gTransferCnt++]);
      }
    }
  
  SDIO_ClearITStatus(SDIO,SDIO_INTSTS_TXDR);
  }
  if(SDIO->INTSTS & SDIO_INTSTS_SBE)
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_SBE);
    gError = HAL_SD_ERROR_SBE;
  }
  if(SDIO->INTSTS & SDIO_INTSTS_EBE)
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_EBE);
    gError =  HAL_SD_ERROR_EBE;
  }
  if(SDIO->INTSTS & SDIO_INTSTS_DCRC)
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_DCRC);
    gError =  HAL_SD_ERROR_DCRC;
  }
  if(SDIO->INTSTS & SDIO_INTSTS_RTO_BAR)
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_RTO_BAR);
    gError =  HAL_SD_ERROR_DRTO;
  }
  if(SDIO->INTSTS & SDIO_INTSTS_FRUN)
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_FRUN);
    gError =  HAL_SD_ERROR_FRUN;
  }
}

/**
  * @brief  Checks for error conditions for R1 response.
  * @retval SD Card error state
  */
SD_Error CmdResp1Error(void)
{
  while(!(SDIO->INTSTS & ((1<<1)|(1<<2)|(1<<6)|(1<<8))))
  {
  }
  if(SDIO_GetITStatus(SDIO,SDIO_IT_RTO) != RESET) 
  {
    SDIO_ClearITStatus(SDIO,SDIO_IT_RTO); 
    return HAL_SD_ERROR_RTO;
  }
  if(SDIO_GetITStatus(SDIO,SDIO_IT_RCRC) != RESET) 
  {
    SDIO_ClearITStatus(SDIO,SDIO_IT_RCRC); 
    return HAL_SD_ERROR_RCRC;
  }  
  
  /* Clear all the static flags */
  SDIO_ClearITStatus(SDIO,SDIO_STATIC_FLAGS);
  
  return (SD_Error)(SDIO->RESP0 & SD_OCR_ERRORBITS);
}

/**
  * @brief  Checks for error conditions for R2 response.
  * @retval SD Card error state
  */
SD_Error CmdResp2Error(void)
{
  uint32_t status; 
  while(1)
  {
    status=SDIO->INTSTS;
    /* check Response error/Command done/Response CRC error/Response timeout */
    if(status&((1<<1)|(1<<2)|(1<<6)|(1<<8)))
      break;
  } 
  if(SDIO_GetITStatus(SDIO,SDIO_IT_RTO) != RESET) 
  { 
    SDIO_ClearITStatus(SDIO,SDIO_IT_RTO); 
    return HAL_SD_ERROR_RTO;
  } 
  if(SDIO_GetITStatus(SDIO,SDIO_IT_RCRC) != RESET) 
  { 
    SDIO_ClearITStatus(SDIO,SDIO_IT_RCRC); 
    return HAL_SD_ERROR_RCRC;
  } 
  
  /* Clear all the static flags */
  SDIO_ClearITStatus(SDIO,SDIO_STATIC_FLAGS); 
  
  return HAL_SD_ERROR_NONE; 
}

/**
  * @brief  Checks for error conditions for R3 response.
  * @retval SD Card error state
  */
SD_Error CmdResp3Error(void)
{
  while(!(SDIO->INTSTS&((1<<1)|(1<<2)|(1<<6)|(1<<8))))
  {
  }

  if(SDIO_GetITStatus(SDIO,SDIO_IT_RTO) != RESET) 
  { 
    SDIO_ClearITStatus(SDIO,SDIO_IT_RTO); 
    return HAL_SD_ERROR_RTO;
  }  
  
  /* Clear all the static flags */
  SDIO_ClearITStatus(SDIO,SDIO_STATIC_FLAGS); 
  
  return HAL_SD_ERROR_NONE; 
}

/**
  * @brief  Checks for error conditions for R6 response.
  * @param  pRCA: Pointer to the variable that will contain the SD card relative 
  *         address RCA   
  * @retval SD Card error state
  */
SD_Error CmdResp6Error(uint16_t *pRCA)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  uint32_t status; 
  uint32_t rspr0;
  while(1)
  {
    status = SDIO->INTSTS;
    
    /* check Response error/Command done/Response CRC error/Response timeout */
    if(status&((1<<1)|(1<<2)|(1<<6)|(1<<8)))
      break;
  }
  if(SDIO_GetITStatus(SDIO,SDIO_IT_RTO) != RESET) 
  { 
    SDIO_ClearITStatus(SDIO,SDIO_IT_RTO); 
    return HAL_SD_ERROR_RTO;
  } 
  if(SDIO_GetITStatus(SDIO,SDIO_IT_RCRC) != RESET) 
  { 
    SDIO_ClearITStatus(SDIO,SDIO_IT_RCRC);
    return HAL_SD_ERROR_RCRC;
  }
    
  SDIO_ClearITStatus(SDIO,SDIO_STATIC_FLAGS);
  rspr0=SDIO->RESP0; 
  
  *pRCA=(uint16_t)(rspr0>>16); 
  
  return errorstatus;
}

/**
  * @brief  Checks for error conditions for R7 response.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
SD_Error CmdResp7Error(void)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  uint32_t status;
  uint32_t timeout = SDIO_CMD0TIMEOUT;
  while(timeout--)
  {
    status = SDIO->INTSTS;
    if(status&(SDIO_INTSTS_RE|SDIO_INTSTS_RCRC|SDIO_INTSTS_RTO_BAR|SDIO_INTSTS_CD))
      break;
  }
  if((timeout == 0)||(status & SDIO_INTSTS_RTO_BAR))
  {
    errorstatus = HAL_SD_ERROR_RTO;
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_RTO_BAR); 
    return errorstatus;
  }
  if(status&SDIO_INTSTS_CD)
  {
    errorstatus = HAL_SD_ERROR_NONE;
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_CD);
  }
  SDIO_ClearITStatus(SDIO,SDIO_STATIC_FLAGS);
  return errorstatus;
}

/**
  * @brief  Enables the SDIO wide bus mode.
  * @param  WideMode: Specifies the SD card wide bus mode
  *          This parameter can be one of the following values:
  *            @arg SDIO_BUS_WIDE_1B: 1-bit data transfer
  *            @arg SDIO_BUS_WIDE_4B: 4-bit data transfer
  * @retval error state
  */
SD_Error SD_EnWideBus(uint32_t WideMode)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  uint32_t arg = 0X00;
  
  if(WideMode == SDIO_BUS_WIDE_4B)
  {
    arg = 0X02;
  }
  else
  {
    arg = 0X00;
  }
  
  /* send CMD55 */
  SDIO_CmdInitStructure.Argument     = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.CmdIndex     = SDIO_CMD_APP_CMD;
  SDIO_CmdInitStructure.DataTransfer = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait         = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM         = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
  errorstatus = CmdResp1Error();
  
  if(errorstatus!=HAL_SD_ERROR_NONE)
    return errorstatus; 
  
  /* Send ACMD6 APP_CMD with argument as 2 or 1 for wide bus mode */
  SDIO_CmdInitStructure.Argument     = arg;
  SDIO_CmdInitStructure.CmdIndex     = SDIO_CMD_APP_SD_SET_BUSWIDTH;
  SDIO_CmdInitStructure.DataTransfer = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.Response     = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait         = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM         = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
  errorstatus = CmdResp1Error();
  
  return errorstatus;

} 

/**
  * @brief  Get the status of the card.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_GetCardStatus(uint32_t *pstatus)
{
  volatile uint32_t status = 0;
  
  /* Send CMD13 */
  SDIO_CmdInitStructure.Argument      = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.CmdIndex      = SDIO_CMD_SEND_STATUS;
  SDIO_CmdInitStructure.Response      = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.Wait          = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.InitSignal    = SDIO_INITSIGNAL_DISABLE;
  SDIO_CmdInitStructure.Response_CRC  = SDIO_RESPONSE_CRC_ENABLE;
  SDIO_CmdInitStructure.DataTransfer  = SDIO_DATATRANSFER_DISABLE;
  SDIO_CmdInitStructure.CPSM          = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO,&SDIO_CmdInitStructure);
  
  do
  {
    status = SDIO->INTSTS;
  }while(!(status & (SDIO_INTSTS_RE|SDIO_INTSTS_RCRC|SDIO_INTSTS_RTO_BAR|SDIO_INTSTS_CD)));
  
  if(status & SDIO_INTSTS_RE) 
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_RE); 
    return HAL_SD_ERROR_RE;
  }
  if(status & SDIO_INTSTS_RCRC) 
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_RCRC);  
    return HAL_SD_ERROR_RCRC;
  }
  if(status & SDIO_INTSTS_RTO_BAR)  
  {
    SDIO_ClearITStatus(SDIO,SDIO_INTSTS_RTO_BAR);
    return HAL_SD_ERROR_RTO;
  }
  if(SDIO_GetRspIdx(SDIO) != SDIO_CMD_SEND_STATUS)
    return HAL_SD_ERROR_CIE;  
  
  SDIO_ClearITStatus(SDIO,SDIO_STATIC_FLAGS); 
  
  *pstatus = SDIO->RESP0; 
  
  return HAL_SD_ERROR_NONE;
}


/**
  * @brief  Wait for card operation
  * @param  None
  * @retval None.
  */
void SD_WaitCardOperation()
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  uint8_t cardstate = 0;
  uint32_t R1;

  /* wait card write complete */
  do
  {
    /* Wait till the card is in programming state */
    errorstatus = SD_GetCardStatus(&R1);
    /* get CARD CURRENT_STATE */
    cardstate = ((R1 >> 9) & 0x0000000F);
  }while ((errorstatus == HAL_SD_ERROR_NONE) && \
          ((SD_CARD_PROGRAMMING == cardstate) || (SD_CARD_RECEIVING == cardstate)));
}

SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo)
{
  SD_Error errorstatus = HAL_SD_ERROR_NONE;
  uint8_t tmp=0;
  cardinfo->CardType = (uint8_t)CardType; 
  cardinfo->RCA = (uint16_t)RCA;
  
  tmp = (uint8_t)((CSD_Tab[0]&0xFF000000)>>24);
  cardinfo->SD_csd.CSDStruct = (tmp&0xC0)>>6;	
  cardinfo->SD_csd.SysSpecVersion = (tmp&0x3C)>>2;
  cardinfo->SD_csd.Reserved1 = tmp&0x03;
  
  tmp = (uint8_t)((CSD_Tab[0]&0x00FF0000)>>16);
  cardinfo->SD_csd.TAAC = tmp;
  
  tmp = (uint8_t)((CSD_Tab[0]&0x0000FF00)>>8);
  cardinfo->SD_csd.NSAC = tmp;
  
  tmp = (uint8_t)(CSD_Tab[0]&0x000000FF);
  cardinfo->SD_csd.MaxBusClkFrec = tmp;
  
  tmp = (uint8_t)((CSD_Tab[1]&0xFF000000)>>24);
  cardinfo->SD_csd.CardComdClasses = tmp<<4;
  
  tmp = (uint8_t)((CSD_Tab[1]&0x00FF0000)>>16);
  cardinfo->SD_csd.CardComdClasses |= (tmp&0xF0)>>4;
  cardinfo->SD_csd.RdBlockLen = tmp&0x0F;
  
  tmp = (uint8_t)((CSD_Tab[1]&0x0000FF00)>>8);
  cardinfo->SD_csd.PartBlockRead = (tmp&0x80)>>7;
  cardinfo->SD_csd.WrBlockMisalign = (tmp&0x40)>>6;
  cardinfo->SD_csd.RdBlockMisalign = (tmp&0x20)>>5;
  cardinfo->SD_csd.DSRImpl = (tmp&0x10)>>4;
  cardinfo->SD_csd.Reserved2 = 0; 
  
  /* standard 1.1 card/2.0 card/mmc card */
  if((CardType==SDIO_STD_CAPACITY_SD_CARD_V1_1)|| \
     (CardType==SDIO_STD_CAPACITY_SD_CARD_V2_0)|| \
      (SDIO_MULTIMEDIA_CARD==CardType))             
  {
    cardinfo->SD_csd.DeviceSize = (tmp&0x03)<<10;
  
    tmp = (uint8_t)(CSD_Tab[1]&0x000000FF); 
    cardinfo->SD_csd.DeviceSize |= (tmp)<<2;
  
    tmp = (uint8_t)((CSD_Tab[2]&0xFF000000)>>24);
    cardinfo->SD_csd.DeviceSize |= (tmp&0xC0)>>6;
    cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp&0x38)>>3;
    cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp&0x07);
  
    tmp = (uint8_t)((CSD_Tab[2]&0x00FF0000)>>16);
    cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp&0xE0)>>5;
    cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp&0x1C)>>2;
    cardinfo->SD_csd.DeviceSizeMul = (tmp&0x03)<<1;
  
    tmp = (uint8_t)((CSD_Tab[2]&0x0000FF00)>>8);
    cardinfo->SD_csd.DeviceSizeMul |= (tmp&0x80)>>7;
    cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize+1); /* Card capacity */
    cardinfo->CardCapacity *= (1<<(cardinfo->SD_csd.DeviceSizeMul+2));
    cardinfo->CardBlockSize = 1<<(cardinfo->SD_csd.RdBlockLen); /* Block size */
    cardinfo->CardCapacity *= cardinfo->CardBlockSize;
  }
  /* High capacity card */
  else if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)
  {
    tmp = (uint8_t)(CSD_Tab[1]&0x000000FF);
    cardinfo->SD_csd.DeviceSize = (tmp&0x3F)<<16;
    tmp = (uint8_t)((CSD_Tab[2]&0xFF000000)>>24);
    cardinfo->SD_csd.DeviceSize |= (tmp<<8);
    tmp = (uint8_t)((CSD_Tab[2]&0x00FF0000)>>16);
    cardinfo->SD_csd.DeviceSize |= (tmp);
    tmp = (uint8_t)((CSD_Tab[2]&0x0000FF00)>>8);
    cardinfo->CardCapacity = (long long)(cardinfo->SD_csd.DeviceSize+1)*512*1024; /* Card capacity */
    cardinfo->CardBlockSize = 512;  /* Block size */
  }
  cardinfo->SD_csd.EraseGrSize = (tmp&0x40)>>6;
  cardinfo->SD_csd.EraseGrMul = (tmp&0x3F)<<1;
  tmp = (uint8_t)(CSD_Tab[2]&0x000000FF);
  cardinfo->SD_csd.EraseGrMul |= (tmp&0x80)>>7;
  cardinfo->SD_csd.WrProtectGrSize = (tmp&0x7F);
  tmp = (uint8_t)((CSD_Tab[3]&0xFF000000)>>24);
  cardinfo->SD_csd.WrProtectGrEnable = (tmp&0x80)>>7;
  cardinfo->SD_csd.ManDeflECC = (tmp&0x60)>>5;
  cardinfo->SD_csd.WrSpeedFact = (tmp&0x1C)>>2;
  cardinfo->SD_csd.MaxWrBlockLen = (tmp&0x03)<<2;
  tmp = (uint8_t)((CSD_Tab[3]&0x00FF0000)>>16);
  cardinfo->SD_csd.MaxWrBlockLen |= (tmp&0xC0)>>6;
  cardinfo->SD_csd.WriteBlockPaPartial = (tmp&0x20)>>5;
  cardinfo->SD_csd.Reserved3 = 0;
  cardinfo->SD_csd.ContentProtectAppli = (tmp&0x01);
  tmp = (uint8_t)((CSD_Tab[3]&0x0000FF00)>>8);
  cardinfo->SD_csd.FileFormatGrouop = (tmp&0x80)>>7;
  cardinfo->SD_csd.CopyFlag = (tmp&0x40)>>6;
  cardinfo->SD_csd.PermWrProtect = (tmp&0x20)>>5;
  cardinfo->SD_csd.TempWrProtect = (tmp&0x10)>>4;
  cardinfo->SD_csd.FileFormat = (tmp&0x0C)>>2;
  cardinfo->SD_csd.ECC = (tmp&0x03);
  tmp = (uint8_t)(CSD_Tab[3]&0x000000FF);
  cardinfo->SD_csd.CSD_CRC = (tmp&0xFE)>>1;
  cardinfo->SD_csd.Reserved4 = 1;
  tmp = (uint8_t)((CID_Tab[0]&0xFF000000)>>24);
  cardinfo->SD_cid.ManufacturerID = tmp;
  tmp = (uint8_t)((CID_Tab[0]&0x00FF0000)>>16);
  cardinfo->SD_cid.OEM_AppliID = tmp<<8;
  tmp = (uint8_t)((CID_Tab[0]&0x000000FF00)>>8);
  cardinfo->SD_cid.OEM_AppliID |= tmp;
  tmp = (uint8_t)(CID_Tab[0]&0x000000FF);
  cardinfo->SD_cid.ProdName1 = tmp<<24;
  tmp = (uint8_t)((CID_Tab[1]&0xFF000000)>>24);
  cardinfo->SD_cid.ProdName1 |= tmp<<16;
  tmp = (uint8_t)((CID_Tab[1]&0x00FF0000)>>16);
  cardinfo->SD_cid.ProdName1 |= tmp<<8;
  tmp = (uint8_t)((CID_Tab[1]&0x0000FF00)>>8);
  cardinfo->SD_cid.ProdName1 |= tmp;
  tmp = (uint8_t)(CID_Tab[1]&0x000000FF);
  cardinfo->SD_cid.ProdName2 = tmp;
  tmp = (uint8_t)((CID_Tab[2]&0xFF000000)>>24);
  cardinfo->SD_cid.ProdRev = tmp;
  tmp = (uint8_t)((CID_Tab[2]&0x00FF0000)>>16);
  cardinfo->SD_cid.ProdSN = tmp<<24;
  tmp = (uint8_t)((CID_Tab[2]&0x0000FF00)>>8);
  cardinfo->SD_cid.ProdSN |= tmp<<16;
  tmp = (uint8_t)(CID_Tab[2]&0x000000FF);
  cardinfo->SD_cid.ProdSN |= tmp<<8;
  tmp = (uint8_t)((CID_Tab[3]&0xFF000000)>>24);
  cardinfo->SD_cid.ProdSN |= tmp;
  tmp = (uint8_t)((CID_Tab[3]&0x00FF0000)>>16);
  cardinfo->SD_cid.Reserved1 |= (tmp&0xF0)>>4;
  cardinfo->SD_cid.ManufactDate = (tmp&0x0F)<<8;
  tmp = (uint8_t)((CID_Tab[3]&0x0000FF00)>>8);
  cardinfo->SD_cid.ManufactDate |= tmp;
  tmp = (uint8_t)(CID_Tab[3]&0x000000FF);
  cardinfo->SD_cid.CID_CRC = (tmp&0xFE)>>1;
  cardinfo->SD_cid.Reserved2 = 1;
  return errorstatus;
}

#endif /* HAL_SD_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
