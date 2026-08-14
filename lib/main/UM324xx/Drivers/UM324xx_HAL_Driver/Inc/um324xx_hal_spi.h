/**
  ******************************************************************************
  * @file    um324xx_hal_spi.h
  * @author  MCU Team
  * @version V1.00 
  * @date    10-February-2023  
  * @brief   SPI HAL module driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023 Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */
  
  
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __UM324XX_HAL_SPI_H__
#define __UM324XX_HAL_SPI_H__

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"
#include "um324xx.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup SPI
  * @{
  */
  
/* Exported types ------------------------------------------------------------*/
/** @defgroup SPI_Exported_Types SPI Exported Types
  * @{
  */

/**
  * @brief  SPI Configuration Structure definition
  */
typedef struct
{
    uint32_t Mode;                	/*  Master/Slave��
                                    1��Master
                                    0��Slave */
    

                        
      
    uint32_t CLKPhase;            	/*  SPI master mode ��CS0 Clock phase��
                                    1��The second clock edge is the first capture edge
                                    0��The firstclock edge is the first capture edge*/	
      
    uint32_t CLKPolarity;         	/*  SPI master mode ��CS0 Clock polarity selection��
                                    1��The serial clock stops at high level
                                    0��The serial clock stops at low level*/
      
    uint32_t FirstBit;            	/*  SPI master mode��CS0 (Frame format) 
                                    0��MSB
                                    1��LSB */															

    uint32_t BaudRatePrescaler;   	/*SPI master mode��CS0 BaudRatePrescaler clk div��highest 12M��:
                                    000�� fPCLK/2   001�� fPCLK /4  010�� fPCLK /8 011�� fPCLK /16 100�� fPCLK /32
                                    101�� fPCLK /64 110�� fPCLK /128 111�� fPCLK /256 */
   
    uint32_t SSNM	;					/*	Master SSN ctrl set
                            1��The Master pulls up the SSN every 8bit after sending, and the WAIT register controls the high level time
                            0��The Master keeps the SSN low after every 8bit, and the WAIT register controls how long the SSN stays low
                            */
     
     
    uint32_t SSN0	;					/*	In SPI Master mode, CS0 corresponds to master mode. If SSNSEN is 1, the software can control the output level of SSN through this bit
                            1��SSN output low
                            0��SSN output high */
                      
    uint32_t SSNSEN ; 				/*Master mode��soft ctrl SSN enable
                              1��Master SSN ctrl by software
                              0��Master SSN ctrl by hardware*/
    uint32_t CS1CLKPhase;            	/*  SPI master mode ��CS1 Clock phase��
                                    1��The second clock edge is the first capture edge
                                    0��The firstclock edge is the first capture edge*/	
      
    uint32_t CS1CLKPolarity;         	/*  SPI master mode ��CS1 Clock polarity selection��
                                    1��The serial clock stops at high level
                                    0��The serial clock stops at low level*/
      
    uint32_t CS1FirstBit;            	/*  SPI master mode��CS1 (Frame format) 
                                    0��MSB
                                    1��LSB */															

    uint32_t CS1BaudRatePrescaler;   	/*SPI master mode��CS1 BaudRatePrescaler clk div��highest 12M��:
                                    000�� fPCLK/2   001�� fPCLK /4  010�� fPCLK /8 011�� fPCLK /16 100�� fPCLK /32
                                    101�� fPCLK /64 110�� fPCLK /128 111�� fPCLK /256 */
   
    uint32_t CS1SSNM	;					/*	Master SSN ctrl set
                            1��The Master pulls up the SSN every 8bit after sending, and the WAIT register controls the high level time
                            0��The Master keeps the SSN low after every 8bit, and the WAIT register controls how long the SSN stays low
                            */
     
     
    uint32_t SSN1	;					/*	In SPI Master mode, CS1 corresponds to master mode. If SSNSEN is 1, the software can control the output level of SSN through this bit
                            1��SSN output low
                            0��SSN output high */
                                   

    uint32_t TRI0Mode	;				/*SPI 3 lines mode enable*/
    
    uint32_t MSPA;					/*The Master adjusts the sampling position of MISO signal, which is used to compensate PCB wiring delay in high-speed communication*/
    
    uint32_t SSPA;					/*Slave Sending Position Adjustment��Slave MISO Sending position adjustment*/
    
    uint32_t DMARxEn;					/*DMA rx enable*/
    
    uint32_t DMATxEn;					/*DMA tx enable*/
    
    uint32_t TxoAc;                  /*TXO_AC*/
    
    uint32_t DMARxLev;				
    
    uint32_t DMATxLev;
     
    uint32_t FLTEN ;                  /*slave Pin filter*/
    
    uint32_t Wait;                    /*In Master mode, at least (1+WAIT) SCK cycle wait time is added after each 8Bit is sent before the next 8Bit data is transmitted*/
    
   uint32_t  TXO;                    /*Only  TX   MODE*/
   
   uint32_t SPI_EN;

    uint32_t IrqType;                 /*irq type*/
  
} SPI_InitTypeDef;

/**
  * @brief  HAL SPI State structure definition
  */
typedef enum
{
    HAL_SPI_STATE_RESET      = 0x00U,    /*!< Peripheral not Initialized                         */
    HAL_SPI_STATE_READY      = 0x01U,    /*!< Peripheral Initialized and ready for use           */
    HAL_SPI_STATE_BUSY       = 0x02U,    /*!< an internal process is ongoing                     */
    HAL_SPI_STATE_BUSY_TX    = 0x03U,    /*!< Data Transmission process is ongoing               */
    HAL_SPI_STATE_BUSY_RX    = 0x04U,    /*!< Data Reception process is ongoing                  */
    HAL_SPI_STATE_BUSY_TX_RX = 0x05U,    /*!< Data Transmission and Reception process is ongoing */
    HAL_SPI_STATE_ERROR      = 0x06U,    /*!< SPI01 error state                                    */
    HAL_SPI_STATE_ABORT      = 0x07U     /*!< SPI01 abort is ongoing                               */
} HAL_SPI_StateTypeDef;

/** @defgroup HAL_mode_structure_definition HAL mode structure definition
  * @brief SPI0101 MODE enumeration
  * @{
  */

typedef enum
{
  HAL_SPI_MODE_NONE           = 0x00U,   /*!< No SPI01 communication on going             */
  HAL_SPI_MODE_MASTER         = 0x10U,   /*!< SPI01 communication is in Master Mode       */
  HAL_SPI_MODE_SLAVE          = 0x20U,   /*!< SPI01 communication is in Slave Mode        */

} HAL_SPI_ModeTypeDef;
/**
  * @brief  SPI handle Structure definition
  */
typedef struct __SPI_HandleTypeDef
{
    SPI_TypeDef                *Instance;      /*!< SPI registers base address               */

    SPI_InitTypeDef            Init;           /*!< SPI communication parameters             */

    uint8_t                    *pTxBuffPtr;    /*!< Pointer to SPI Tx transfer Buffer        */

    uint16_t                   TxXferSize;     /*!< SPI Tx Transfer size                     */

    __IO uint16_t              TxXferCount;    /*!< SPI Tx Transfer Counter                  */

    uint8_t                    *pRxBuffPtr;    /*!< Pointer to SPI Rx transfer Buffer        */

    uint16_t                   RxXferSize;     /*!< SPI Rx Transfer size                     */

    __IO uint16_t              RxXferCount;    /*!< SPI Rx Transfer Counter                  */

    uint32_t                   CRCSize;        /*!< SPI CRC size used for the transfer       */

    HAL_StatusTypeDef (*RxISR)(struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Rx ISR       */

    HAL_StatusTypeDef (*TxISR)(struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Tx ISR       */

    DMA_HandleTypeDef          *hdmatx;        /*!< SPI Tx DMA Handle parameters             */

    DMA_HandleTypeDef          *hdmarx;        /*!< SPI Rx DMA Handle parameters             */

  //  HAL_LockTypeDef            Lock;           /*!< Locking object                           */

    __IO HAL_SPI_StateTypeDef  state;          /*!< SPI communication state                  */

    __IO uint32_t              ErrorCode;      /*!< SPI Error code                           */

    
    HAL_LockTypeDef            Lock;           /*!< I2C locking object                        */
    
    HAL_SPI_ModeTypeDef      Mode;
  #if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
   void (* MspInitCallback)(struct __SPI_HandleTypeDef *hspi);
  #endif 
} SPI_HandleTypeDef;
  
  

  
/**
  * @}
  */


/* Exported constants --------------------------------------------------------*/
/** @defgroup SPI_Exported_Constants SPI Exported Constants
  * @{
  */

/** @defgroup SPI_reg_macro SPI reg macro
* @brief SPI reg macro
* @{
*/

#define SPI_IE_FIELDS  	 (uint32_t)(SPI_IE_RXBFIE  | SPI_IE_TXBEIE |  \
									SPI_IE_TXCOLIE | SPI_IE_RXCOLIE|SPI_IE_SERRIE|SPI_IE_MERRIE | \
									 SPI_IE_SSNPOSIE|SPI_IE_SSNNEGIE  \
                                   )


#define SPI_CR_FIELDS  (uint32_t)(SPI_CR_SPIEN | SPI_CR_SSNSEN | SPI_CR_WAIT | SPI_CR_MM |\
								  SPI_CR_SSPA | SPI_CR_MSPA | SPI_CR_TXO | SPI_CR_TXO_AC | SPI_CR_SSNM |\
								  SPI_CR_FLTEN | SPI_CR_DMA_RX_EN | SPI_CR_DMA_TX_EN   \
								  )


#define SPI_CS0_FIELDS  (uint32_t)(SPI_CS0_CPHA0 | SPI_CS0_CPOL0 | SPI_CS0_LSBF0 | \
						  SPI_CS0_BAUD0 | SPI_CS0_SSN0 )
                          
#define SPI_CS1_FIELDS  (uint32_t)( SPI_CS1_CPHA1 | SPI_CS1_CPOL1 | \
						  SPI_CS1_LSBF1 | SPI_CS1_BAUD1| SPI_CS1_SSN1 )


#define SPI_CR_MODE_Master		(0x1UL << SPI_CR_MM_Pos) 
#define SPI_CR_MODE_Slave		(0x0UL << SPI_CR_MM_Pos) 



#define SPI_CS0_LSBF0_MSB		(0x0UL << SPI_CS0_LSBF0_Pos)
#define SPI_CS0_LSBF0_LSB		(0x1UL << SPI_CS0_LSBF0_Pos)
#define SPI_CS1_LSBF1_MSB		(0x0UL << SPI_CS1_LSBF1_Pos)
#define SPI_CS1_LSBF1_LSB		(0x1UL << SPI_CS1_LSBF1_Pos)

#define SPI_CS0_CPHA0_ONE		(0x0UL << SPI_CS0_CPHA0_Pos)
#define SPI_CS0_CPHA0_TWO		(0x1UL << SPI_CS0_CPHA0_Pos)
#define SPI_CS1_CPHA1_ONE		(0x0UL << SPI_CS1_CPHA1_Pos)
#define SPI_CS1_CPHA1_TWO		(0x1UL << SPI_CS1_CPHA1_Pos)

#define SPI_CS0_CPOL0_LOW		(0x0UL << SPI_CS0_CPOL0_Pos)
#define SPI_CS0_CPOL0_HIGH		(0x1UL << SPI_CS0_CPOL0_Pos)
#define SPI_CS1_CPOL1_LOW		(0x0UL << SPI_CS1_CPOL1_Pos)
#define SPI_CS1_CPOL1_HIGH		(0x1UL << SPI_CS1_CPOL1_Pos)



#define SPI_CS0_BAUD0_2DIV		(0x0UL << SPI_CS0_BAUD0_Pos)
#define SPI_CS0_BAUD0_4DIV		(0x1UL << SPI_CS0_BAUD0_Pos)
#define SPI_CS0_BAUD0_8DIV		(0x2UL << SPI_CS0_BAUD0_Pos)
#define SPI_CS0_BAUD0_16DIV		(0x3UL << SPI_CS0_BAUD0_Pos)
#define SPI_CS0_BAUD0_32DIV		(0x4UL << SPI_CS0_BAUD0_Pos)
#define SPI_CS0_BAUD0_64DIV		(0x5UL << SPI_CS0_BAUD0_Pos)
#define SPI_CS0_BAUD0_128DIV	(0x6UL << SPI_CS0_BAUD0_Pos)
#define SPI_CS0_BAUD0_256DIV	(0x7UL << SPI_CS0_BAUD0_Pos)
#define SPI_CS1_BAUD1_2DIV		(0x0UL << SPI_CS1_BAUD1_Pos)
#define SPI_CS1_BAUD1_4DIV		(0x1UL << SPI_CS1_BAUD1_Pos)
#define SPI_CS1_BAUD1_8DIV		(0x2UL << SPI_CS1_BAUD1_Pos)
#define SPI_CS1_BAUD1_16DIV		(0x3UL << SPI_CS1_BAUD1_Pos)
#define SPI_CS1_BAUD1_32DIV		(0x4UL << SPI_CS1_BAUD1_Pos)
#define SPI_CS1_BAUD1_64DIV		(0x5UL << SPI_CS1_BAUD1_Pos)
#define SPI_CS1_BAUD1_128DIV	(0x6UL << SPI_CS1_BAUD1_Pos)
#define SPI_CS1_BAUD1_256DIV	(0x7UL << SPI_CS1_BAUD1_Pos)





#define SPI_CS0_SSN0_HIGH		(0x0UL << SPI_CS0_SSN0_Pos)
#define SPI_CS0_SSN0_LOW		(0x1UL << SPI_CS0_SSN0_Pos)
#define SPI_CS1_SSN1_HIGH		(0x0UL << SPI_CS1_SSN1_Pos)
#define SPI_CS1_SSN1_LOW		(0x1UL << SPI_CS1_SSN1_Pos)


#define SPI_CR_SSNSDISEN                  (0x0UL << SPI_CR_SSNSEN_Pos)

/**
  * @}
  */



  
/** @defgroup SPI01_mode SPI01 mode
* @brief SPI01 mode code
* @{
*/
#define SPI_MODE_MASTER    SPI_CR_MODE_Master	
#define SPI_MODE_SLAVE     SPI_CR_MODE_Slave	

/**
  * @}
  */
  
/** @defgroup SPI01_cpha SPI01 cpha
* @brief SPI01 cpha code
* @{
*/
  
#define SPI_CPHA_FIRST_EDGE     SPI_CS0_CPHA0_ONE		  //The first clock edge is the first capture edge
#define SPI_CPHA_SECOND_EDGE    SPI_CS0_CPHA0_TWO		  //Thesecond clock edge is the first capture edge

/**
  * @}
  */
  
/** @defgroup SPI_cphol SPI01 cphol
* @brief SPI01 cphol code
* @{
*/ 

#define SPI_CPHOL_LOW    SPI_CS0_CPOL0_LOW		  //The serial clock stops at low level
#define SPI_CPHOL_HIGH   SPI_CS0_CPOL0_HIGH		  //The serial clock stops at high level

/**
  * @}
  */
  
  
/** @defgroup SPI01_cs_ctrl SPI01 cs ctrl
* @brief SPI01 cs ctrl code
* @{
*/ 
 
#define SPI_CS_SOFT_CTRL   SPI_CR_SSNSEN       //CS is controlled by software
#define SPI_CS_HW_CTRL     SPI_CR_SSNSDISEN    //CS is controlled by hardware
  
/**
  * @}
  */
  
/** @defgroup SPI01_MSB_LSB_transmission SPI01 MSB LSB Transmission
* @brief SPI01 MSB LSB Transmission code
* @{
*/ 

#define SPI_FIRSTBIT_MSB    SPI_CS0_LSBF0_MSB		
#define SPI_FIRSTBIT_LSB    SPI_CS0_LSBF0_LSB	

/**
  * @}
  */
  
  
/** @defgroup SPI01_BaudRate_Prescaler SPI01 BaudRate Prescaler
* @brief SPI01 BaudRate Prescaler code
* @{
*/   

#define SPI_BAUDRATEPRESCALER_2         SPI_CS0_BAUD0_2DIV
#define SPI_BAUDRATEPRESCALER_4         SPI_CS0_BAUD0_4DIV
#define SPI_BAUDRATEPRESCALER_8         SPI_CS0_BAUD0_8DIV
#define SPI_BAUDRATEPRESCALER_16        SPI_CS0_BAUD0_16DIV
#define SPI_BAUDRATEPRESCALER_32        SPI_CS0_BAUD0_32DIV
#define SPI_BAUDRATEPRESCALER_64        SPI_CS0_BAUD0_64DIV
#define SPI_BAUDRATEPRESCALER_128       SPI_CS0_BAUD0_128DIV
#define SPI_BAUDRATEPRESCALER_256       SPI_CS0_BAUD0_256DIV

/**
  * @}
  */
  
  


/** @defgroup SPI_wait SPI01 wait
* @brief SPI01 wait code
* @{
*/ 
#define SPI_WAIT_CYCLE_0   SPI_CR_WAIT_0 		
#define SPI_WAIT_CYCLE_1   SPI_CR_WAIT_1 		
#define SPI_WAIT_CYCLE_2   SPI_CR_WAIT_2 		
#define SPI_WAIT_CYCLE_3   SPI_CR_WAIT_3 		
/**
  * @}
  */
  
  
/** @defgroup SPI01_flag SPI01 flag
* @brief SPI01 flag code
* @{
*/
#define SPI_FLAG_RXBF                   SPI_IF_RXBF    /* RX Buffer no empty     */
#define SPI_FLAG_TXBE                   SPI_IF_TXBE    /* TX Buffer Empty    */
#define SPI_FLAG_IDLE                   SPI_IF_IDLE    /* SPI010 idle��read only      */
#define SPI_FLAG_TXCOL                	SPI_IF_TXCOL   /* tx fifo overflow��*/
#define SPI_FLAG_RXCOL                  SPI_IF_RXCOL   /* rx fifo overflow��*/
#define SPI_FLAG_SERR                   SPI_IF_SERR    /* Slave Error  */
#define SPI_FLAG_MERR                   SPI_IF_MERR    /* Master Error  */
#define SPI_FLAG_TNF                  	SPI_IF_TXNF     /* Spi Tx Fifo Not Full */
#define SPI_FLAG_RNF                  	SPI_IF_RXF     /* Spi Rx Fifo Full */
#define SPI_FLAG_SSNPOS 			  	SPI_IF_SSNPOSIF  /* SSN Posedge  */
#define SPI_FLAG_SSNNEG					SPI_IF_SSNNEGIF  /* SSN Negedge  */
#define SPI_FLAG_MASK					(SPI_IF_RXBF | SPI_IF_TXBE | SPI_IF_IDLE | SPI_IF_TXCOL\
										| SPI_IF_RXCOL | SPI_IF_SERR | SPI_IF_MERR | SPI_IF_TXNF\
										| SPI_IF_RXF | SPI_IF_SSNPOSIF | SPI_IF_SSNNEGIF)

/**
  * @}
  */
  

/** @defgroup SPI01_irq SPI01 irq
* @brief SPI01 irq code
* @{
*/
#define SPI_IRQ_RXBF                    SPI_IE_RXBFIE    /* RX Buffer no empty     */
#define SPI_IRQ_TXBE                    SPI_IE_TXBEIE    /* TX Buffer Empty   */
#define SPI_IRQ_IDLE                    SPI_IE_IDLE    /* SPI010 idle��read only      */
#define SPI_IRQ_TXCOL                	SPI_IE_TXCOLIE     /* tx fifo overflow��*/
#define SPI_IRQ_RXCOL                   SPI_IE_RXCOLIE   /* rx fifo overflow��*/
#define SPI_IRQ_SERR                    SPI_IE_SERRIE    /* Slave Error  */
#define SPI_IRQ_MERR                    SPI_IE_MERRIE    /* Master Error  */
#define SPI_IRQ_TNF                  	SPI_IE_TXNFIE     /* Spi Tx Fifo Not Full */
#define SPI_IRQ_RNF                  	SPI_IE_RXFIE     /* Spi Rx Fifo Full */
#define SPI_IRQ_SSNPOS 			  		SPI_IE_SSNPOSIE  /* SSN Posedge  */
#define SPI_IRQ_SSNNEG					SPI_IE_SSNNEGIE  /* SSN Negedge  */
#define SPI_IRQ_NONE				    0x00000000U         //none irq
#define SPI_IRQ_ENABLE				    0x80000000U         //enable irq
#define SPI_IRQ_MASK					(SPI_IRQ_RXBF | SPI_IRQ_TXBE | SPI_IRQ_IDLE | SPI_IRQ_TXCOL\
										| SPI_IRQ_RXCOL | SPI_IRQ_SERR | SPI_IRQ_MERR | SPI_IRQ_TNF\
										| SPI_IRQ_RNF | SPI_IRQ_SSNPOS | SPI_IRQ_SSNNEG)


/**
  * @}
  */
  

/** @defgroup SPI01_clear SPI01 clear
* @brief SPI01 clear
* @{
*/

#define SPI_CLEAR_SERRC                  SPI_OPCR_SERRC   /* Slave Error Clear��write 1 clear SPI_IF.SERR  */
#define SPI_CLEAR_MERRC                  SPI_OPCR_MERRC   /* Master Error Clear��write 1 clear SPI01_IF. MERR  */
#define SPI_CLEAR_RXBFC                  SPI_OPCR_RXBFC   /* Receive Buffer Clear��write 1 clear rxfifo��  */
#define SPI_CLEAR_TXBFC                	 SPI_OPCR_TXBFC   /* Transmit Buffer Clear��write 1 clear tx fifo*/
#define SPI_READ_SSNSTAT                 SPI_OPCR_SSNSTAT


#define          STATE_FINISH              1
#define          STATE_UNFINISH            0

#define        TRI_EN             (1<<SPI_CR_TRI_EN_Pos)  
#define        TRI_DIS             (0<<SPI_CR_TRI_EN_Pos) 


/** @defgroup SPI_Error_Code SPI Error Code
  * @{
  */
#define HAL_SPI_ERROR_NONE              (0x00000000U)   /*!< No error                               */
#define HAL_SPI_ERROR_DMA               (0x00000010U)   /*!< DMA transfer error                     */
#define HAL_SPI_ERROR_FLAG              (0x00000020U)   /*!< Error on RXOERR/UNDERRUN Flag          */
#define HAL_SPI_ERROR_ABORT             (0x00000040U)   /*!< Error during SPI Abort procedure       */
/**
  * @}
  */
  
/**
  * @}
  */
  
/* Exported macro ------------------------------------------------------------*/
/** @defgroup SPI01_Exported_Macros SPI01 Exported Macros
  * @{
  */



/** @brief  Enable the SPI peripheral.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1 to select the SPI peripheral.
  * @retval None
  */
#define HAL_SPIEN_ENABLE(__HANDLE__)     					SET_BIT((__HANDLE__)->Instance->CR,SPI_CR_SPIEN)

/** @brief  Disable the SPI peripheral.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1 to select the SPI peripheral.
  * @retval None
  */
#define HAL_SPIEN_DISABLE(__HANDLE__)     				CLEAR_BIT((__HANDLE__)->Instance->CR,SPI_CR_SPIEN)

/** @brief  Enable the specified SPI interrupts.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the interrupt source to enable.
  *         This parameter can be one of the following values:           
  *            @arg SPI_IRQ_RXBF              RX Buffer no empty     
  *            @arg SPI_IRQ_TXBE              TX Buffer Empty   
  *            @arg SPI_IRQ_IDLE              SPI01 idle read only     
  *            @arg SPI_IRQ_TXCOL             tx fifo overflow
  *            @arg SPI_IRQ_RXCOL             rx fifo overflow
  *            @arg SPI_IRQ_SERR              Slave Error  
  *            @arg SPI_IRQ_MERR              Master Error 
  *            @arg SPI_IRQ_TNF               Spi Tx Fifo Not Full 
  *            @arg SPI_IRQ_RNF               Spi Rx Fifo Full 
  *            @arg SPI_IRQ_SSNPOS 			  		SSN Posedge  
  *            @arg SPI_IRQ_SSNNEG					  SSN Negedge           
  * @retval None
  */
#define HAL_SPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)  	SET_BIT((__HANDLE__)->Instance->IE, (__INTERRUPT__))

/** @brief  Disable the specified SPI interrupts.
  * @param  __HANDLE__ specifies the SPI handle.
  *         This parameter can be SPIx where x: 0, 1 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IRQ_RXBF              RX Buffer no empty     
  *            @arg SPI_IRQ_TXBE              TX Buffer Empty   
  *            @arg SPI_IRQ_IDLE              SPI01 idle read only     
  *            @arg SPI_IRQ_TXCOL             tx fifo overflow
  *            @arg SPI_IRQ_RXCOL             rx fifo overflow
  *            @arg SPI_IRQ_SERR              Slave Error  
  *            @arg SPI_IRQ_MERR              Master Error 
  *            @arg SPI_IRQ_TNF               Spi Tx Fifo Not Full 
  *            @arg SPI_IRQ_RNF               Spi Rx Fifo Full 
  *            @arg SPI_IRQ_SSNPOS 			  		SSN Posedge  
  *            @arg SPI_IRQ_SSNNEG					  SSN Negedge                
  * @retval None
  */
#define HAL_SPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)  	CLEAR_BIT((__HANDLE__)->Instance->IE, (__INTERRUPT__))  
  
/** @brief  Check whether the specified SPI flag is set or not.
  * @note   If an interrupt is not enabled, the corresponding interrupt flag in this register will not be set.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1 to select the SPI peripheral.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXBF             RX Buffer no empty 
  *            @arg SPI_FLAG_TXBE             TX Buffer Empty    
  *            @arg SPI_FLAG_IDLE             SPI010 idle��read only    
  *            @arg SPI_FLAG_TXCOL            tx fifo overflow
  *            @arg SPI_FLAG_RXCOL            rx fifo overflow
  *            @arg SPI_FLAG_SERR             Slave Error  
  *            @arg SPI_FLAG_MERR             Master Error
  *            @arg SPI_FLAG_TNF              Spi Tx Fifo Not Full 
  *            @arg SPI_FLAG_RNF              Spi Rx Fifo Full 
  *            @arg SPI_FLAG_SSNPOS 			  	SSN Posedge  
  *            @arg SPI_FLAG_SSNNEG					  SSN Negedge                   
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define HAL_SPI_GET_FLAG(__HANDLE__, __FLAG__) 			((((__HANDLE__)->Instance->IF) & (__FLAG__)) == (__FLAG__))  

/** @brief  Clear the SPI pending flag.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1 to select the SPI peripheral.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXBF             RX Buffer no empty 
  *            @arg SPI_FLAG_TXBE             TX Buffer Empty    
  *            @arg SPI_FLAG_IDLE             SPI010 idle��read only    
  *            @arg SPI_FLAG_TXCOL            tx fifo overflow
  *            @arg SPI_FLAG_RXCOL            rx fifo overflow
  *            @arg SPI_FLAG_SERR             Slave Error  
  *            @arg SPI_FLAG_MERR             Master Error
  *            @arg SPI_FLAG_TNF              Spi Tx Fifo Not Full 
  *            @arg SPI_FLAG_RNF              Spi Rx Fifo Full 
  *            @arg SPI_FLAG_SSNPOS 			  	SSN Posedge  
  *            @arg SPI_FLAG_SSNNEG					  SSN Negedge                       
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define HAL_SPI_CLEAR_CRCERRFLAG(__HANDLE__, __FLAG__) 	((__HANDLE__)->Instance->IF |= (__FLAG__))  
  
/**
  * @brief  open TXO
  * 
  */
#define HAL_SPI_TXO_OPEN(__HANDLE__)     	SET_BIT((__HANDLE__)->Instance->CR,SPI_CR_TXO)

/**
  * @brief close TXO
  * 
  */
#define HAL_SPI_TXO_CLOSE(__HANDLE__)     CLEAR_BIT((__HANDLE__)->Instance->CR,SPI_CR_TXO)

/**
  * @brief SPI01 cs low
  * 
  */
#define HAL_SPI_CS_Enable(__HANDLE__) 			SET_BIT((__HANDLE__)->Instance->CS0,SPI_CS0_SSN0)

/**
  * @brief SPI cs high
  * 
  */
#define HAL_SPI_CS_Disable(__HANDLE__) 			CLEAR_BIT((__HANDLE__)->Instance->CS0,SPI_CS0_SSN0)

/**
  * @brief SPI01 cs1 low
  * 
  */
#define HAL_SPI_CS1_Enable(__HANDLE__) 			SET_BIT((__HANDLE__)->Instance->CS1,SPI_CS1_SSN1)

/**
  * @brief SPI cs1 high
  * 
  */
#define HAL_SPI_CS1_Disable(__HANDLE__) 			CLEAR_BIT((__HANDLE__)->Instance->CS1,SPI_CS1_SSN1)


/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __INTSTAT__  copy of SPI INTSTAT register.
  * @param  __FLAG__     specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXBF             RX Buffer no empty 
  *            @arg SPI_FLAG_TXBE             TX Buffer Empty    
  *            @arg SPI_FLAG_IDLE             SPI010 idle��read only    
  *            @arg SPI_FLAG_TXCOL            tx fifo overflow
  *            @arg SPI_FLAG_RXCOL            rx fifo overflow
  *            @arg SPI_FLAG_SERR             Slave Error  
  *            @arg SPI_FLAG_MERR             Master Error
  *            @arg SPI_FLAG_TNF              Spi Tx Fifo Not Full 
  *            @arg SPI_FLAG_RNF              Spi Rx Fifo Full 
  *            @arg SPI_FLAG_SSNPOS 			  	SSN Posedge  
  *            @arg SPI_FLAG_SSNNEG					  SSN Negedge  
  * @retval SET or RESET.
  */
#define SPI_CHECK_FLAG(__SR__, __FLAG__) ((((__SR__) & ((__FLAG__) & SPI_FLAG_MASK)) == \
                                          ((__FLAG__) & SPI_FLAG_MASK)) ? 1 : 0)

/** @brief  Check whether the specified SPI Interrupt is set or not.
  * @param  __INTEN__  copy of SPI INTEN register.
  * @param  __INTERRUPT__ specifies the SPI interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_IRQ_RXBF              RX Buffer no empty     
  *            @arg SPI_IRQ_TXBE              TX Buffer Empty   
  *            @arg SPI_IRQ_IDLE              SPI01 idle read only     
  *            @arg SPI_IRQ_TXCOL             tx fifo overflow
  *            @arg SPI_IRQ_RXCOL             rx fifo overflow
  *            @arg SPI_IRQ_SERR              Slave Error  
  *            @arg SPI_IRQ_MERR              Master Error 
  *            @arg SPI_IRQ_TNF               Spi Tx Fifo Not Full 
  *            @arg SPI_IRQ_RNF               Spi Rx Fifo Full 
  *            @arg SPI_IRQ_SSNPOS 			  		SSN Posedge  
  *            @arg SPI_IRQ_SSNNEG					  SSN Negedge   
  * @retval SET or RESET.
  */
#define SPI_CHECK_IT_SOURCE(__IE__, __INTERRUPT__) ((((__IE__) & (__INTERRUPT__)) == \
                                                     (__INTERRUPT__)) ? 1 : 0)
													 
/**
  * @brief SPI clear rx fifo
  * 
  */
#define SPI_Clear_RxBuffer(__HANDLE__)           	SET_BIT((__HANDLE__)->Instance->OPCR,SPI_CLEAR_RXBFC)

/**
  * @brief SPI clear tx fifo
  * 
  */
#define SPI_Clear_TxBuffer(__HANDLE__)           	SET_BIT((__HANDLE__)->Instance->OPCR,SPI_CLEAR_TXBFC)




/**
  * @}
  */  
  
  
  
/* Private macros ------------------------------------------------------------*/
/** @addtogroup SPI_Private_Macros SPI Private Macros
  * @{
  */
  
/**
  * @}
  */
  

/* Exported functions --------------------------------------------------------*/ 
/** @addtogroup SPI_Exported_Functions SPI Exported Functions
  * @{
  */

/** @addtogroup SPI_Exported_Functions_Group1 Initialization/de-initialization functions 
 *  @brief    Initialization and Configuration functions
 * @{
 */
     
/* Initialization and de-initialization functions *****************************/
/**
  * @brief 
  * 
  * @param hspi  A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong 
  *     @retval  HAL_ERROR something wrong
  * 
  */
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);



/**
  * @}
  */

/** @addtogroup SPI_Exported_Functions_Group2 IO operation functions 
 * @{
 */
   
/* IO operation functions *****************************************************/

/**
  * @brief    Send an amount of data in blocking mode.
  * 
  * @param hspi       A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @param pData      Pointer to the address to send data
  * @param Size       The amount of data to be sent
  * @param Timeout    Timeout period for waiting for sending to complete
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong 
  *     @retval  HAL_ERROR something wrong
  *     @retval  HAL_TIMEOUT transmit timeout
  * 
  */
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);


/**
  * @brief    Receive an amount of data in blocking mode.
  * 
  * @param hspi      A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @param pData     A pointer to the address to receive data
  * @param Size      The amount of data to be received
  * @param Timeout   Wait to receive timeout
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong 
  *     @retval  HAL_ERROR something wrong
  *     @retval  HAL_TIMEOUT receive timeout
  */
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
  * 
  * @param hspi   hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param pTxData  pointer to transmission data buffer
  * @param pRxData  pointer to reception data buffer
  * @param Size     amount of data to be sent and received
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong 
  *     @retval  HAL_ERROR something wrong
  * 
  */
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,uint32_t Timeout);

/**
  * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
  * 
  * @param hspi  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param pData  pointer to data buffer
  * @param Size   amount of data to be sent
  * @param recv_callback  transmit irq callback
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong 
  *     @retval  HAL_ERROR something wrong
  * 
  */
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size,HAL_StatusTypeDef (*recv_callback)());

/**
  * @brief       Receive an amount of data in non-blocking mode with Interrupt.
  * 
  * @param hspi  pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param pData  pointer to data buffer
  * @param Size  amount of data to be sent
  * @param recv_callback  receive irq callback
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong 
  *     @retval  HAL_ERROR something wrong
  * 
  */
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size , HAL_StatusTypeDef (*recv_callback)());

/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
  * 
  * @param hspi   hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param pTxData  pointer to transmission data buffer
  * @param pRxData  pointer to reception data buffer
  * @param Size     amount of data to be sent and received
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong 
  *     @retval  HAL_ERROR something wrong
  * 
  */
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,uint16_t Size); 

/**
  * @brief  SPI irq function
  * 
  * @param hspi  A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong 
  *     @retval  HAL_ERROR something wrong
  * 
  */
  
  
/**
  * @brief    Send an amount of data in blocking mode.
  * 
  * @param hspi       A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @param pData      Pointer to the address to send data
  * @param Size       The amount of data to be sent
  * @param Timeout    Timeout period for waiting for sending to complete
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong 
  *     @retval  HAL_ERROR something wrong
  *     @retval  HAL_TIMEOUT transmit timeout
  * 
  */
HAL_StatusTypeDef HAL_SPI_Master_Transmit_DC(SPI_HandleTypeDef *hspi, uint32_t *pData, uint16_t Size, uint32_t Timeout);

/**
  * @brief    Receive an amount of data in blocking mode.
  * 
  * @param hspi      A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @param pData     A pointer to the address to receive data
  * @param Size      The amount of data to be received
  * @param Timeout   Wait to receive timeout
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong 
  *     @retval  HAL_ERROR something wrong
  *     @retval  HAL_TIMEOUT receive timeout
  */
HAL_StatusTypeDef HAL_SPI_Master_Receive_DC(SPI_HandleTypeDef *hspi, uint32_t *pData, uint16_t Size, uint32_t Timeout);

/**
  * @brief  Tx and Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
//__weak void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//    /* Prevent unused argument(s) compilation warning */
//    UNUSED(hspi);
//    
//    /* NOTE : This function should not be modified, when the callback is needed,
//                the HAL_SPI_TxRxCpltCallback should be implemented in the user file
//    */
//}


/**
  * @brief    Send an amount of data in blocking mode.
  * 
  * @param hspi       A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @param pData      Pointer to the address to send data
  * @param Size       The amount of data to be sent
  * @param Timeout    Timeout period for waiting for sending to complete
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong 
  *     @retval  HAL_ERROR something wrong
  *     @retval  HAL_TIMEOUT transmit timeout
  * 
  */
HAL_StatusTypeDef HAL_SPI_Transmit_32Bit(SPI_HandleTypeDef *hspi, uint32_t *pData, uint16_t Size, uint32_t Timeout);


/**
  * @brief    Receive an amount of data in blocking mode.
  * 
  * @param hspi      A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @param pData     A pointer to the address to receive data
  * @param Size      The amount of data to be received
  * @param Timeout   Wait to receive timeout
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong 
  *     @retval  HAL_ERROR something wrong
  *     @retval  HAL_TIMEOUT receive timeout
  */
HAL_StatusTypeDef HAL_SPI_Receive_32Bit(SPI_HandleTypeDef *hspi, uint32_t *pData, uint16_t Size, uint32_t Timeout);

void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);  
void HAL_SPI_MspInit(SPI_HandleTypeDef *spi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);
/**
  * @}
  */








/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */  
  
#ifdef __cplusplus
}
#endif

#endif /* __UM32X42X_HAL_SPI_H__ */  
  
  







