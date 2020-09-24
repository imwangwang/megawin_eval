/**
  ******************************************************************************
 *
 * @file        main.c
 *
 * @brief       IIC
 *
 * @par         Project
 *              MG32x02z
 *							此demo旨在利用IIC做IIC主机，并使用DMA与24C64进行发送和接收。
 *							注意：
 *							
 *							
 *							
 * @version     V1.00
 * @date        2020/04/15
 * @author      Megawin Software Center
 * @copyright   Copyright (c) 2017 MegaWin Technology Co., Ltd.
 *              All rights reserved.
 *
 ******************************************************************************
 * @par Disclaimer
 * The Demo software is provided "AS IS" without any warranty, either
 * expressed or implied, including, but not limited to, the implied warranties
 * of merchantability and fitness for a particular purpose. The author will
 * not be liable for any special, incidental, consequential or indirect
 * damages due to loss of data or any other reason.
 * These statements agree with the world wide and local dictated laws about
 * authorship and violence against these laws.
 *******************************************************************************
 */


#include "MG32x02z_DRV.H"

#include <stdio.h>

#include "Type.h"
#include "UserDefine.h"


#define _CK_IHRCO_		0
#define _CK_XOSC_		1
#define _CK_SEL_		_CK_IHRCO_

#define SYS_CLOCK		48.000				// sysclk =48MHz
#define PRINTF_URTX		URT0				// URT0 for printf

// URT0 115200bps@48MHz
#define URT0_PSR_VALUE		0
#define URT0_RLR_VALUE		12
#define URT0_OS_NUM_VALUE	31
#define I2C_PR_FREQUENCY	48000000
#define I2C0_CLOCK_FREQ		400000

// DEMO Led define
#define IO_LED_G_0			PD8
#define IO_LED_R				PD9
#define IO_LED_G_1			PD10

#define SLAVE_ADDRESS_A0		 0xA0
#define DATA_LENGTH          32					//因为24C64的内部Buffer为32bit，因此单次写入数据长度超过32的话会造成写出错
u8 SendBuf[DATA_LENGTH];
u8 RcvBuf[DATA_LENGTH];

/*
*************************************************************************************
* Interrupt Handler
*
*************************************************************************************
*/

/**
 *******************************************************************************
 * @brief       系统Tick 中断处理入口			    
 * @details     系统Tick 中断服务函数
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void SysTick_Handler(void);
 * @endcode
 *******************************************************************************
 */
void SysTick_Handler(void)
{
    //to do......
	IncTick();
}

/*
*************************************************************************************
*/ 

/*
*************************************************************************************
*  Init device
*
*************************************************************************************
*/

/**
 *******************************************************************************
 * @brief       系统时钟初始化			    
 * @details     系统时钟初始化为系统时钟48MHz
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void CSC_Init (void);
 * @endcode
 *******************************************************************************
 */
void CSC_Init (void)
{
	CSC_PLL_TyprDef CSC_PLL_CFG;
    
	
    UnProtectModuleReg(MEMprotect);     	// Setting flash wait state
    MEM_SetFlashWaitState(MEM_FWAIT_ONE);	// 50MHz> Sysclk >=25MHz
    ProtectModuleReg(MEMprotect);

    UnProtectModuleReg(CSCprotect);
	CSC_CK_APB_Divider_Select(APB_DIV_1);	// Modify CK_APB divider	APB=CK_MAIN/1
	CSC_CK_AHB_Divider_Select(AHB_DIV_1);	// Modify CK_AHB divider	AHB=APB/1

#if (_CK_SEL_==_CK_IHRCO_)	
	/* CK_HS selection */
	CSC_IHRCO_Select(IHRCO_12MHz);			// IHRCO Sel 12MHz
	CSC_IHRCO_Cmd(ENABLE);
	while(CSC_GetSingleFlagStatus(CSC_IHRCOF) == DRV_Normal);
	CSC_ClearFlag(CSC_IHRCOF);
	CSC_CK_HS_Select(HS_CK_IHRCO);			// CK_HS select IHRCO


	/* PLL */
	/**********************************************************/
	CSC_PLL_CFG.InputDivider=PLLI_DIV_2;	// 12M/2=6M
	CSC_PLL_CFG.Multiplication=PLLIx16;		// 6M*16=96M
	CSC_PLL_CFG.OutputDivider=PLLO_DIV_2;	// PLLO=96M/2=48M
	CSC_PLL_Config(&CSC_PLL_CFG);
	CSC_PLL_Cmd(ENABLE);
	while(CSC_GetSingleFlagStatus(CSC_PLLF) == DRV_Normal);
	CSC_ClearFlag(CSC_PLLF);
	/**********************************************************/

	
	/* CK_MAIN */ 
	CSC_CK_MAIN_Select(MAIN_CK_PLLO);	
#else
	
	/* CK_HS selection */
	CSC_XOSCGain_Select(Gain_Medium);		
	CSC_PeriphOnModeClock_Config(CSC_ON_PortC, ENABLE);
	CSC_XOSC_Cmd(ENABLE);					// Enable XOSC
	while(CSC_GetSingleFlagStatus(CSC_XOSCF) == DRV_Normal);
	CSC_ClearFlag(CSC_XOSCF);
	
	CSC_MissingClockDetectionDuration_Select(MCD_Duration_125us);
	CSC_MissingClockDetection_Cmd(ENABLE);

	CSC_CK_HS_Select(HS_CK_XOSC);
	
	/* PLL */
	/**********************************************************/
	CSC_PLL_CFG.InputDivider=PLLI_DIV_2;	// 12M/2=65M
	CSC_PLL_CFG.Multiplication=PLLIx16;		// 6M*16=96M
	CSC_PLL_CFG.OutputDivider=PLLO_DIV_2;	// PLLO=96M/2=48M
	CSC_PLL_Config(&CSC_PLL_CFG);
	CSC_PLL_Cmd(ENABLE);
	while(CSC_GetSingleFlagStatus(CSC_PLLF) == DRV_Normal);
	CSC_ClearFlag(CSC_PLLF);
	/**********************************************************/

	
	/* CK_MAIN */ 
	CSC_CK_MAIN_Select(MAIN_CK_PLLO);	
#endif

    ProtectModuleReg(CSCprotect);
    
}

/**
 *******************************************************************************
 * @brief       系统滴答时钟初始化			    
 * @details     系统滴答时钟初始化，用于Delay()函数
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void TICK_Init (void);
 * @endcode
 *******************************************************************************
 */
void TICK_Init (void)
{
	InitTick(SYS_CLOCK*1000000,0);			// Enable SysTick & Interrupt
}

/**
 *******************************************************************************
 * @brief       GPIO初始化			    
 * @details     初始化用于LED的GPIO
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void GPIO_Init(void);
 * @endcode
 *******************************************************************************
 */
void GPIO_Init(void)
{
	PIN_InitTypeDef PINX_InitStruct;
    
	//==Set GPIO Clock
    UnProtectModuleReg(CSCprotect);
 	CSC_PeriphOnModeClock_Config(CSC_ON_PortD,ENABLE);						// Enable PortD Clock
    ProtectModuleReg(CSCprotect);
    
	//==Set GPIO init
	PINX_InitStruct.PINX_Mode				 = PINX_Mode_PushPull_O; 		// Pin select Push Pull mode
	PINX_InitStruct.PINX_PUResistant		 = PINX_PUResistant_Enable;  	// Enable pull up resistor
	PINX_InitStruct.PINX_Speed 			 	 = PINX_Speed_Low;			 
	PINX_InitStruct.PINX_OUTDrive			 = PINX_OUTDrive_Level0;	 	// Pin output driver full strength.
	PINX_InitStruct.PINX_FilterDivider 	 	 = PINX_FilterDivider_Bypass;	// Pin input deglitch filter clock divider bypass
	PINX_InitStruct.PINX_Inverse			 = PINX_Inverse_Disable;	 	// Pin input data not inverse

	PINX_InitStruct.PINX_Alternate_Function  = 0;							// Pin AFS = GPIO
	PINX_InitStruct.PINX_Pin				 = (PX_Pin_8|PX_Pin_9|PX_Pin_10);//初始化LED的GPIO
	GPIO_PortMode_Config(IOMD,&PINX_InitStruct); 					 		 
	
}

/**
 *******************************************************************************
 * @brief       URT0初始化			    
 * @details      初始化URT0  使用引脚   TX:PC10  RX:PC11  波特率：115200
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void URT0_Init(void);
 * @endcode
 *******************************************************************************
 */
void URT0_Init(void)
{
    URT_BRG_TypeDef  URT_BRG;
    URT_Data_TypeDef DataDef;
    
	PIN_InitTypeDef PINX_InitStruct;

	//==Set URT0 Clock
    UnProtectModuleReg(CSCprotect);
	CSC_PeriphProcessClockSource_Config(CSC_UART0_CKS, CK_APB);
 	CSC_PeriphOnModeClock_Config(CSC_ON_UART0,ENABLE);						// Enable UART0 Clock
 	CSC_PeriphOnModeClock_Config(CSC_ON_PortC,ENABLE);						// Enable PortC Clock
    ProtectModuleReg(CSCprotect);
    
	//==Set GPIO init
	//PE0 PPO TX ,PE1 ODO RX
	PINX_InitStruct.PINX_Mode				 = PINX_Mode_PushPull_O; 	 	// Pin select Push Pull mode
	PINX_InitStruct.PINX_PUResistant		 = PINX_PUResistant_Enable;  	// Enable pull up resistor
	PINX_InitStruct.PINX_Speed 			 	 = PINX_Speed_High;			 
	PINX_InitStruct.PINX_OUTDrive			 = PINX_OUTDrive_Level0;	 	// Pin output driver full strength.
	PINX_InitStruct.PINX_FilterDivider 	 	 = PINX_FilterDivider_Bypass;	// Pin input deglitch filter clock divider bypass
	PINX_InitStruct.PINX_Inverse			 = PINX_Inverse_Disable;	 	// Pin input data not inverse
	
	PINX_InitStruct.PINX_Alternate_Function  = PC10_AF_URT0_TX;				// Pin AFS = URT0_TX
	GPIO_PinMode_Config(PINC(10),&PINX_InitStruct); 					 	// TXD at PC10
	PINX_InitStruct.PINX_Mode				 = PINX_Mode_OpenDrain_O; 		// Pin select Open Drain mode
	PINX_InitStruct.PINX_Alternate_Function  = PC11_AF_URT0_RX;				// Pin AFS = URT0_RX
	GPIO_PinMode_Config(PINC(11),&PINX_InitStruct); 					 	// RXD at PC11

    
    //=====Set Clock=====//
    //---Set BaudRate---//
    URT_BRG.URT_InteranlClockSource = URT_BDClock_PROC;
    URT_BRG.URT_BaudRateMode = URT_BDMode_Separated;
    URT_BRG.URT_PrescalerCounterReload = URT0_PSR_VALUE;      				//Set PSR
    URT_BRG.URT_BaudRateCounterReload = URT0_RLR_VALUE;       				//Set RLR
    URT_BaudRateGenerator_Config(URT0, &URT_BRG);		    				//BR115200 = f(CK_URTx)/(PSR+1)/(RLR+1)/(OS_NUM+1)
    URT_BaudRateGenerator_Cmd(URT0, ENABLE);	            				//Enable BaudRateGenerator
    //---TX/RX Clock---//
    URT_TXClockSource_Select(URT0, URT_TXClock_Internal);					//URT_TX use BaudRateGenerator
    URT_RXClockSource_Select(URT0, URT_RXClock_Internal);					//URT_RX use BaudRateGenerator
    URT_TXOverSamplingSampleNumber_Select(URT0, URT0_OS_NUM_VALUE);	        //Set TX OS_NUM
    URT_RXOverSamplingSampleNumber_Select(URT0, URT0_OS_NUM_VALUE);	        //Set RX OS_NUM
    URT_RXOverSamplingMode_Select(URT0, URT_RXSMP_3TIME);
    URT_TX_Cmd(URT0, ENABLE);	                            				//Enable TX
    URT_RX_Cmd(URT0, ENABLE);	                            				//Enable RX
    
    

    //=====Set Mode=====//
    //---Set Data character config---//
    DataDef.URT_TX_DataLength  = URT_DataLength_8;
    DataDef.URT_RX_DataLength  = URT_DataLength_8;
    DataDef.URT_TX_DataOrder   = URT_DataTyped_LSB;
    DataDef.URT_RX_DataOrder   = URT_DataTyped_LSB;
    DataDef.URT_TX_Parity      = URT_Parity_No;
    DataDef.URT_RX_Parity      = URT_Parity_No;
    DataDef.URT_TX_StopBits    = URT_StopBits_1_0;
    DataDef.URT_RX_StopBits    = URT_StopBits_1_0;
    DataDef.URT_TX_DataInverse = DISABLE;
    DataDef.URT_RX_DataInverse = DISABLE;
    URT_DataCharacter_Config(URT0, &DataDef);
    //---Set Mode Select---//
    URT_Mode_Select(URT0, URT_URT_mode);
    //---Set DataLine Select---//
    URT_DataLine_Select(URT0, URT_DataLine_2);
    
    //=====Set Error Control=====//
    // to do...
    
    //=====Set Bus Status Detect Control=====//
    // to do...
    
    //=====Set Data Control=====//
    URT_RXShadowBufferThreshold_Select(URT0, URT_RXTH_1BYTE);
    URT_IdlehandleMode_Select(URT0, URT_IDLEMode_No);
    
    //=====Enable URT Interrupt=====//
    //URT_IT_Cmd(URT0, URT_IT_RX, ENABLE);
    //URT_ITEA_Cmd(URT0, ENABLE);
    //NVIC_EnableIRQ(URT0_IRQn);

    //=====Enable URT=====//
    URT_Cmd(URT0, ENABLE);
		
}

/**
 *******************************************************************************
 * @brief        I2C默认初始化			    
 * @details      默认初始化I2C
 * @param[in]    I2C_Struct* I2Cx
 * @return			 DRV_Success or DRV_Failure
 * @note 
 * @par         Example
 * @code
   DRV_Return I2C_DeInit(I2C_Struct* I2Cx);
 * @endcode
 *******************************************************************************
 */
DRV_Return I2C_DeInit(I2C_Struct* I2Cx)
{
    if(__I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSYF))
        return DRV_Busy;

    __I2C_Disable(I2Cx);

    I2Cx->CR0.W = 0;
    I2Cx->STA.W = (~(I2C_FLAG_EVENTF | I2C_FLAG_TXF));
    I2Cx->INT.W = 0;
    I2Cx->CLK.W = 0;
    I2Cx->CR1.W = 0;
    I2Cx->CR2.W = 0;
    I2Cx->SADR.W = 0;
    I2Cx->TMOUT.W = 0;
    I2Cx->CR0.W = 0;

    return DRV_Success;
}

/**
 *******************************************************************************
 * @brief       I2C初始化			    
 * @details     初始化I2C  使用引脚   SCL:PB10  SDA:PB11  速率：400KHz
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void I2C_Init(void);
 * @endcode
 *******************************************************************************
 */
void I2C_Init(void)
{
	uint32_t			CK_I2C_PR_Frequency;
	uint32_t			SCL_Clock;
  uint16_t lI2C_Pre = 1;
  uint16_t lI2C_DIV = 1;
  uint16_t lI2C_DIV_INX = 1;
  uint16_t lI2C_HT_LT = 0;
  uint16_t lI2C_LT;
  uint16_t lI2C_HT;
    
	PIN_InitTypeDef PINX_InitStruct;
	
	//==Set I2C0 Clock
  UnProtectModuleReg(CSCprotect);
	CSC_PeriphProcessClockSource_Config(CSC_I2C0_CKS, CK_APB);
	CSC_PeriphOnModeClock_Config(CSC_ON_I2C0, ENABLE);					  // Enable IIC0 module clock
	CSC_PeriphOnModeClock_Config(CSC_ON_PortB, ENABLE);					  // Enable PortB module clock
  ProtectModuleReg(CSCprotect);

  PINX_InitStruct.PINX_Mode	            = PINX_Mode_OpenDrain_O;        // Pin select Open Drain mode
  PINX_InitStruct.PINX_PUResistant        = PINX_PUResistant_Enable;      // Enable pull up resistor
  PINX_InitStruct.PINX_Speed              = PINX_Speed_High;           
  PINX_InitStruct.PINX_OUTDrive           = PINX_OUTDrive_Level0;         // Pin output driver full strength.
  PINX_InitStruct.PINX_FilterDivider      = PINX_FilterDivider_Bypass;    // Pin input deglitch filter clock divider bypass
  PINX_InitStruct.PINX_Inverse            = PINX_Inverse_Disable;         // Pin input data not inverse

  PINX_InitStruct.PINX_Alternate_Function = PB10_AF_I2C0_SCL;              // Pin AFS = I2C0_SCL 
  GPIO_PinMode_Config(PINB(10),&PINX_InitStruct);                         
    
  PINX_InitStruct.PINX_Alternate_Function = PB11_AF_I2C0_SDA;              // Pin AFS = I2C0_SDA
  GPIO_PinMode_Config(PINB(11),&PINX_InitStruct);                         

	__I2C_Disable(I2C0);
	I2C_DeInit(I2C0);
  CK_I2C_PR_Frequency = I2C_PR_FREQUENCY;
  SCL_Clock = I2C0_CLOCK_FREQ;    

    //=== I2C Config ===//
    //===== I2C Output Clock Config =====//
    // CK_I2C_PR
    // SCL Output Clock
    // HT + LT, <= 32 >=9, CK_I2C_PR / SCL Clock / Prescaler / DIV

  do{
      lI2C_HT_LT = CK_I2C_PR_Frequency / SCL_Clock / lI2C_Pre / lI2C_DIV;
      if((lI2C_HT_LT >= 32) || (lI2C_HT_LT <=9)) 
      {
        lI2C_Pre ++;
        if(lI2C_Pre > 8)
        {
          lI2C_Pre = 1;
          lI2C_DIV =lI2C_DIV*2;
          lI2C_DIV_INX++;
        }
      }
  }while((lI2C_HT_LT >= 32) || (lI2C_HT_LT <=9));

  lI2C_LT = (lI2C_HT_LT >> 1);
  lI2C_HT = lI2C_HT_LT - lI2C_LT;

  __I2C_SetClockSource(I2C0, I2C_CLK_SRC_PROC);
  __I2C_SetClockPrescaler(I2C0, lI2C_Pre - 1);
  __I2C_SetClockDivider(I2C0, lI2C_DIV_INX - 1);
  __I2C_SetSCLHighTime(I2C0, lI2C_HT - 1);
  __I2C_SetSCLLowTime(I2C0, lI2C_LT - 1);

	//===== I2C Opration Mode Config =====//
	__I2C_GeneralCallAddress_Disable(I2C0);
	__I2C_SlaveAddressDetect_Disable(I2C0, (I2C_SADR_1 | I2C_SADR_2));
		
	//===== I2C Interrupt Config =====//
	//		  NVIC_EnableIRQ(I2C0_IRQn);
	//		  NVIC_SetPriority(I2C0_IRQn, 1);										  // Suggest SYSTICK Priority = 0
	//																				  //		   Other Priority > 0
	
	__I2C_ITEA_Disable(I2C0);
	__I2C_IT_Disable(I2C0, (I2C_IT_TMOUT | I2C_IT_EVENT | I2C_IT_ERR | I2C_IT_BUF | I2C_IT_WUP));

	//===== I2C Timeout Config =====//
	__I2C_TMO_Enable(I2C0);
	__I2C_SetTimeOutClockSource(I2C0, I2C_TMO_CKS_DIV64);
	__I2C_SetTimeOutDetectionMode(I2C0, I2C_TMO_MDS_GENERAL);
	__I2C_SetTimeOutCount(I2C0, I2C_TMO_MDS_GENERAL);

	//===== I2C Enable =====//
	__I2C_Enable(I2C0);
}

/**
 *******************************************************************************
 * @brief       DMA写入初始化			    
 * @details      初始化DMA用I2C Write
 * @param[in]   No
 * @return		No
 * @note 			
 * @par         Example
 * @code
   void DMA_I2CWrite_Init(void);
 * @endcode
 *******************************************************************************
 */
void DMA_I2CWrite_Init(void)
{  
    
    DMA_BaseInitTypeDef DMATestPattern;
    
	//==Set DMA Clock
    UnProtectModuleReg(CSCprotect);
 	CSC_PeriphOnModeClock_Config(CSC_ON_DMA,ENABLE);
    ProtectModuleReg(CSCprotect);

    // ------------------------------------------------------------------------
    // 1.Enable DMA
    DMA_Cmd(ENABLE);
    
    // ------------------------------------------------------------------------
    // 2.Enable Channel0
    DMA_Channel_Cmd(DMAChannel0, ENABLE);
    
    // ------------------------------------------------------------------------
    DMA_BaseInitStructure_Init(&DMATestPattern);
    
    // 3.initial & modify parameter
    {   
        // DMA channel select
        DMATestPattern.DMAChx = DMAChannel0;
        
        // channel x source/destination auto increase address
        DMATestPattern.SrcSINCSel = ENABLE;
        DMATestPattern.DestDINCSel = DISABLE;
        
        // DMA source peripheral config
        DMATestPattern.SrcSymSel = DMA_MEM_Read;
        
        // DMA destination peripheral config
        DMATestPattern.DestSymSel = DMA_I2C0_TX;
        
        // DMA Burst size config
        DMATestPattern.BurstDataSize = DMA_BurstSize_1Byte;
        
        // DMA transfer data count initial number
        DMATestPattern.DMATransferNUM = DATA_LENGTH;
    
        // source/destination config
        DMATestPattern.DMASourceAddr = &SendBuf;
    }
    
    // ------------------------------------------------------------------------
    // Setting M2M simple parameter
    DMA_Base_Init(&DMATestPattern);
    return;
}

/**
 *******************************************************************************
 * @brief       DMA读取初始化			    
 * @details      初始化DMA用I2C read
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void DMA_I2CRead_Init(void);
 * @endcode
 *******************************************************************************
 */
void DMA_I2CRead_Init(void)
{  
    
    DMA_BaseInitTypeDef DMATestPattern;
    
	//==Set DMA Clock
    UnProtectModuleReg(CSCprotect);
 	CSC_PeriphOnModeClock_Config(CSC_ON_DMA,ENABLE);
    ProtectModuleReg(CSCprotect);

    // ------------------------------------------------------------------------
    // 1.Enable DMA
    DMA_Cmd(ENABLE);
    
    // ------------------------------------------------------------------------
    // 2.Enable Channel0
    DMA_Channel_Cmd(DMAChannel0, ENABLE);
    
    // ------------------------------------------------------------------------
    DMA_BaseInitStructure_Init(&DMATestPattern);
    
    // 3.initial & modify parameter
    {   
        // DMA channel select
        DMATestPattern.DMAChx = DMAChannel0;
        
        // channel x source/destination auto increase address
        DMATestPattern.SrcSINCSel = DISABLE;
        DMATestPattern.DestDINCSel = ENABLE;
        
        // DMA source peripheral config
        DMATestPattern.SrcSymSel = DMA_I2C0_RX;
        
        // DMA destination peripheral config
        DMATestPattern.DestSymSel = DMA_MEM_Write;
        
        // DMA Burst size config
        DMATestPattern.BurstDataSize = DMA_BurstSize_1Byte;
        
        // DMA transfer data count initial number
        DMATestPattern.DMATransferNUM = 64;
    
        // source/destination config
        DMATestPattern.DMADestinationAddr = (uint8_t *)&RcvBuf;
    }
    
    // ------------------------------------------------------------------------
    // Setting M2M simple parameter
    DMA_Base_Init(&DMATestPattern);
    return;
}
/*
*************************************************************************************
*/ 

/*
*************************************************************************************
*  Uart Function
*
*************************************************************************************
*/
/**
 *******************************************************************************
 * @brief       fputc函数重定向			    
 * @details      fputc函数重定向，即可用printf输出信息
 * @param[in]   ch:调用printf时不需要管这个参数
 * @param[in] 	FILE *f:调用printf时不需要管这个参数
 * @return		  ch:调用printf时不需要管这个参数
 * @note 
 * @par         Example
 * @code
   int fputc(int ch,FILE *f);
 * @endcode
 *******************************************************************************
 */
int fputc(int ch,FILE *f)
{
	
	URT_SetTXData(PRINTF_URTX,1,ch);
	while(URT_GetITSingleFlagStatus(PRINTF_URTX,URT_IT_TC)==DRV_UnHappened);
	URT_ClearITFlag(PRINTF_URTX,URT_IT_TC);
	
	return ch;
}

/**
 *******************************************************************************
 * @brief       标准串口输出函数			    
 * @details     串口输出函数，与printf类似
 * @param[in]   ch:发送内容
 * @return		  No
 * @note 
 * @par         Example
 * @code
   void URT_SendByte(0x78);
 * @endcode
 *******************************************************************************
 */
void URT_SendByte(int ch)
{
	
	URT_SetTXData(PRINTF_URTX,1,ch);
	while(URT_GetITSingleFlagStatus(PRINTF_URTX,URT_IT_TC)==DRV_UnHappened);
	URT_ClearITFlag(PRINTF_URTX,URT_IT_TC);
	
}

/*
*************************************************************************************
*/ 
/*
*************************************************************************************
*  Other Function
*
*************************************************************************************
*/
/**
 *******************************************************************************
 * @brief       等待I2C操作完成函数			    
 * @details     等待I2C操作完成函数，查看I2C操作是否成功	
 * @param[in]   No
 * @return		  DRV_Success or DRV_Failure
 * @note 
 * @par         Example
 * @code
   u8 I2C_WaitHandleDone(void);
 * @endcode
 *******************************************************************************
 */
u8 I2C_WaitHandleDone(void)
{
	u8 lState;
	__I2C_SetTimeOutCount(I2C0, 100);
  __I2C_ClearFlag(I2C0,(I2C_FLAG_TMOUTF|I2C_FLAG_EVENTF | I2C_FLAG_TXF));
  while(__I2C_GetStateFlag(I2C0) == 0)
  {
		if(I2C_GetFlagStatus(I2C0,I2C_FLAG_TMOUTF)==DRV_Happened)		//I2C超时
		{
			return DRV_Failure;
		}
  }
  lState = __I2C_GetStatusCode(I2C0);
  if((lState==0x00)||(lState>=0xF8))
  {
		return DRV_Failure;			//发生I2C其他错误
  }
    return DRV_Success;
}

/**
 *******************************************************************************
 * @brief       I2C写24C64函数			    
 * @details     I2C写24C64函数	
 * @param[in]   DevAddr:设备地址，即从机地址
 * @param[in]   RegAddr:24C64数据起始地址
 * @param[in]   *pBuf:待发送数据的缓存地址
 * @param[in]   Len:待发送的数据个数
 * @return		  DRV_Success or DRV_Failure
 * @note 				因为24C64的内部Buffer为32bit，因此Len长度超过32的话会造成写出错，
 * 							需要分多次写入
 * @par         Example
 * @code
   u8 I2C_Write_24C64_DMA((u8 DevAddr,u8 RegAddr,u8 *pBuf,u8 Len);
 * @endcode
 *******************************************************************************
 */
u8 I2C_Write_24C64_DMA(u8 DevAddr,u8 RegAddr,u8 *pBuf,u8 Len)
{
	DMA_I2CWrite_Init();
	// 发送START
	Set_STA_STO_AA_100(I2C0);	
	if(I2C_WaitHandleDone()==DRV_Failure) return DRV_Failure;		// 等待I2C事件完成
	
	// 发送器件地址
	Set_STA_STO_AA_000(I2C0);	
	__I2C_SendSBUF(I2C0,DevAddr&0xFE);
	if(I2C_WaitHandleDone()==DRV_Failure) return DRV_Failure;		// 等待I2C事件完成

	// 发送数据地址
	Set_STA_STO_AA_001(I2C0);	
	__I2C_SendSBUF(I2C0, HIBYTE(RegAddr));	// 数据地址高8位
	if(I2C_WaitHandleDone()==DRV_Failure) return DRV_Failure;		// 等待I2C事件完成
	Set_STA_STO_AA_001(I2C0);	
	__I2C_SendSBUF(I2C0, LOBYTE(RegAddr));	// 数据地址低8位
	if(I2C_WaitHandleDone()==DRV_Failure) return DRV_Failure;		// 等待I2C事件完成

	// 发送数据
	I2C_TXDMA_Cmd( I2C0, ENABLE);
	DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
	DMA_StartRequest(DMAChannel0);
	while(DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
	if(I2C_WaitHandleDone()==DRV_Failure) return DRV_Failure;		// 不仅是等到DMA发送完成，还要等待I2C事件完成
	
  Set_STA_STO_AA_010(I2C0);	// 发送STOP
	__I2C_ClearStateFlag(I2C0);
	DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
	
	return DRV_Success;
}

/**
 *******************************************************************************
 * @brief       I2C读24C64函数			    
 * @details     I2C读24C64函数	
 * @param[in]   DevAddr:设备地址，即从机地址
 * @param[in]   RegAddr:24C64数据起始地址
 * @param[in]   *pBuf:保存读取数据的缓存地址
 * @param[in]   Len:待读取的数据个数
 * @return		  DRV_Success or DRV_Failure
 * @note 
 * @par         Example
 * @code
   u8 I2C_Read_24C64_DMA((u8 DevAddr,u8 RegAddr,u8 *pBuf,u8 Len);
 * @endcode
 *******************************************************************************
 */
u8 I2C_Read_24C64_DMA(u8 DevAddr,u8 RegAddr,u8 *pBuf,u8 Len)
{
	DMA_I2CRead_Init();
	Set_STA_STO_AA_100(I2C0);	// 发送START
	if(I2C_WaitHandleDone()==DRV_Failure) return DRV_Failure;		// 等待完成
	
	Set_STA_STO_AA_000(I2C0);	
	__I2C_SendSBUF(I2C0, DevAddr&0xFE);// 发送器件地址
	if(I2C_WaitHandleDone()==DRV_Failure) return DRV_Failure;		// 等待完成

	// 发送数据地址
	Set_STA_STO_AA_001(I2C0);	
	__I2C_SendSBUF(I2C0, HIBYTE(RegAddr));	// 数据地址高8位
	if(I2C_WaitHandleDone()==DRV_Failure) return DRV_Failure;		// 等待I2C事件完成
	Set_STA_STO_AA_001(I2C0);	
	__I2C_SendSBUF(I2C0, LOBYTE(RegAddr));	// 数据地址低8位
	if(I2C_WaitHandleDone()==DRV_Failure) return DRV_Failure;		// 等待I2C事件完成

	Set_STA_STO_AA_100(I2C0);	// 重新发送START
	if(I2C_WaitHandleDone()==DRV_Failure) return DRV_Failure;		// 等待完成
	
	Set_STA_STO_AA_000(I2C0);	
	__I2C_SendSBUF(I2C0, DevAddr|0x01);// 发送器件地址
	if(I2C_WaitHandleDone()==DRV_Failure) return DRV_Failure;		// 等待完成

	Set_STA_STO_AA_001(I2C0);
	// 启动DMA I2C Receive
	I2C_RXDMA_Cmd(I2C0, ENABLE);
	DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
	DMA_StartRequest(DMAChannel0);
	while(DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
	if(I2C_WaitHandleDone()==DRV_Failure) return DRV_Failure;		// 不仅是等到DMA接收完成，还要等待I2C事件完成

	Set_STA_STO_AA_010(I2C0);			// 发送STOP
	__I2C_ClearStateFlag(I2C0);
	DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
	return DRV_Success;
}
/*
*************************************************************************************
*  MAIN
*
*************************************************************************************
*/
/**
 *******************************************************************************
 * @brief       主函数			    
 * @details     主函数		
 * @param[in]   No
 * @return		  No
 * @note 
 * @par         Example
 * @code
 * @endcode
 *******************************************************************************
 */
int main()
{	
	u8 i=0;
	u8 I2C_Write_Return=0,I2C_Read_Return=0;
	u16 Data_Address_24C64=0x00;
	CSC_Init();
	TICK_Init();
	GPIO_Init();
	URT0_Init();
	I2C_Init();
	IO_LED_G_0=0;IO_LED_R=0;IO_LED_G_1=0;
	Delay(100);
	printf("\nHello!");
	IO_LED_G_0=1;IO_LED_R=1;IO_LED_G_1=1;

	for(i=0;i<DATA_LENGTH;i++)
	{
		SendBuf[i]=i;				//准备一组数据buffer
	}
	i=0;
	
	while(1)
  {
		Delay(100);
		IO_LED_G_0=!IO_LED_G_0;
		I2C_Write_Return=I2C_Write_24C64_DMA(SLAVE_ADDRESS_A0, Data_Address_24C64, &SendBuf[0],DATA_LENGTH);	
		Delay(100);			//给一个小的延时后再读
		I2C_Read_Return=I2C_Read_24C64_DMA(SLAVE_ADDRESS_A0, Data_Address_24C64, &RcvBuf[0],DATA_LENGTH);
		
		if((I2C_Write_Return!=0)||(I2C_Read_Return!=0))
		{
			// 出错,重置I2C
			IO_LED_R=0;
			I2C_Init();
			IO_LED_R=1;
		}
		printf("\r\nSend: ");
		for(i=0;i<DATA_LENGTH;i++)
		{
			printf("0x%02X  ",SendBuf[i]);
		}
		
		printf("\r\nRcv:  ");
		for(i=0;i<64;i++)
		{
			printf("0x%02X  ",RcvBuf[i]);
		}
		
  }
}
/*
*************************************************************************************
*/ 


