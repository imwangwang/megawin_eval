/**
  ******************************************************************************
 *
 * @file        main.c
 *
 * @brief       Timer36
 *
 * @par         Project
 *              MG32x02z
 *							此demo旨在利用TM36输入捕获PWM波形。
 *							此demo会捕获10个脉冲到数组中，并在
 *							串口打印这10个脉冲的计数器值。
 *		
 *							TM中仅TM36的IC3支持使用DMA捕获。
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

// DEMO Led define
#define IO_LED_G_0			PD8
#define IO_LED_R				PD9
#define IO_LED_G_1			PD10

#define DMA_BUFFER_CNT	10		//抓取10个输入捕获数据
uint16_t RcvBuf[]={0};
u8 DMAComplete = 0;
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

/**
 *******************************************************************************
 * @brief       DMA中断处理入口			    
 * @details     DMA中断服务函数
 * @param[in]   No
 * @return			No
 * @note 
 * @par         Example
 * @code
   void DMA_IRQHandler(void);
 * @endcode
 *******************************************************************************
 */
void DMA_IRQHandler(void)
{
    uint8_t IRQ_ID;
    
    
    IRQ_ID = EXIC_GetITSourceID(DMA_IRQn);      
    if( IRQ_ID & EXIC_SRC2_ID8_dma_b0)
    {
			if(DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_Happened)
			{
				IO_LED_G_1=!IO_LED_G_1;
				DMAComplete=1;
				DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			}
    }    
    
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
 * @brief       TM36捕获初始化			    
 * @details     初始化TM36捕获功能
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void TM36_Capture_Init(void);
 * @endcode
 *******************************************************************************
 */
void TM36_Capture_Init(void)
{  
		TM_TimeBaseInitTypeDef TM_TimeBase_InitStruct;
		PIN_InitTypeDef PINX_InitStruct;

		UnProtectModuleReg(CSCprotect);
		CSC_PeriphProcessClockSource_Config(CSC_TM36_CKS, CK_APB);
		CSC_PeriphOnModeClock_Config(CSC_ON_TM36,ENABLE);						// Enable TM36 Clock
		CSC_PeriphOnModeClock_Config(CSC_ON_PortA,ENABLE);						// Enable PortB Clock
    ProtectModuleReg(CSCprotect);

		//==Set GPIO init
		PINX_InitStruct.PINX_Mode				 = PINX_Mode_Digital_I; 								// Pin select Digital input mode
		PINX_InitStruct.PINX_PUResistant		 = PINX_PUResistant_Enable;  				// Enable pull up resistor
		PINX_InitStruct.PINX_Speed 			 	 = PINX_Speed_Low;			 
		PINX_InitStruct.PINX_OUTDrive			 = PINX_OUTDrive_Level0;	 						// Pin output driver full strength.
		PINX_InitStruct.PINX_FilterDivider 	 	 = PINX_FilterDivider_Bypass;			// Pin input deglitch filter clock divider bypass
		PINX_InitStruct.PINX_Inverse			 = PINX_Inverse_Disable;	 						// Pin input data not inverse
		PINX_InitStruct.PINX_Alternate_Function  = PA15_AF_TM36_IC3;														// Pin AFS = TIM36 IC
		GPIO_PinMode_Config(PINA(15),&PINX_InitStruct); 		  										// Set PA15 to TIM36 IC
	
    TM_DeInit(TM36);
    // ----------------------------------------------------
    // 1.TimeBase structure initial
    TM_TimeBaseStruct_Init(&TM_TimeBase_InitStruct);
    
    // modify parameter
    TM_TimeBase_InitStruct.TM_MainClockDirection =TM_UpCount;
    TM_TimeBase_InitStruct.TM_Period = 65535; 
    TM_TimeBase_InitStruct.TM_Prescaler = 0;
		TM_TimeBase_InitStruct.TM_MainClockSource = TM_CK_INT;				
    TM_TimeBase_InitStruct.TM_2ndClockSource = TM_CK_INT;					
    TM_TimeBase_InitStruct.TM_CounterMode = Separate;
    
    TM_TimeBase_Init(TM36, &TM_TimeBase_InitStruct);  
    // ----------------------------------------------------
    // 2.config overwrite mode (keep data)
    TM_IC3OverWritten_Cmd(TM36, TM_Keep);    
    // ----------------------------------------------------
    // 3.config TM36 channel 3 function 
    TM_CH3Function_Config(TM36, TM_InputCapture);
    TM_IN3Source_Select(TM36, TMx_InputMUX_Pin);      			// TM36_IN3 from Pin (PA15)               
    // ----------------------------------------------------	
// Rising to Rising edge 
//    TM_TRGICounter_Select(TM36,TRGI_StartupRising);      //TM36不可加这行
//    TM_TRGIPrescaler_Select(TM36,TRGI_StartupRising);      //TM36不可加这行
    TM_IN3TriggerEvent_Select(TM36, IC_RisingEdge);
		TM_Counter_Cmd(TM36, ENABLE);
	  TM_Prescaler_Cmd(TM36, ENABLE);

    
    // ----------------------------------------------------
    // 5.clear all flag
    TM_ClearFlag(TM36, TMx_CF3A | TMx_CF3B);
    // ----------------------------------------------------
    // 6.enable Timer 
    TM_Timer_Cmd(TM36,ENABLE);
    
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
 	CSC_PeriphOnModeClock_Config(CSC_ON_PortA,ENABLE);						// Enable PortD Clock
 	CSC_PeriphOnModeClock_Config(CSC_ON_PortD,ENABLE);						// Enable PortD Clock
    ProtectModuleReg(CSCprotect);
    
	//==Set GPIO init
	PINX_InitStruct.PINX_Mode				 = PINX_Mode_PushPull_O; 		// Pin select Push Pull mode
	PINX_InitStruct.PINX_PUResistant		 = PINX_PUResistant_Enable;  	// Enable pull up resistor
	PINX_InitStruct.PINX_Speed 			 	 = PINX_Speed_High;			 
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
 * @brief       DMA初始化			    
 * @details      初始化DMA
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void DMA_Init(void);
 * @endcode
 *******************************************************************************
 */
void DMA_Init(void)
{
	DMA_BaseInitTypeDef DMATestPattern;

	UnProtectModuleReg(CSCprotect);
	CSC_PeriphOnModeClock_Config(CSC_ON_DMA,ENABLE);						// Enable DMA module clock
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
       
        // DMA channel select
        DMATestPattern.DMAChx = DMAChannel0;
        
        // channel x source/destination auto increase address
        DMATestPattern.SrcSINCSel = DISABLE;
        DMATestPattern.DestDINCSel = ENABLE;
        
        // DMA source peripheral config
        DMATestPattern.SrcSymSel = DMA_TM36_IC3;
        
        // DMA destination peripheral config
        DMATestPattern.DestSymSel = DMA_MEM_Write;
        
        // DMA Burst size config
        DMATestPattern.BurstDataSize = DMA_BurstSize_2Byte;           //输入捕获是16bit
        
        // DMA transfer data count initial number
        DMATestPattern.DMATransferNUM = DMA_BUFFER_CNT*2;							//一个数据是16bit，但是单位是byte，因此要*2
    
        // source/destination config
        DMATestPattern.DMADestinationAddr = RcvBuf;
				
				DMA_IT_Config(DMAChannel0, DMA_Complete_ITE, ENABLE);
				DMA_ITEA_Cmd(DMA, ENABLE);
				NVIC_EnableIRQ(DMA_IRQn);
				DMA_Channel_Cmd(DMAChannel0, ENABLE);
				DMA_Base_Init(&DMATestPattern);
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
	u32 buf[10]={0};
	CSC_Init();
	TICK_Init();
	TM36_Capture_Init();
	GPIO_Init();
	URT0_Init();
	DMA_Init();
	IO_LED_G_0=0;IO_LED_R=0;IO_LED_G_1=0;
	Delay(100);
	printf("\nHello!");
	IO_LED_G_0=1;IO_LED_R=1;IO_LED_G_1=1;

	TM_DMAChannel_Cmd(TM36, TMx_DMA_IC3, ENABLE);
	DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
	
	DMA_StartRequest(DMAChannel0);
	while(1)
	{
			if(DMAComplete == 1)
		{
			DMAComplete=0;
			for(i=0;i<DMA_BUFFER_CNT;i++)		
			{
				printf("%d ",RcvBuf[i]);
			}
			break;
		}
	}

	TM_Timer_Cmd(TM36,DISABLE);
	
	while(1);
	

}
/*
*************************************************************************************
*/ 


