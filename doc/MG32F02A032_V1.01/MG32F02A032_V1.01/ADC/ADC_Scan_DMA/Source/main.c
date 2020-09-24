/**
  ******************************************************************************
 *
 * @file        main.c
 *
 * @brief       Analog conversion to Digital (ADC)
 *
 * @par         Project
 *              MG32x02z
 *							利用DMA进行ADC通道扫描采集PA0、PA1、PA2的
 *							电压,并透过串口输出，当ADC数据量较大时，
 *							DMA有较为明显的为CPU降低压力的效果。
 *							该Demo中使用的ADC输入引脚为PA0、PA1、PA2。
 *							使用该Demo时请为VERF+引脚供大于等于2.7V且不
 *							大于VDDA的参考电压，并根据实际参考电压修改
 *							ADC数据公式。
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
#define IO_LED_R			PD9
#define IO_LED_G_1			PD10

#define ADC_DMA_SENDBYTE_CNT  12			//通过DMA传输的ADC数据字节数，非ADC数据数
uint16_t AdcData[20] = {0};						//ADC接收数据缓冲
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
 * @brief       ADC初始化			    
* @details      初始化ADC  使用引脚   PA0、PA1、PA2
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void ADC_Init(void);
 * @endcode
 *******************************************************************************
 */
void ADC_Init(void)
{
		ADC_InitTypeDef ADC_Base;
		PIN_InitTypeDef PINX_InitStruct;
			//==Set ADC Clock
    UnProtectModuleReg(CSCprotect);
		CSC_PeriphProcessClockSource_Config(CSC_ADC0_CKS, CK_APB);
		CSC_PeriphOnModeClock_Config(CSC_ON_ADC0,ENABLE);						// Enable ADC0 Clock
		CSC_PeriphOnModeClock_Config(CSC_ON_PortA,ENABLE);						// Enable PortA Clock
    ProtectModuleReg(CSCprotect);
	
		PINX_InitStruct.PINX_Mode				 = PINX_Mode_PushPull_O; 	 // Pin select digital input mode
		PINX_InitStruct.PINX_PUResistant		 = PINX_PUResistant_Enable;  // Enable pull up resistor
		PINX_InitStruct.PINX_Speed 			 	 = PINX_Speed_Low;			 
		PINX_InitStruct.PINX_OUTDrive			 = PINX_OUTDrive_Level0;	 // Pin output driver full strength.
		PINX_InitStruct.PINX_FilterDivider 	 	 = PINX_FilterDivider_Bypass;// Pin input deglitch filter clock divider bypass
		PINX_InitStruct.PINX_Inverse			 = PINX_Inverse_Disable;	 // Pin input data not inverse
	
		PINX_InitStruct.PINX_Mode				 = PINX_Mode_Analog_IO; 	 // Pin select digital input mode
		PINX_InitStruct.PINX_Alternate_Function = 0;						 // Pin AFS = 0
		GPIO_PinMode_Config(PINA(0),&PINX_InitStruct); 					 // ADC setup at PA0
		GPIO_PinMode_Config(PINA(1),&PINX_InitStruct); 					 // ADC setup at PA1
		GPIO_PinMode_Config(PINA(2),&PINX_InitStruct); 					 // ADC setup at PA2
		
    ADC_DeInit(ADC0);
    // ------------------------------------------------------------------------
    // 1.Config ADC base parameter    
    ADC_BaseStructure_Init(&ADC_Base);
       // modify parameter
    ADC_Base.ADCMainClockSelect = ADC_CKADC;
    ADC_Base.ADC_IntCK_Div = ADC_IntDIV16;   // for internal clock divider
    
        // ADC data alignment mode (Right or Left)
    ADC_Base.ADC_DataAlign = ADC_RightJustified;
        
        // ADC conversion resolution 8, 10 or 12 bit
    ADC_Base.ADC_ResolutionSel = ADC_12BitData;
        
        // ADC overwritten data or keep data
    ADC_Base.ADC_DataOverrunEvent = ADC_DataOverWritten;
        
    
    ADC_Base_Init(ADC0, &ADC_Base);
    
    // ------------------------------------------------------------------------
	// 2.Enable ADC 
    ADC_Cmd(ADC0, ENABLE);
    
    // ------------------------------------------------------------------------
    // 3.Config ADC Mode
    ADC_ConversionMode_Select(ADC0, LoopMode);   // Continue模式
    ADC_PGA_Cmd(ADC0, DISABLE);                 // Disable PGA
    ADC_SingleDifferentMode_Select(ADC0, ADC_SingleMode);  // Single Mode  
    
    // ------------------------------------------------------------------------
    // 4.Clear all flag
    ADC_ClearFlag(ADC0, 0xFFFFFFFF);

    // ------------------------------------------------------------------------
    // 5.Start Calibration 
   
		ADC_StartCalibration(ADC0, ENABLE);
		// 6.Select Exnternal Channel (PA0)
		ADC_ExternalChannel_Select(ADC0, ADC_ExtAIN0);
		ADC_ExternalChannel_Select(ADC0, ADC_ExtAIN0);
		ADC_ExternalChannel_Select(ADC0, ADC_ExtAIN1);
		ADC_ExternalChannel_Select(ADC0, ADC_ExtAIN2);
		ADC_ScanLoopChannel_Enable(ADC0, (ADC_MskAIN0 | ADC_MskAIN1| ADC_MskAIN2), ENABLE);	//通道只选择需要的通道进行扫描。

    // ------------------------------------------------------------------------
		// 7.Trigger Source select and Start conversion
    ADC_TriggerSource_Select(ADC0, ADC_START);

}
/**
 *******************************************************************************
 * @brief       DMA初始化			    
 * @details     DMA初始化，用于ADC数据传输
 * @param[in]   No
 * @return		  No
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

    /* Configure peripheral clock */
    UnProtectModuleReg(CSCprotect);
		CSC_PeriphOnModeClock_Config(CSC_ON_DMA, ENABLE);					  // Enable DMA module clock
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
        DMATestPattern.SrcSymSel = DMA_ADC0_IN;
        
        // DMA destination peripheral config
        DMATestPattern.DestSymSel = DMA_MEM_Write;
        
        // DMA Burst size config
        DMATestPattern.BurstDataSize = DMA_BurstSize_2Byte;
        
        // DMA transfer data count initial number
        DMATestPattern.DMATransferNUM = ADC_DMA_SENDBYTE_CNT;			//传输数据量，单位为字节，并不是多少个数据包
    
        // source/destination config
				DMATestPattern.DMASourceAddr = &ADC0->DAT0;
        DMATestPattern.DMADestinationAddr = (uint32_t *)&AdcData;
    }
		DMA_IT_Config(DMAChannel0, DMA_Complete_ITE, ENABLE);
		DMA_ITEA_Cmd(DMA, ENABLE);
		NVIC_EnableIRQ(DMA_IRQn);
    ADC_DMADataSize_Select(ADC0, ADC_DMA_16bit);//设置ADC的DMA传输数据包长为16位
		ADC_DMA_Cmd(ADC0, ENABLE);
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
*  ADC Function
*
*************************************************************************************
*/
/**
 *******************************************************************************
 * @brief       使用DMA进行外部通道ADC通道扫描采样			    
 * @details     使用DMA进行外部通道ADC通道扫描采样		
 * @param[in]   No
 * @return		  No
 * @note 
 * @par         Example
 * @code
   void ADC_DMA_Scan_GetValue(void);
 * @endcode
 *******************************************************************************
 */
void ADC_DMA_Scan_GetValue(void)
{
	ADC_SoftwareConversion_Cmd(ADC0, ENABLE);
	ADC_DMA_Cmd(ADC0, ENABLE);//每次DMA传输开始前都必须重新使能一次
	ADC_DMADataSize_Select(ADC0, ADC_DMA_16bit);//设置ADC的DMA传输数据包长为16位,每次DMA传输开始前都必须设置一次
	DMA_StartRequest(DMAChannel0);
}

/*
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
	u8 i;
	
	CSC_Init();
	TICK_Init();
	GPIO_Init();
	URT0_Init();
	DMA_Init();
	ADC_Init();
	
	IO_LED_G_0=0;IO_LED_R=0;IO_LED_G_1=0;
	Delay(100);
	printf("\nHello!");
	IO_LED_G_0=1;IO_LED_R=1;IO_LED_G_1=1;
	ADC_DMA_Scan_GetValue();

	while(1)
  {
		Delay(100);
		if(DMAComplete == 1)
		{
			DMAComplete = 0;
			for(i=0;i<ADC_DMA_SENDBYTE_CNT/2;i++)			//采集一次ADC是2字节
			{
				printf("\nADC: 0x%04X   ",AdcData[i]);
			}
			DMA_Init();			//每次通道扫描前要重置一下DMA和ADC的初始化，避免DMA接收的数据通道混乱
			ADC_Init();
			memset(AdcData,0,20*sizeof(u16));//清除缓冲区
			ADC_DMA_Scan_GetValue();
		}
  }
}
/*
*************************************************************************************
*/ 


