/**
  ******************************************************************************
 *
 * @file        main.c
 *
 * @brief       CRC
 *
 * @par         Project
 *              MG32x02z
 *							功能描述：
 *							此demo使用DMA导入校验数据包到CRC中进行各种
 *							CRC类型运算，运算结果正确会在串口输出通过，
 *							不正确也会在串口输出不通过的讯息。
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

// DEMO Led define
#define IO_LED_G_0			PD8
#define IO_LED_R				PD9
#define IO_LED_G_1			PD10

#define GPL_InputDataEndian_LITTLE      GPL_CR0_BEND_EN_disable_w
#define GPL_InputDataEndian_BIG         GPL_CR0_BEND_EN_mask_w

#define IS_GPL_InputDataEndian_MODE(MODE)     (((MODE) == GPL_InputDataEndian_LITTLE) || \
                                               ((MODE) == GPL_InputDataEndian_BIG))



#define GPL_InputDataInverse_DISABLE    GPL_CR0_IN_INV_disable_w

#define GPL_InputDataInverse_ENABLE     GPL_CR0_IN_INV_mask_w

#define IS_GPL_InputDataInverse_MODE(MODE)    (((MODE) == GPL_InputDataInverse_ENABLE) || \
                                               ((MODE) == GPL_InputDataInverse_DISABLE))


#define GPL_InputDataReverse_NONE       GPL_CR0_BREV_MDS_disable_w
#define GPL_InputDataReverse_BYTE       GPL_CR0_BREV_MDS_8bit_w
#define GPL_InputDataReverse_HALFWORD   GPL_CR0_BREV_MDS_16bit_w
#define GPL_InputDataReverse_WORD       GPL_CR0_BREV_MDS_32bit_w

#define IS_GPL_InputDataReverse_MODE(MODE)    (((MODE) == GPL_InputDataReverse_NONE) || \
                                               ((MODE) == GPL_InputDataReverse_BYTE) || \
                                               ((MODE) == GPL_InputDataReverse_HALFWORD) || \
                                               ((MODE) == GPL_InputDataReverse_WORD))



#define GPL_OutputDataReverse_NONE      GPL_CR1_CRC_BREV_disable_w
#define GPL_OutputDataReverse_BYTE      GPL_CR1_CRC_BREV_8bit_w
#define GPL_OutputDataReverse_HALFWORD  GPL_CR1_CRC_BREV_16bit_w
#define GPL_OutputDataReverse_WORD      GPL_CR1_CRC_BREV_32bit_w

#define IS_GPL_OutputDataReverse_MODE(MODE)    (((MODE) == GPL_OutputDataReverse_NONE) || \
                                                ((MODE) == GPL_OutputDataReverse_BYTE) || \
                                                ((MODE) == GPL_OutputDataReverse_HALFWORD) || \
                                                ((MODE) == GPL_OutputDataReverse_WORD))



#define GPL_CRC_Polynomial_0x1021       GPL_CR1_CRC_MDS_ccitt16_w
#define GPL_CRC_Polynomial_0x07         GPL_CR1_CRC_MDS_crc8_w
#define GPL_CRC_Polynomial_0x8005       GPL_CR1_CRC_MDS_crc16_w
#define GPL_CRC_Polynomial_0x4C11DB7    GPL_CR1_CRC_MDS_crc32_w

#define IS_GPL_CRC_Polynomial_MODE(MODE)    (((MODE) == GPL_CRC_Polynomial_0x1021) || \
                                             ((MODE) == GPL_CRC_Polynomial_0x07) || \
                                             ((MODE) == GPL_CRC_Polynomial_0x8005) || \
                                             ((MODE) == GPL_CRC_Polynomial_0x4C11DB7))



#define GPL_CRC_InputDataWidth_8bit     GPL_CR1_CRC_DSIZE_8bit_w
#define GPL_CRC_InputDataWidth_16bi     GPL_CR1_CRC_DSIZE_16bit_w
#define GPL_CRC_InputDataWidth_32bit    GPL_CR1_CRC_DSIZE_32bit_w

#define IS_GPL_CRC_InputDataWidth_MODE(MODE)    (((MODE) == GPL_CRC_InputDataWidth_8bit) || \
                                                 ((MODE) == GPL_CRC_InputDataWidth_16bi) || \
                                                 ((MODE) == GPL_CRC_InputDataWidth_32bit))

#define CRC_8 													0
#define CRC_8_ITU 											1
#define CRC_8_ROHC											2
#define CRC16_BUYPASS 									3
#define CRC16_ARC 											4
#define CRC16_MAXIM 										5
#define CRC16_DDS110 										6
#define CRC16_CMS 											7
#define CRC16_USB 											8
#define CRC16_MODBUS 										9
#define CRC16_XMODEM 										10
#define CRC16_GSM 											11
#define CRC16_KERMIT 										12
#define CRC16_AUG_CCITT 								13
#define CRC16_TMS37157 									14
#define CRC16_RIELLO 										15
#define CRC16_A 												16
#define CRC16_CCITT_FALSE 							17
#define CRC16_GENIBUS 									18
#define CRC16_MCRF4XX 									19
#define CRC16_X25 											20
#define CRC32_POSIX 										21
#define CRC32_MPEG2 										22
#define CRC32 													23
#define CRC32_BZIP2 										24
#define CRC32_JAMCRC 										25

typedef struct{
    uint32_t InputDataInverse;
    uint32_t InputDataEndian;
    uint32_t InputDataReverse;
    uint32_t CRC_Polynomial;
    uint32_t CRC_InputDataWidth;
    uint32_t OutputDataReverse;
    uint32_t CRC_InitialValue;
		uint32_t CRC_Type;
}GPL_CRC_InitTypedef;

uint8_t DataPattern[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};//CRC校验数据包
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
	CSC_GetSingleFlagStatus(CSC_XOSCF);     // 确认晶振(XOSC)振荡已稳定	
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
       
        // DMA channel select
        DMATestPattern.DMAChx = DMAChannel0;
        
        // channel x source/destination auto increase address
        DMATestPattern.SrcSINCSel = ENABLE;
        DMATestPattern.DestDINCSel = DISABLE;
        
        // DMA source peripheral config
        DMATestPattern.SrcSymSel = DMA_MEM_Read;
        
        // DMA destination peripheral config
        DMATestPattern.DestSymSel = DMA_GPL_Write;
        
        // DMA Burst size config
        DMATestPattern.BurstDataSize = DMA_BurstSize_1Byte;
        
        // DMA transfer data count initial number
        DMATestPattern.DMATransferNUM = 9;				//传输数据量，单位为字节，并不是多少个数据包
    
        // source/destination config
        DMATestPattern.DMASourceAddr = &DataPattern;
//				DMATestPattern.DMADestinationAddr = &GPL->DIN;
				
				DMA_Channel_Cmd(DMAChannel0, ENABLE);
				DMA_Base_Init(&DMATestPattern);
}

/**
 *******************************************************************************
 * @brief       GPL_CRC初始化			    
 * @details      初始化GPL_CRC
 * @param[in]   GPL_CRC_InitTypedef * GPL_CRC
 * @return		No
 * @note 
 * @par         Example
 * @code
   GPL_CRC_Config(GPL_CRC_InitTypedef * GPL_CRC);
 * @endcode
 *******************************************************************************
 */
void GPL_CRC_Config(GPL_CRC_InitTypedef * GPL_CRC)
{

    GPL->CR1.W = 0;
    GPL->CR0.W = 0;
    GPL->DIN.W = 0;
    GPL->CR0.W = GPL_CRC->InputDataInverse | 
                 GPL_CRC->InputDataEndian | 
                 GPL_CRC->InputDataReverse;

    GPL->CRCINIT.W = GPL_CRC->CRC_InitialValue;

    GPL->CR1.W = GPL_CRC->CRC_Polynomial |
                 GPL_CRC->CRC_InputDataWidth |
                 GPL_CRC->OutputDataReverse;

    GPL->CR1.B[0] |= GPL_CR1_CRC_EN_mask_b0;    
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
 * @brief       CRC校验函数			    
 * @details     进行各种CRC校验公式的处理
 * @param[in]   No
 * @return		  No
 * @note 
 * @par         Example
 * @code
   void GPL_CRC_Check(0x78);
 * @endcode
 *******************************************************************************
 */
void GPL_CRC_Check(void)
{
		GPL_CRC_InitTypedef lGPL_CRC;
		UnProtectModuleReg(CSCprotect);
		CSC_PeriphOnModeClock_Config(CSC_ON_GPL,ENABLE);
		ProtectModuleReg(CSCprotect);
	
		lGPL_CRC.CRC_Type = CRC32_POSIX; 							//选择你要使用的校验类型
	
		switch(lGPL_CRC.CRC_Type)
		{
			case CRC_8:// width=8 poly=0x07 init=0x00 refin=false refout=false xorout=0x00 check=0xf4 residue=0x00 name="CRC-8"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.CRC_InitialValue = 0UL;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x07;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
							
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);

							if(GPL->DOUT.B[0] != 0xF4)
									printf("Error!. CRC8 0x07 not equal is 0xF4, Now is 0x%2X Fail.\n\r", GPL->DOUT.B[0]);
							else
									printf("CRC8 0x07 Check OK\n\r");
							break;
							
			case CRC_8_ITU:// width=8 poly=0x07 init=0x00 refin=false refout=false xorout=0x55 check=0xa1 residue=0xac name="CRC-8/ITU"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.CRC_InitialValue = 0UL;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x07;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
							
							if((GPL->DOUT.B[0] ^ 0x55) != 0xA1)
									printf("Error!. CRC8 0x07 ITU not equal is 0xA1, Now is 0x%2X Fail.\n\r", (GPL->DOUT.B[0] ^ 0x55));  
							else
									printf("CRC8 0x07 ITU Check OK\n\r");
							break;
							
			case CRC_8_ROHC:// width=8 poly=0x07 init=0xff refin=true refout=true xorout=0x00 check=0xd0 residue=0x00 name="CRC-8/ROHC"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.CRC_InitialValue = 0xFFUL;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x07;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_BYTE;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/							
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);

							if(GPL->DOUT.B[0] != 0xD0)
									printf("Error!. CRC8 0x07 ROHC not equal is 0xD0, Now is 0x%2X Fail.\n\r", GPL->DOUT.B[0]);
							else
									printf("CRC8 0x07 ROHC Check OK\n\r");	
							break;
							
			case CRC16_BUYPASS:							// width=16 poly=0x8005 init=0x0000 refin=false refout=false xorout=0x0000 check=0xfee8 residue=0x0000 name="CRC-16/BUYPASS"

							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.CRC_InitialValue = 0UL;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x8005;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if(GPL->DOUT.H[0] != 0xFEE8)
									printf("Error!. CRC16 0x8005 BUYPASS not equal 0xFEE8, Now is 0x%4X Fail.\n\r", GPL->DOUT.H[0]);				
							else
									printf("CRC16 0x8005 BUYPASS Check OK\n\r");	
							break;
							
			case CRC16_ARC:				    // width=16 poly=0x8005 init=0x0000 refin=true refout=true xorout=0x0000 check=0xbb3d residue=0x0000 name="ARC"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x8005;
							lGPL_CRC.CRC_InitialValue = 0UL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_HALFWORD;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/

							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if(GPL->DOUT.H[0] != 0xBB3D)
									printf("Error!. CRC16 0x8005 ARC not equal 0xBB3D, Now is 0x%4X Fail.\n\r", GPL->DOUT.H[0]); 
							else
									printf("CRC16 0x8005 ARC Check OK\n\r");	
							break;
							
			case CRC16_MAXIM:// width=16 poly=0x8005 init=0x0000 refin=true refout=true xorout=0xffff check=0x44c2 residue=0xb001 name="CRC-16/MAXIM"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.CRC_InitialValue = 0UL;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x8005;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_HALFWORD;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);			
			
							if((GPL->DOUT.H[0] ^ 0xFFFF) != 0x44C2)
									printf("Error!. CRC16 0x8005 MAXIM not equal 0x44C2, Now is 0x%4X Fail.\n\r", GPL->DOUT.H[0]);
							else
									printf("CRC16 0x8005 MAXIM Check OK\n\r");	
							break;
							
			case CRC16_DDS110:// width=16 poly=0x8005 init=0x800d refin=false refout=false xorout=0x0000 check=0x9ecf residue=0x0000 name="CRC-16/DDS-110"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x8005;
							lGPL_CRC.CRC_InitialValue = 0x800DUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if(GPL->DOUT.H[0] != 0x9ECF)
									printf("Error!. CRC16 0x8005 DDS-110 not equal 0x9ECF, Now is 0x%4X Fail.\n\r", GPL->DOUT.H[0]);
							else
									printf("CRC16 0x8005 DDS-110 Check OK\n\r");	
							break;
				
			case CRC16_CMS:// width=16 poly=0x8005 init=0xffff refin=false refout=false xorout=0x0000 check=0xaee7 residue=0x0000 name="CRC-16/CMS"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x8005;
							lGPL_CRC.CRC_InitialValue = 0xFFFFUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if(GPL->DOUT.H[0] != 0xAEE7)
									printf("Error!. CRC16 0x8005 CMS not equal 0xAEE7, Now is 0x%4X Fail.\n\r", GPL->DOUT.H[0]);				
							else
									printf("CRC16 0x8005 CMS Check OK\n\r");	
							break;
							
			case CRC16_USB:// width=16 poly=0x8005 init=0xffff refin=true refout=true xorout=0xffff check=0xb4c8 residue=0xb001 name="CRC-16/USB"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x8005;
							lGPL_CRC.CRC_InitialValue = 0xFFFFUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_HALFWORD;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.H[0] ^ 0xFFFF) != 0xB4C8)
									printf("Error!. CRC16 0x8005 USB not equal 0xB4C8, Now is 0x%4X Fail.\n\r", GPL->DOUT.H[0]);
							else
									printf("CRC16 0x8005 USB Check OK\n\r");	
							break;	

			case CRC16_MODBUS:// width=16 poly=0x8005 init=0xffff refin=true refout=true xorout=0x0000 check=0x4b37 residue=0x0000 name="MODBUS"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x8005;
							lGPL_CRC.CRC_InitialValue = 0xFFFFUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_HALFWORD;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if(GPL->DOUT.H[0] != 0x4B37)
									printf("Error!. CRC16 0x8005 MODBUS not equal 0x4B37, Now is 0x%4X Fail.\n\r", GPL->DOUT.H[0]);
							else
									printf("CRC16 0x8005 MODBUS Check OK\n\r");	
							break;				

			case CRC16_XMODEM:// width=16 poly=0x1021 init=0x0000 refin=false refout=false xorout=0x0000 check=0x31c3 residue=0x0000 name="XMODEM"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x1021;
							lGPL_CRC.CRC_InitialValue = 0UL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);
							
/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if(GPL->DOUT.H[0] != 0x31C3)
									printf("Error!. CRC16 0x1021 XMODEM not equal 0x31C3, Now is 0x%4X Fail.\n\r", GPL->DOUT.H[0]);
							else
									printf("CRC16 0x1021 XMODEM Check OK\n\r");	
							break;								
							
			case CRC16_GSM:// width=16 poly=0x1021 init=0x0000 refin=false refout=false xorout=0xffff check=0xce3c residue=0x1d0f name="CRC-16/GSM"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x1021;
							lGPL_CRC.CRC_InitialValue = 0UL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);
/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.H[0] ^ 0xFFFF) != 0xCE3C)
									printf("Error!. CRC16 0x1021 GSM not equal 0xCE3C, Now is 0x%4X Fail.\n\r", (GPL->DOUT.H[0] ^ 0xFFFF));				
							else
									printf("CRC16 0x1021 GSM Check OK\n\r");	
							break;		
							
			case CRC16_KERMIT:// width=16 poly=0x1021 init=0x0000 refin=true refout=true xorout=0x0000 check=0x2189 residue=0x0000 name="KERMIT"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x1021;
							lGPL_CRC.CRC_InitialValue = 0UL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_HALFWORD;
							GPL_CRC_Config(&lGPL_CRC);
/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.H[0] ^ 0x0000) != 0x2189)
									printf("Error!. CRC16 0x1021 KERMIT not equal 0x2189, Now is 0x%4X Fail.\n\r", (GPL->DOUT.H[0] ^ 0x0000));	
							else
									printf("CRC16 0x1021 KERMIT Check OK\n\r");	
							break;	

			case CRC16_AUG_CCITT:// width=16 poly=0x1021 init=0x1d0f refin=false refout=false xorout=0x0000 check=0xe5cc residue=0x0000 name="CRC-16/AUG-CCITT"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x1021;
							lGPL_CRC.CRC_InitialValue = 0x1D0FUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);
/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.H[0] ^ 0x0000) != 0xE5CC)
									printf("Error!. CRC16 0x1021 AUG-CCITT not equal 0xE5CC, Now is 0x%4X Fail.\n\r", (GPL->DOUT.H[0] ^ 0x0000));
							else
									printf("CRC16 0x1021 AUG-CCITT Check OK\n\r");	
							break;	
							
			case CRC16_TMS37157:// width=16 poly=0x1021 init=0x89ec refin=true refout=true xorout=0x0000 check=0x26b1 residue=0x0000 name="CRC-16/TMS37157"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x1021;
							lGPL_CRC.CRC_InitialValue = 0x89ECUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_HALFWORD;
							GPL_CRC_Config(&lGPL_CRC);
/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.H[0] ^ 0x0000) != 0x26B1)
									printf("Error!. CRC16 0x1021 TMS37157 not equal 0x26B1, Now is 0x%4X Fail.\n\r", (GPL->DOUT.H[0] ^ 0x0000));				
							else
									printf("CRC16 0x1021 TMS37157 Check OK\n\r");	
							break;								
			case CRC16_RIELLO:// width=16 poly=0x1021 init=0xb2aa refin=true refout=true xorout=0x0000 check=0x63d0 residue=0x0000 name="CRC-16/RIELLO"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x1021;
							lGPL_CRC.CRC_InitialValue = 0xB2AAUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_HALFWORD;
							GPL_CRC_Config(&lGPL_CRC);
/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.H[0] ^ 0x0000) != 0x63D0)
									printf("Error!. CRC16 0x1021 RIELLO not equal 0x63D0, Now is 0x%4X Fail.\n\r", (GPL->DOUT.H[0] ^ 0x0000));				
							else
									printf("CRC16 0x1021 RIELLO Check OK\n\r");	
							break;			

			case CRC16_A:// width=16 poly=0x1021 init=0xc6c6 refin=true refout=true xorout=0x0000 check=0xbf05 residue=0x0000 name="CRC-A"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x1021;
							lGPL_CRC.CRC_InitialValue = 0xC6C6UL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_HALFWORD;
							GPL_CRC_Config(&lGPL_CRC);
/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.H[0] ^ 0x0000) != 0xBF05)
									printf("Error!. CRC16 0x1021 CRC-A not equal 0xBF05, Now is 0x%4X Fail.\n\r", (GPL->DOUT.H[0] ^ 0x0000));				
							else
									printf("CRC16 0x1021 CRC-A Check OK\n\r");	
							break;		

			case CRC16_CCITT_FALSE:// width=16 poly=0x1021 init=0xffff refin=false refout=false xorout=0x0000 check=0x29b1 residue=0x0000 name="CRC-16/CCITT-FALSE"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x1021;
							lGPL_CRC.CRC_InitialValue = 0xFFFFUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);
/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.H[0] ^ 0x0000) != 0x29B1)
									printf("Error!. CRC16 0x1021 CCITT-FALSE not equal 0x29B1, Now is 0x%4X Fail.\n\r", (GPL->DOUT.H[0] ^ 0x0000));
							else
									printf("CRC16 0x1021 CCITT-FALSE Check OK\n\r");	
							break;		

			case CRC16_GENIBUS:// width=16 poly=0x1021 init=0xffff refin=false refout=false xorout=0xffff check=0xd64e residue=0x1d0f name="CRC-16/GENIBUS"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x1021;
							lGPL_CRC.CRC_InitialValue = 0xFFFFUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);
/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.H[0] ^ 0xFFFF) != 0xD64E)
									printf("Error!. CRC16 0x1021 GENIBUS not equal 0xD64E, Now is 0x%4X Fail.\n\r", (GPL->DOUT.H[0] ^ 0xFFFF));
							else
									printf("CRC16 0x1021 GENIBUS Check OK\n\r");	
							break;	

		  case CRC16_MCRF4XX:// width=16 poly=0x1021 init=0xffff refin=true refout=true xorout=0x0000 check=0x6f91 residue=0x0000 name="CRC-16/MCRF4XX"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x1021;
							lGPL_CRC.CRC_InitialValue = 0xFFFFUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_HALFWORD;
							GPL_CRC_Config(&lGPL_CRC);
/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.H[0] ^ 0x0000) != 0x6F91)
									printf("Error!. CRC16 0x1021 MCRF4XX not equal 0x6F91, Now is 0x%4X Fail.\n\r", (GPL->DOUT.H[0] ^ 0x0000));
							else
									printf("CRC16 0x1021 MCRF4XX Check OK\n\r");	
							break;	
							
			case CRC16_X25:// width=16 poly=0x1021 init=0xffff refin=true refout=true xorout=0xffff check=0x906e residue=0xf0b8 name="X-25"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x1021;
							lGPL_CRC.CRC_InitialValue = 0xFFFFUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_HALFWORD;
							GPL_CRC_Config(&lGPL_CRC);
/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.H[0] ^ 0xFFFF) != 0x906E)
									printf("Error!. CRC16 0x1021 X-25 not equal 0x906E, Now is 0x%4X Fail.\n\r", (GPL->DOUT.H[0] ^ 0xFFFF));				
							else
									printf("CRC16 0x1021 X-25 Check OK\n\r");	
							break;
							
			case CRC32_POSIX:// width=32 poly=0x04c11db7 init=0x00000000 refin=false refout=false xorout=0xffffffff check=0x765e7680 residue=0xc704dd7b name="CRC-32/POSIX"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x4C11DB7;
							lGPL_CRC.CRC_InitialValue = 0UL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.W ^ 0xffffffff) != 0x765e7680)
									printf("Error!. CRC32 0x04c11db7 POSIX not equal 0x765e7680, Now is 0x%8X Fail.\n\r", (GPL->DOUT.W ^ 0xffffffff));				
							else
									printf("CRC32 0x04c11db7 POSIX Check OK\n\r");	
							break;
							
			case CRC32_MPEG2:// width=32 poly=0x04c11db7 init=0xffffffff refin=false refout=false xorout=0x00000000 check=0x0376e6e7 residue=0x00000000 name="CRC-32/MPEG-2"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x4C11DB7;
							lGPL_CRC.CRC_InitialValue = 0xFFFFFFFFUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if(GPL->DOUT.W != 0x0376e6e7UL)
									printf("Error!. CRC32 0x04c11db7 MPEG-2 not equal 0x0376e6e7, Now is 0x%8X Fail.\n\r", GPL->DOUT.W);				
							else
									printf("CRC32 0x04c11db7 MPEG-2 Check OK\n\r");	
							break;

			case CRC32:// width=32 poly=0x04c11db7 init=0xffffffff refin=true refout=true xorout=0xffffffff check=0xcbf43926 residue=0xdebb20e3 name="CRC-32"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x4C11DB7;
							lGPL_CRC.CRC_InitialValue = 0xFFFFFFFFUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_WORD;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.W ^ 0xffffffff) != 0xCBF43926UL)
									printf("Error!. CRC32 0x04c11db7 not equal 0xCBF43926, Now is 0x%8X Fail.\n\r", (GPL->DOUT.W ^ 0xffffffff));				
							else
									printf("CRC32 0x04c11db7 Check OK\n\r");	
							break;		

			case CRC32_BZIP2:// width=32 poly=0x04c11db7 init=0xffffffff refin=false refout=false xorout=0xffffffff check=0xfc891918 residue=0xc704dd7b name="CRC-32/BZIP2"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x4C11DB7;
							lGPL_CRC.CRC_InitialValue = 0xFFFFFFFFUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_NONE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_NONE;
							GPL_CRC_Config(&lGPL_CRC);

/*							lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
*/
			
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
			
							if((GPL->DOUT.W ^ 0xffffffff) != 0xFC891918UL)
									printf("Error!. CRC32 0x04c11db7 BZIP2 not equal 0xFC891918, Now is 0x%8X Fail.\n\r", (GPL->DOUT.W ^ 0xffffffff));
							else
									printf("CRC32 0x04c11db7 BZIP2 Check OK\n\r");	
							break;

			case CRC32_JAMCRC:// width=32 poly=0x04c11db7 init=0xffffffff refin=true refout=true xorout=0x00000000 check=0x340bc6d9 residue=0x00000000 name="JAMCRC"
							lGPL_CRC.InputDataInverse = GPL_InputDataInverse_DISABLE;
							lGPL_CRC.InputDataEndian = GPL_InputDataEndian_LITTLE;
							lGPL_CRC.CRC_InputDataWidth = GPL_CRC_InputDataWidth_8bit;
							lGPL_CRC.CRC_Polynomial = GPL_CRC_Polynomial_0x4C11DB7;
							lGPL_CRC.CRC_InitialValue = 0xFFFFFFFFUL;
							lGPL_CRC.InputDataReverse = GPL_InputDataReverse_BYTE;
							lGPL_CRC.OutputDataReverse = GPL_OutputDataReverse_WORD;
							GPL_CRC_Config(&lGPL_CRC);

					/*    lCount = 0;
							do{
									GPL->DIN.B[0] = DataPattern[lCount];
							}while(++ lCount < 9);
					*/
								
							GPL_DMA_Cmd(ENABLE);
						  while (DMA_GetSingleFlagStatus(DMA, DMA_FLAG_CH0_TCF) == DRV_UnHappened);
							DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
								
							if((GPL->DOUT.W ^ 0x00000000) != 0x340BC6D9UL)
									printf("Error!. CRC32 0x04c11db7 JAMCRC not equal 0x340BC6D9, Now is 0x%8X Fail.\n\r", (GPL->DOUT.W ^ 0x00000000));				
							else
									printf("CRC32 0x04c11db7 JAMCRC Check OK\n\r");	
							break;												
		}

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
	CSC_Init();
	TICK_Init();
	GPIO_Init();
	URT0_Init();
	DMA_Init();
	
	IO_LED_G_0=0;IO_LED_R=0;IO_LED_G_1=0;
	Delay(100);
	printf("\nHello!");
	IO_LED_G_0=1;IO_LED_R=1;IO_LED_G_1=1;

	DMA_ClearFlag(DMA, DMA_FLAG_CH0_TCF);
	DMA_StartRequest(DMAChannel0);
	GPL_CRC_Check();
	while(1);
}
/*
*************************************************************************************
*/ 


