/**
  ******************************************************************************
 *
 * @file        main.c
 *
 * @brief       Real Time Clock
 *
 * @par         Project
 *              MG32x02z
 *							功能描述：
 *							利用RTC唤醒处于STOP状态的MCU
 *							此demo会在PD8（LED）闪烁5次后进入STOP模式，
 *							RTC报警中断发生时唤醒MCU，并让MCU继续从睡眠
 *							处继续执行代码。
 *							RTC中断类型为系统中断，因此RTC的中断程序应写入
 *							SYS_IRQHandler(void)中。
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

#define RTC_ALARM_VALUE	  20			//设定RTC报警比较值，计数到达该值时触发报警
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
 * @brief       系统中断处理入口			    
 * @details     系统中断服务函数
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void SYS_IRQHandler(void);
 * @endcode
 *******************************************************************************
 */
void SYS_IRQHandler(void)
{
  if(PW_GetSingleFlagStatus(PW_WKF) == DRV_Happened)
  {
		IO_LED_G_1=~IO_LED_G_1;
    PW_ClearFlag(PW_WKF);
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
 * @brief       电源策略初始化			    
 * @details      初始化电源策略，保证RTC在STOP模式下运行
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void PWR_Init(void);
 * @endcode
 *******************************************************************************
 */
void PWR_Init()
{
	UnProtectModuleReg(PWprotect);
	NVIC_EnableIRQ(SYS_IRQn);
	SYS_ITEA_Cmd(ENABLE);                        //use it to enable SYS_IRQHandler
	PW_StopModeLDO_Select(PW_LowPower_LDO);
	PW_OnModeLDO_Select(PW_Normal_LDO);
	PW_PeriphStopModeWakeUp_Config(PW_WKSTP_RTC, ENABLE);
	PW_IT_Config(PW_INT_WK , ENABLE);
	PW_ClearFlag(PW_INT_WK);
	PW_ITEA_Cmd(ENABLE);
	ProtectModuleReg(PWprotect);
}

/**
 *******************************************************************************
 * @brief       RTC初始化			    
 * @details      初始化RTC
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void RTC_Init(void);
 * @endcode
 *******************************************************************************
 */
void RTC_Init (void)
{

    /*=== 1. Enable CSC to RTC clock ===*/
    UnProtectModuleReg(CSCprotect);                                 // Unprotect CSC module
	
		CSC_XOSCGain_Select(Gain_Low);
		CSC_CK_LS_Select(LS_CK_XOSC);   
		CSC_PeriphOnModeClock_Config(CSC_ON_PortC,ENABLE);
		CSC_XOSC_Cmd(ENABLE);    																			   //使能XOSC前必须先执行CSC_PeriphOnModeClock_Config(CSC_ON_PortC,ENABLE);
		CSC_GetSingleFlagStatus(CSC_XOSCF);     // 确认晶振(XOSC)振荡已稳定
		/* CK_LS setting */
		CSC_PeriphSleepModeClock_Config(CSC_SLP_RTC, ENABLE);
		CSC_PeriphStopModeClock_Config(CSC_STP_RTC, ENABLE);
		CSC_PeriphOnModeClock_Config(CSC_ON_RTC, ENABLE);               // Enable RTC module clock
    
    ProtectModuleReg(CSCprotect);                                   // protect CSC module
      
    /*=== 2. Configure RTC clock ===*/
    UnProtectModuleReg(RTCprotect);                                 // Unprotect RTC module
    RTC_CLK_Select(RTC_CK_LS);                                      // RTC clock source = CK_LS
    RTC_PreDivider_Select(RTC_PDIV_4096);                           // PDIV output = RTC clock / 4096
    RTC_Divider_Select(RTC_DIV_8);                                  // DIV output = (RTC clock / 4096) / 8
    RTC_OutputSignal_Select(RTC_ALM);                                // RTC output = DIV otuput frequency
    
    /*=== 3. Set RTC timer value ===*/
    RTC_RCR_Mode_Select(RTC_RCR_MOD_ForceReload);                   // RTC switch to reload mode
    RTC_SetReloadReg(0);                                             // Set reload data
    RTC_TriggerStamp_SW();                                          // Trigger reload data update to RTC timer
    while(RTC_GetSingleFlagStatus(RTC_RCRF) == DRV_UnHappened);     // Waiting reload complete
    RTC_ClearFlag(RTC_ALLF);                                        // Clear flag
    
    /*=== 4. Update ALM value ===*/
    if(RTC_GetAlarmState() == DRV_True)                             // When alarm function enable
    {
        RTC_Alarm_Cmd(DISABLE);                                     // Disable alarm function
        while(RTC_GetSingleFlagStatus(RTC_RCRF) == DRV_UnHappened); // Waiting alarm function disable  
        RTC_ClearFlag(RTC_RCRF);                                    // Clear flag RCRF
    }
    RTC_SetAlarmCompareValue(RTC_ALARM_VALUE);                      // 设定报警比较值，计数到达该值时触发报警
    RTC_Alarm_Cmd(ENABLE);                                          // Enable Alarm function
		
		RTC_StopModeWakeUpEvent_Config(RTC_ALM_WPEN, ENABLE);						//使能停止模式下使用RTC报警时唤醒MCU
		
//		RTC_IT_Config(RTC_INT_ALM,ENABLE);
//		RTC_ITEA_Cmd(ENABLE);
		PW_ClearFlag(PW_ALLF);
    RTC_ClearFlag(RTC_ALLF);
    /*=== 5. Enable RTC module ===*/
    RTC_Cmd (ENABLE);                                               // Enable RTC module
    ProtectModuleReg(RTCprotect);                                   // Protect RTC module

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
	
	CSC_Init();
	TICK_Init();
	GPIO_Init();
	URT0_Init();
	PWR_Init();
	RTC_Init();
	
	IO_LED_G_0=0;IO_LED_R=0;IO_LED_G_1=0;
	Delay(100);
	printf("\nHello!");
	IO_LED_G_0=1;IO_LED_R=1;IO_LED_G_1=1;

	while(1)
  {
		Delay(500);
		IO_LED_G_0=!IO_LED_G_0;
		i++;
		if(i>=10)
		{
			i=0;
			CM0_SysTick_Cmd(DISABLE);//进入睡眠前必须先禁用SysTick中断功能，不然会被瞬间唤醒
			IO_LED_G_0=1;	
			RTC_ClearFlag(RTC_ALLF);
			SCB->SCR  = (SCB_SCR_SLEEPDEEP_Msk);  //执行WFI指令前执行该指令可让MCU进入STOP模式
			__WFI();             //该指令为cmsis_arm.cc的指令， 执行该指令时MCU进入睡眠。
			CM0_SysTick_Cmd(ENABLE);
			UnProtectModuleReg(RTCprotect);
			RTC_SetReloadReg(0); 											// 设置重载值
			RTC_TriggerStamp_SW();											// Trigger reload data update to RTC timer
			while(RTC_GetSingleFlagStatus(RTC_RCRF) == DRV_UnHappened); 	// Waiting reload complete
			RTC_ClearFlag(RTC_ALLF);										// Clear flag
			ProtectModuleReg(RTCprotect);
		}
  }
}
/*
*************************************************************************************
*/ 


