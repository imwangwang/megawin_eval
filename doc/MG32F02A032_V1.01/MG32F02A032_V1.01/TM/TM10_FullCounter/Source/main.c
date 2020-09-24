/**
  ******************************************************************************
 *
 * @file        main.c
 *
 * @brief       Timer10
 *
 * @par         Project
 *              MG32x02z
 *							��demoּ������TM10��ȫ���������ܣ���ѧ��
 *							������ʱ��Ƶ�ʡ�
 *							��demo���ڶ�ʱ�����ʱ��תPD9 IO״̬��
 *							��PA0Ϊ�͵�ƽʱ������ʱ�����ڡ�
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

#define TM10_PRE_2nd_RELOAD			(u16)((u32)(200*1000*SYS_CLOCK/1-1)%65536)		// 200ms 200ms�������Ҫ�ļ�����ֵ���ǣ�
																				//ÿ12MHz��ʱ�������ֵ��+1,Ҳ����1��T=1/f����ô200ms��Ҫ�������Ӷ��٣�����(200*1000)/T   %65536��ȡ��16λ /�Ǹ�16λ
#define TM10_MAIN_RELOAD			(u16)((u32)(200*1000*SYS_CLOCK/1-1)/65536)		// 200ms

/*
*************************************************************************************
* Interrupt Handler
*
*************************************************************************************
*/
/**
 *******************************************************************************
 * @brief       ϵͳTick �жϴ������			    
 * @details     ϵͳTick �жϷ�����
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
 * @brief       TM10�жϴ������			    
 * @details     TM10�жϷ�����
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void TM10_IRQHandler(void);
 * @endcode
 *******************************************************************************
 */
void TM10_IRQHandler(void)
{
	if (TM_GetSingleFlagStatus(TM10, TMx_TOF) == DRV_Happened)	   // Main Timer overflow flag
	{
		IO_LED_R=!IO_LED_R;
		TM_ClearFlag (TM10, TMx_TOF);
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
 * @brief       ϵͳʱ�ӳ�ʼ��			    
 * @details     ϵͳʱ�ӳ�ʼ��Ϊϵͳʱ��48MHz
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
 * @brief       ϵͳ�δ�ʱ�ӳ�ʼ��			    
 * @details     ϵͳ�δ�ʱ�ӳ�ʼ��������Delay()����
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
 * @brief       TM10��ʼ��			    
 * @details     ��ʼ��TM10
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void TM10_Init(void);
 * @endcode
 *******************************************************************************
 */
void TM10_Init(void)
{	
	TM_TimeBaseInitTypeDef TM_TimeBase_InitStruct;
	 
	//==Set TM1x Clock
	UnProtectModuleReg(CSCprotect);
	CSC_PeriphProcessClockSource_Config(CSC_TM10_CKS, CK_APB);
	CSC_PeriphOnModeClock_Config(CSC_ON_TM10,ENABLE);						// Enable TM10 Clock
	ProtectModuleReg(CSCprotect);					
		
	TM_DeInit(TM10);
			
	// ----------------------------------------------------
	// Set TimeBase structure parameter
	TM_TimeBase_InitStruct.TM_CounterMode = Full_Counter;					// ȫ������ģʽ
	TM_TimeBase_InitStruct.TM_MainClockSource = TM_CK_INT;					// ��������Դ: (ȫ����ģʽ��,��������)
	TM_TimeBase_InitStruct.TM_2ndClockSource = TM_CK_INT;					// Ԥ��Ƶ��(2nd������)������Դ: TM_CK_INT
	TM_TimeBase_InitStruct.TM_MainClockDirection = TM_UpCount;				// �����������ϼ��� 
	TM_TimeBase_InitStruct.TM_2ndClockDirection = TM_UpCount;				// Ԥ��Ƶ��(2nd������)���ϼ���
		
	TM_TimeBase_InitStruct.TM_Prescaler = TM10_PRE_2nd_RELOAD;				// Ԥ��Ƶ��(2nd������)����ֵ	
	TM_TimeBase_InitStruct.TM_Period = TM10_MAIN_RELOAD;					// ������������ֵ
	TM_TimeBase_Init(TM10, &TM_TimeBase_InitStruct);
		 
	//== SET TM10 ClockOut
	TM_ClockOutSource_Select(TM10, MainCKO);
	TM_ClockOut_Cmd(TM10, ENABLE);	

	// ----------------------------------------------------
	// 2.clear TOF flag
	TM_ClearFlag(TM10, TMx_TOF);
			
	// ----------------------------------------------------
	// 3.Enable TM10/TM16 interrupt 
	TM_IT_Config(TM10,TMx_TIE_IE,ENABLE);									// ������������ж�ʹ��
		
	TM_ITEA_Cmd(TM10,ENABLE);

	// ----------------------------------------------------
	// Timer enable
	TM_Timer_Cmd(TM10,ENABLE);
		
	// ----------------------------------------------------
	// NVIC Enable TM10/TM16
	NVIC_EnableIRQ(TM10_IRQn); 

	
}

/**
 *******************************************************************************
 * @brief       GPIO��ʼ��			    
 * @details     ��ʼ������LED��GPIO
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
	PINX_InitStruct.PINX_Pin				 = (PX_Pin_8|PX_Pin_9|PX_Pin_10);//��ʼ��LED��GPIO
	GPIO_PortMode_Config(IOMD,&PINX_InitStruct); 					 		 
	
	PINX_InitStruct.PINX_Mode				 = PINX_Mode_OpenDrain_O; 	 	// Pin select open drain Output mode
	GPIO_PinMode_Config(PINA(0),&PINX_InitStruct);							//ģ�ⰴ��IO

}

/**
 *******************************************************************************
 * @brief       URT0��ʼ��			    
* @details      ��ʼ��URT0  ʹ������   TX:PC10  RX:PC11  �����ʣ�115200
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
 * @brief       fputc�����ض���			    
 * @details      fputc�����ض��򣬼�����printf�����Ϣ
 * @param[in]   ch:����printfʱ����Ҫ���������
 * @param[in] 	FILE *f:����printfʱ����Ҫ���������
 * @return		  ch:����printfʱ����Ҫ���������
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
 * @brief       ��׼�����������			    
 * @details     ���������������printf����
 * @param[in]   ch:��������
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
 * @brief       ����TM10����ֵ			    
 * @details     ����TM10����ֵ
 * @param[in]   dwFreq:����ֵ
 * @return		  No
 * @note 
 * @par         Example
 * @code
   void SetT1xReload(0x78);
 * @endcode
 *******************************************************************************
 */
void SetT1xReload(int dwFreq)
{
	DWordTypeDef dwT1Value;
	TM_TimeBaseInitTypeDef TM_TimeBase_InitStruct;
	dwFreq=dwFreq*2;		
	
	dwT1Value.DW=(SYS_CLOCK*1000000)/dwFreq-1;    //������TM1X����ֵһ������������ֵ���������������ֵ����0��ʼ�����ֵ�����Ҫ��T�������ǵ�1/dwFreq����
																					//ÿ48MHz��ʱ�������ֵ+1����1/48MHzʱ�������ֵ+1����ô�ܵ���Ҫ����ֵ����T/ϵͳƵ��ʱ�䣬�� ��1/dwFreq��/��1/48MHz�� -1
	TM_Counter_Config(TM10, 0, dwT1Value.W.WHigh);
	TM_Prescaler_Config(TM10, 0, dwT1Value.W.WLow);


	
}

/*
*************************************************************************************
*  MAIN
*
*************************************************************************************
*/
/**
 *******************************************************************************
 * @brief       ������			    
 * @details     ������		
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
	int i=0;
	CSC_Init();
	TICK_Init();
	TM10_Init();
	GPIO_Init();
	URT0_Init();
	
	IO_LED_G_0=0;IO_LED_R=0;IO_LED_G_1=0;
	Delay(100);
	printf("\nHello!");
	IO_LED_G_0=1;IO_LED_R=1;IO_LED_G_1=1;

	while(1)
  {
		if(PA0 == 0)			//��������
		{
			Delay(500);
			i+=5;
			SetT1xReload(i);
		}
		
  }
}
/*
*************************************************************************************
*/ 


