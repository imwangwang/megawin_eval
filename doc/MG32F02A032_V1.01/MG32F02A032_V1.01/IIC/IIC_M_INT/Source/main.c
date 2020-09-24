/**
  ******************************************************************************
 *
 * @file        main.c
 *
 * @brief       I2C Interrupt for Master (I2C)
 *
 * @par         Project
 *              MG32x02z
 *				该Demo中使用I2C中断方式操作24C64

 * @version     V1.00
 * @date        2020/04/14
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

// I2C Frequency define
#define I2C_PR_FREQUENCY	48000000
#define I2C0_CLOCK_FREQ		400000
#define I2C1_CLOCK_FREQ		400000


// I2C Salve Address define
#define I2C0_SLAVE_ADDR_1	0xA0
#define I2C0_SLAVE_ADDR_2	0xA2

#define _I2C0_TX_TEST
#define _I2C0_RX_TEST

enum{
	EE_IDLE=0,
	EE_SEND_START,
	EE_SEND_SLA,
	EE_WRITE_ADDR_H,
	EE_WRITE_ADDR_L,
	EE_SEND_RESTART,
	EE_WRITE_DATA,
	EE_READ_DATA,
	EE_SEND_STOP
}EE_STATUS;

#define EE_SUCCESS		0x00

#define DATA_LENGTH          1024
u8 EEWriteBuf[DATA_LENGTH];
u8 EEReadBuf[DATA_LENGTH];

#define EE_OVTIME_MAX		20

u16 EEOvtime;

#define EE_MODE_WRITE		0
#define EE_MODE_READ		1

typedef struct 
{
	u8 EEMode;
	u8 EEStatus;
	u8 EEDeviceAddr;
	u16 EEDataAddr;
	u8 *EEDataBuf;
	u8 EEFlag;
	u16 EEDataInx;
	u16 EEDataLength;
}EEPROM_PROC_STRUCT;

EEPROM_PROC_STRUCT EEProm;


u8 Rand;
u16 LedTime;
u16 EETestTime;

const u8 TestCode[]={
	0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,
	0x0F,0x1E,0x2D,0x3C,0x4B,0x5A,0x69,0x78,
	0x87,0x96,0xA5,0xB4,0xC3,0xD2,0xE1,0xF0,
	0xFE,0xDC,0xBA,0x98,0x76,0x54,0x32,0x10,
};

enum{
	EE_TEST_IDLE,
	EE_TEST_WRITE,
	EE_TEST_WRITE_DONE,
	EE_TEST_READ,
	EE_TEST_READ_DONE
}EE_TEST;
u8 EETestStatus;
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
	if(LedTime!=0) LedTime--;
	if(EEOvtime!=0) EEOvtime--;
	if(EETestTime!=0) EETestTime--;
	
}


/**
 *******************************************************************************
 * @brief       I2C0 Interrupt Handler		    
 * @details     
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void I2C0_IRQHandler();
 * @endcode
 *******************************************************************************
 */
void I2C0_IRQHandler(void)
{
    uint8_t lState;
	lState=__I2C_GetStatusCode(I2C0);
	
	if(lState==0x08)
	{ // Tx:Start
		if(EEProm.EEStatus==EE_SEND_START)
		{
			Set_STA_STO_AA_000(I2C0);	
			__I2C_SendSBUF(I2C0,EEProm.EEDeviceAddr&0xFE);
			EEProm.EEStatus=EE_SEND_SLA;
		}
		else
		{
			EEProm.EEFlag=EEProm.EEStatus+0x80;
			EEProm.EEStatus=EE_SEND_STOP;
			Set_STA_STO_AA_010(I2C0);	
		}
	}
	else if(lState==0x18)
    { // Tx:SLA+W Rx:ACK
    	// To send RegAddr high
		Set_STA_STO_AA_001(I2C0);	
		__I2C_SendSBUF(I2C0,(u8)(EEProm.EEDataAddr>>8));
		EEProm.EEStatus=EE_WRITE_ADDR_H;
    }
    else if(lState==0x20)
    { // Tx:SLA+W Rx:NACK
		EEProm.EEFlag=EEProm.EEStatus+0x80;
		EEProm.EEStatus=EE_SEND_STOP;
		Set_STA_STO_AA_010(I2C0);	
    }
    else if(lState==0x28)
    { // Tx:DAT Rx:ACK
		if(EEProm.EEStatus==EE_WRITE_ADDR_H)
		{
			// To send RegAddr LOW
			Set_STA_STO_AA_001(I2C0);	
			__I2C_SendSBUF(I2C0,(u8)(EEProm.EEDataAddr));
			EEProm.EEStatus=EE_WRITE_ADDR_L;
		}
		else if(EEProm.EEStatus==EE_WRITE_ADDR_L)
		{
			// Send RegAddr Done
			if(EEProm.EEMode==EE_MODE_READ)
			{
				// Read mode: To Tx ReStart;
				Set_STA_STO_AA_100(I2C0);	
				EEProm.EEStatus=EE_SEND_RESTART;
			}
			else
			{
				// Write mode: To Tx Data;
				Set_STA_STO_AA_001(I2C0);	
				EEProm.EEStatus=EE_WRITE_DATA;
				EEProm.EEDataInx=0;
				__I2C_SendSBUF(I2C0, EEProm.EEDataBuf[EEProm.EEDataInx]);
				EEProm.EEDataInx++;
			}
		}
		else if(EEProm.EEStatus==EE_WRITE_DATA)
		{
			if(EEProm.EEDataInx>=EEProm.EEDataLength)
			{
				Set_STA_STO_AA_010(I2C0);	// 发送STOP
				EEProm.EEFlag= EE_SUCCESS;
				EEProm.EEStatus=EE_SEND_STOP;
			}
			else
			{
				__I2C_SendSBUF(I2C0, EEProm.EEDataBuf[EEProm.EEDataInx]);
				EEProm.EEDataInx++;
			}
		}
		else
		{
			Set_STA_STO_AA_010(I2C0);	// 发送STOP
			EEProm.EEFlag=EEProm.EEStatus+0x80;
			EEProm.EEStatus=EE_SEND_STOP;
		}
    }
    else if(lState==0x30)
    { // Tx:DAT Rx:NACK
		EEProm.EEFlag= EEProm.EEStatus+0x80;
		if(EEProm.EEStatus==EE_WRITE_DATA)
		{
			if(EEProm.EEDataInx==EEProm.EEDataLength)
			{
				EEProm.EEFlag= EE_SUCCESS;
			}
		}
		Set_STA_STO_AA_010(I2C0);	// 发送STOP
		EEProm.EEStatus=EE_SEND_STOP;
    }
	else if(lState==0x10)
	{ // Tx:ReStart
		if(EEProm.EEMode==EE_MODE_READ)
		{ 
			Set_STA_STO_AA_000(I2C0);	
			__I2C_SendSBUF(I2C0,EEProm.EEDeviceAddr|0x01);
			EEProm.EEStatus=EE_SEND_SLA;
		}
		else
		{
			EEProm.EEFlag=EEProm.EEStatus+0x80;
			EEProm.EEStatus=EE_SEND_STOP;
			Set_STA_STO_AA_010(I2C0);	
		}
	}
    else if(lState==0x40)
    { // Tx:SLA+R Rx:ACK
		Set_STA_STO_AA_001(I2C0);	
		EEProm.EEDataInx=0;
		EEProm.EEStatus=EE_READ_DATA;
    }
    else if(lState==0x48)
    { // Tx:SLA+R Rx:NACK
		EEProm.EEFlag=EEProm.EEStatus+0x80;
		EEProm.EEStatus=EE_SEND_STOP;
		Set_STA_STO_AA_010(I2C0);	
    }
    else if(lState==0x50)
    { // Rx:DAT Tx:ACK
		if(EEProm.EEStatus==EE_READ_DATA)
		{
			if(EEProm.EEDataInx>=EEProm.EEDataLength)
			{
				Set_STA_STO_AA_010(I2C0);	// 发送STOP
				EEProm.EEFlag= EE_SUCCESS;
				EEProm.EEStatus=EE_SEND_STOP;
			}
			else
			{
				EEProm.EEDataBuf[EEProm.EEDataInx]=__I2C_ReceiveSBUF(I2C0);
				EEProm.EEDataInx++;
				if(EEProm.EEDataInx>=EEProm.EEDataLength-1)
				{
					Set_STA_STO_AA_000(I2C0);
				}
				else
				{
					Set_STA_STO_AA_001(I2C0);
				}
			}
		}
		else
		{
			EEProm.EEFlag=EEProm.EEStatus+0x80;
			EEProm.EEStatus=EE_SEND_STOP;
			Set_STA_STO_AA_010(I2C0);	
		}
    }
    else if(lState==0x58)
    { // Rx:DAT Tx:NACK
		EEProm.EEFlag= EEProm.EEStatus+0x80;
		if(EEProm.EEStatus==EE_READ_DATA)
		{
			EEProm.EEDataBuf[EEProm.EEDataInx]=__I2C_ReceiveSBUF(I2C0);
			EEProm.EEDataInx++;
			if(EEProm.EEDataInx==EEProm.EEDataLength)
			{
				EEProm.EEFlag= EE_SUCCESS;
			}
		}
		Set_STA_STO_AA_010(I2C0);	// 发送STOP
		EEProm.EEStatus=EE_SEND_STOP;
    }
	EEOvtime=EE_OVTIME_MAX;
	 // to do..
	 __I2C_ClearFlag(I2C0,I2C_FLAG_EVENTF);
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
 * @brief  	    I2C reset. 
 * @details  
 * @param[in]   I2Cx:
 * 	@arg\b			I2C0.
 * @return	    DRV_Return	
 * @see         DRV_Success : True
 * @see         DRV_Failure : False
 * @note
 * @par         Example
 * @code
    I2C_DeInit(I2C0);
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
 * @brief       I2C0初始化			    
* @details      初始化I2C0使用引脚   SCL:PB10  ,SDA:PB11
 * @param[in]   No
 * @return		No
 * @note 
 * @par         Example
 * @code
   void I2C0_Init(void);
 * @endcode
 *******************************************************************************
 */
void I2C0_Init(void)
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
		
    I2C_SetSlaveAddress(I2C0, I2C_SADR_1,I2C0_SLAVE_ADDR_1);
    I2C_SetSlaveAddress(I2C0, I2C_SADR_2,I2C0_SLAVE_ADDR_2);
    
	//===== I2C Interrupt Config =====//
	 NVIC_EnableIRQ(I2C0_IRQn);
	 NVIC_SetPriority(I2C0_IRQn, 1);										  // Suggest SYSTICK Priority = 0
																					  //		   Other Priority > 0
	
	__I2C_ITEA_Disable(I2C0);
	__I2C_IT_Disable(I2C0, (I2C_IT_TMOUT | I2C_IT_EVENT | I2C_IT_ERR | I2C_IT_BUF | I2C_IT_WUP));

    __I2C_IT_Enable(I2C0, ( I2C_IT_EVENT ));
    __I2C_ITEA_Enable(I2C0);
    
	//===== I2C Timeout Config =====//
	__I2C_TMO_Enable(I2C0);
	__I2C_SetTimeOutClockSource(I2C0, I2C_TMO_CKS_DIV64);
	__I2C_SetTimeOutDetectionMode(I2C0, I2C_TMO_MDS_GENERAL);
	__I2C_SetTimeOutCount(I2C0, I2C_TMO_MDS_GENERAL);

	EEProm.EEStatus=EE_IDLE;
	EEOvtime=EE_OVTIME_MAX;

	//===== I2C Enable =====//
	__I2C_Enable(I2C0);

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
	u16 Inx;
	u16 i=0;
	
	CSC_Init();
	TICK_Init();
	GPIO_Init();
	URT0_Init();
	
	I2C0_Init(); 

	
	IO_LED_G_0=0;IO_LED_R=0;IO_LED_G_1=0;
	Delay(1000);
	printf("\nHello!");
	IO_LED_G_0=1;IO_LED_R=1;IO_LED_G_1=1;


	EEProm.EEStatus=EE_IDLE;
	
	EETestStatus=EE_TEST_IDLE;
	LedTime=100;
	EETestTime=1000;
	while(1)
  {
		Rand++;
		if(LedTime==0)
		{
			IO_LED_G_0=!IO_LED_G_0;
			LedTime=100;
		}

		switch(EETestStatus)
		{
			case EE_TEST_IDLE:
				if(EETestTime==0)
				{
					EETestStatus=EE_TEST_WRITE;
					printf("\nI2C0 Write EEPROM 1K Byte Test...");
					i=0;
					for(Inx=0;Inx<DATA_LENGTH;Inx++)
					{
						EEWriteBuf[Inx]=TestCode[i]+Rand;
						Rand++;
						i++;
						if(i>=32) i=0;
					}
					EEProm.EEMode=EE_MODE_WRITE;
					EEProm.EEDataBuf=EEWriteBuf;
					EEProm.EEDeviceAddr=I2C0_SLAVE_ADDR_1;
					EEProm.EEDataAddr=0x0000;
					EEProm.EEDataLength=32;
					EEProm.EEStatus=EE_SEND_START;
					Inx=0;
					// start I2C
					EEOvtime=EE_OVTIME_MAX;
					Set_STA_STO_AA_100(I2C0);	
				}
				break;
			case EE_TEST_WRITE:
				if(EEProm.EEStatus == EE_SEND_STOP)
				{
					if(EEProm.EEFlag != EE_SUCCESS)
					{
						printf("Err: 0x%04X",EEProm.EEFlag);
						EETestStatus=EE_TEST_IDLE;
						EETestTime=1000;
					}
					else
					{
						if(Inx<(DATA_LENGTH-32))
						{
							Inx=Inx+32;
							Delay(5);
							EEProm.EEDataBuf=EEWriteBuf+Inx;
							EEProm.EEDeviceAddr=I2C0_SLAVE_ADDR_1;
							EEProm.EEDataAddr=Inx;
							EEProm.EEStatus=EE_SEND_START;
							// start I2C
							EEOvtime=EE_OVTIME_MAX;
							Set_STA_STO_AA_100(I2C0);	
						}
						else
						{
							printf("Done!");
							EETestStatus=EE_TEST_WRITE_DONE;
							EETestTime=1000;
						}
					}
				}
				else if(EEOvtime==0)
				{ // 
					printf("Fail-->IC20_Init..");
					I2C0_Init();
					EEProm.EEStatus=EE_IDLE;
					EETestStatus=EE_TEST_IDLE;
					EETestTime=1000;
				}
				break;
			case EE_TEST_WRITE_DONE:
				if(EETestTime==0)
				{
					EETestStatus=EE_TEST_READ;
					printf("\nI2C0 Read EEPROM 1K Byte Test...");
					EEProm.EEMode=EE_MODE_READ;
					EEProm.EEDataBuf=EEReadBuf;
					EEProm.EEDeviceAddr=I2C0_SLAVE_ADDR_1;
					EEProm.EEDataAddr=0x0000;
					EEProm.EEDataLength=DATA_LENGTH;
					EEProm.EEStatus=EE_SEND_START;
					Inx=0;
					// start I2C
					EEOvtime=EE_OVTIME_MAX;
					Set_STA_STO_AA_100(I2C0);	
				}
				break;
			case EE_TEST_READ:
				if(EEProm.EEStatus == EE_SEND_STOP)
				{
					if(EEProm.EEFlag != EE_SUCCESS)
					{
						printf("Err: 0x%04X",EEProm.EEFlag);
						EETestStatus=EE_TEST_IDLE;
						EETestTime=1000;
					}
					else
					{
						for(Inx=0;Inx<DATA_LENGTH;Inx++)
						{
							if(EEWriteBuf[Inx]!=EEReadBuf[Inx]) break;
						}
						if(Inx>=DATA_LENGTH)
						{// 校验成功
							IO_LED_G_1=0;
							printf("Success!!");
						}
						else
						{// 校验失败
							printf("Verify Fail!! 0x%04X",Inx);
						}
						EETestStatus=EE_TEST_IDLE;
						EETestTime=1000;
					}
				}
				else if(EEOvtime==0)
				{ // 
					printf("Fail-->IC20_Init..");
					I2C0_Init();
					EEProm.EEStatus=EE_IDLE;
					EETestStatus=EE_TEST_IDLE;
					EETestTime=1000;
				}
				break;
			default:
				EETestStatus=EE_TEST_IDLE;
				break;
		}

  }
  
}
/*
*************************************************************************************
*/ 


