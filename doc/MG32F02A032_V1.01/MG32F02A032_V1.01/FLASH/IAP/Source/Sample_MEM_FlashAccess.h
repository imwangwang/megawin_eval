/**
 ******************************************************************************
 *
 * @file        Sample_MEM_FlashAccess.h
 *
 * @brief       Sample MEM Flash Access Head File 
 *
 * @par         Project
 *              MG32x02z
 * @version     V1.00
 * @date        2018/05/28
 * @author      Megawin Software Center
 * @copyright   Copyright (c) 2018 MegaWin Technology Co., Ltd.
 *              All rights reserved.
 *
 ******************************************************************************
 * @par 		Disclaimer 
 *		The Demo software is provided "AS IS"  without any warranty, either 
 *		expressed or implied, including, but not limited to, the implied warranties 
 *		of merchantability and fitness for a particular purpose.  The author will 
 *		not be liable for any special, incidental, consequential or indirect 
 *		damages due to loss of data or any other reason. 
 *		These statements agree with the world wide and local dictated laws about 
 *		authorship and violence against these laws. 
 ******************************************************************************
 */ 



#ifndef __Sample_MEM_FlashAccess_H
#define __Sample_MEM_FlashAccess_H
//#define __Sample_MEM_FlashAccess_H                            0.01

#ifdef __cplusplus
 extern "C" {
#endif



#include "MG32x02z_DRV.h"



/**
 * @name	Flash IAP Access
 *
 */ 
///@{
DRV_Return Sample_MEM_FlashIAPSingleProgram(uint32_t Address, uint32_t ProgramData);
DRV_Return Sample_MEM_FlashIAPProgram(uint32_t IAPStartAddress, uint32_t DataStartAddress, uint32_t Lenth);
DRV_Return Sample_MEM_FlashIAPPageErase(uint32_t StartPageAddress, uint32_t PageQuantity);

///@}

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif


