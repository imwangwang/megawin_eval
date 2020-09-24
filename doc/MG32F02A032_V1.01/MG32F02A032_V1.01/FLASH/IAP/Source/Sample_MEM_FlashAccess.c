



/**
 ******************************************************************************
 *
 * @file        Sample_MEM_FlashAccess.c
 *
 * @brief       This file is to IAP access
 *   
 * @par         Project
 *              MG32x02z
 * @version     V1.00
 * @date        2018/05/30
 * @author      Megawin Software Center
 * @copyright   Copyright (c) 2017 MegaWin Technology Co., Ltd.
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



#include "Sample_MEM_FlashAccess.h"

/**
 *******************************************************************************
 * @brief	    Memoey Flash IAP Opration
 * @details     1.Set IAP Size 4KByte
 *    \n        2.IAP Erase 4KByte
 *    \n        3.IAP Single Program
 *    \n        4.IAP Continuous Program
 * @note        MG32x02z Flash Program Data Width 32Bits
 *              MG32x02z 1Page = 1024Byte
 *******************************************************************************
 */
#pragma pack(push)
#pragma pack(4)
uint8_t const Value0[]={0x89, 0xAB, 0xCD, 0xEF, 0x76, 0x54, 0x32, 0x10,
                        0xFE, 0xDC, 0xBA, 0x98, 0x01, 0x23, 0x34, 0x67,};
#pragma pack(pop)

void Sample_MEM_FlashIAPAccess(void)
{
    MEM_SetIAPSize(4096);
    Sample_MEM_FlashIAPPageErase(0x1A000000, 4);

    Sample_MEM_FlashIAPSingleProgram(0x1A000800UL, 0x89ABCDEFUL);
    Sample_MEM_FlashIAPSingleProgram(0x1A000804UL, 0x76543210UL);
    Sample_MEM_FlashIAPSingleProgram(0x1A000808UL, 0xFEDCBA98UL);
    Sample_MEM_FlashIAPSingleProgram(0x1A00080CUL, 0x01234567UL);

    Sample_MEM_FlashIAPProgram(0x1A000000UL, (uint32_t)(&Value0), (sizeof(Value0) / 4));
}



/**
 *******************************************************************************
 * @brief       Flash IAP Erase
 * @details     1. Check parameter
 *      \n      2. Enable Erase Function
 *      \n      3. Flash erase and confirm correct or not 
 *      \n      4. Disable Erase Function
 * @param[in]   StartPageAddress : IAP Start Address 0x1A000000
 * @param[in]   PageQuantity : MG32x02z 1Page = 1024Byte
 * @return      DRV_Success : 
 * @return      DRV_Failure : 
 * @exception   None
 * @note        
 * @par         Example
 * @code        
                Sample_MEM_FlashIAPPageErase(0x1A000000UL, 4);
 * @endcode     
 * @par         Modify
 *              DRV_Return Sample_MEM_FlashIAPPageErase(uint32_t StartPageAddress, uint32_t PageQuantity)
 *******************************************************************************
 */
DRV_Return Sample_MEM_FlashIAPPageErase(uint32_t StartPageAddress, uint32_t PageQuantity)
{
    DRV_Return lDRV_Return = DRV_Success;
    uint32_t lCount;
    uint32_t *lptrDest_addr;

    // Check IHRCO Enable
    if(CSC->CR0.MBIT.IHRCO_EN == 0)
        return DRV_Failure;

    // Check flash page erase address alignment
    if(((uint32_t)StartPageAddress & 0x03FF) != 0)
        return DRV_Failure;

    if(((StartPageAddress) < 0x1A000000) || ((StartPageAddress) >= 0x1C000000))
        return DRV_Failure;

    lptrDest_addr = (uint32_t *)StartPageAddress;
    lCount = 0;

    __MEM_UnProtect();
    __MEM_Enable();
    __MEM_Access_Enable(MEM_ACCESS_IAP_WRITE);
    __MEM_SetWriteMode(IAPErase);
    __MEM_MultipleWriteUnProtect();

    do{
        // Mem Flag Clear.
        MEM->STA.B[0] = (MEM_STA_WPEF_mask_b0 | MEM_STA_EOPF_mask_b0);
        __ISB();
        *lptrDest_addr = 0xFFFFFFFF;

        if((MEM->STA.B[0] & (MEM_STA_WPEF_mask_b0 | MEM_STA_EOPF_mask_b0)) != MEM_STA_EOPF_mask_b0)
        {
            lDRV_Return = DRV_Failure;
            break;
        }
        lptrDest_addr += 0x100;
    }while(++lCount < PageQuantity);

    __MEM_UnProtect();
    __MEM_MultipleWriteProtect();
    __MEM_SetWriteMode(None);
    __MEM_Access_Disable(MEM_ACCESS_IAP_WRITE);
    __MEM_Disable();
    __MEM_Protect();

    return lDRV_Return;
}



/**
 *******************************************************************************
 * @brief       Flash IAP Single Program
 * @details     1. Check parameter
 *      \n      2. Enable Single Program Function
 *      \n      3. Flash Program 
 *      \n      4. Disable Erase Function
 *      \n      5. Confirm correct or not 
 * @param[in]   Address : IAP Start Address 0x1A000000
 * @param[in]   ProgramData : MG32x02z 1Page = 1024Byte
 * @return      DRV_Success : 
 * @return      DRV_Failure : 
 * @exception   None
 * @note        
 * @par         Example
 * @code        
                Sample_MEM_FlashIAPSingleProgram(0x1A000000UL, 0x89ABCDEFUL);
 * @endcode     
 * @par         Modify
 *              DRV_Return Sample_MEM_FlashIAPSingleProgram(uint32_t Address, uint32_t ProgramData)
 *******************************************************************************
 */
DRV_Return Sample_MEM_FlashIAPSingleProgram(uint32_t Address, uint32_t ProgramData)
{
    // Check IHRCO Enable
    if(CSC->CR0.MBIT.IHRCO_EN == 0)
        return DRV_Failure;

    // Check write flash address alignment
    if(((uint32_t)Address & 0x03) != 0)
        return DRV_Failure;

    if(((Address) < 0x1A000000) || ((Address) >= 0x1C000000))
        return DRV_Failure;

    __MEM_UnProtect();
    __MEM_Enable();
    __MEM_Access_Enable(MEM_ACCESS_IAP_WRITE);
    __MEM_SetWriteMode(IAPProgram);
    __MEM_SingleWriteUnProtect();

    // Mem Flag Clear.
    MEM->STA.B[0] = (MEM_STA_WPEF_mask_b0 | MEM_STA_EOPF_mask_b0);
    __ISB();

    *(uint32_t *)Address = ProgramData;

    __MEM_UnProtect();
    __MEM_SetWriteMode(None);
    __MEM_Access_Disable(MEM_ACCESS_IAP_WRITE);
    __MEM_Disable();
    __MEM_Protect();

    // Check MEM Flash Flag
    if(((MEM->STA.B[0] & MEM_STA_WPEF_mask_b0) != 0) && ((MEM->STA.B[0] & MEM_STA_EOPF_mask_b0) == 0))
        return DRV_Failure;

    return DRV_Success;
}



/**
 *******************************************************************************
 * @brief       Flash IAP single program
 * @details     1. Check parameter
 *      \n      2. Enable multiple program function
 *      \n      3. Flash program 
 *      \n      4. Disable erase function
 *      \n      5. Confirm correct or not 
 * @param[in]   StartAddress : IAP start address 0x1A000000
 * @param[in]   DataStartAddress : MG32x02z 1Page = 1024Byte
 * @param[in]   Length : Program Length, 1Length = 4Byte
 * @return      DRV_Success : 
 * @return      DRV_Failure : 
 * @exception   None
 * @note        
 * @par         Example
 * @code        
                DRV_Return Sample_MEM_FlashIAPProgram((uint32_t)0x1A000000, (uint32_t)(&RxBuffer), (size(RxBuffer) / 4));
 * @endcode     
 * @par         Modify
 *              DRV_Return Sample_MEM_FlashIAPProgram(uint32_t StartAddress, uint32_t DataStartAddress, uint32_t Length)
 *******************************************************************************
 */
DRV_Return Sample_MEM_FlashIAPProgram(uint32_t StartAddress, uint32_t DataStartAddress, uint32_t Length)
{
    DRV_Return lDRV_Return = DRV_Success;
    uint32_t lCount;
    uint32_t *lptrSrc_addr;
    uint32_t *lptrDest_addr;
    

    // Check IHRCO Enable
    if(CSC->CR0.MBIT.IHRCO_EN == 0)
        return DRV_Failure;

    // Check write flash address alignment
    if(((StartAddress & 0x03) != 0) && ((DataStartAddress & 0x03) != 0))
        return DRV_Failure;

    if(((StartAddress) < 0x1A000000) || ((StartAddress) >= 0x1C000000))
        return DRV_Failure;

    lptrSrc_addr = (uint32_t *)DataStartAddress;
    lptrDest_addr = (uint32_t *)StartAddress;
    lCount = 0;

    __MEM_UnProtect();
    __MEM_Enable();
    __MEM_Access_Enable(MEM_ACCESS_IAP_WRITE);
    __MEM_SetWriteMode(IAPProgram);
    __MEM_MultipleWriteUnProtect();
    __MEM_Protect();

    do{
        // Mem Flag Clear.
        MEM->STA.B[0] = (MEM_STA_WPEF_mask_b0 | MEM_STA_EOPF_mask_b0);
        __ISB();
        *lptrDest_addr = *lptrSrc_addr;

        if((MEM->STA.B[0] & (MEM_STA_WPEF_mask_b0 | MEM_STA_EOPF_mask_b0)) != MEM_STA_EOPF_mask_b0)
        {
            lDRV_Return = DRV_Failure;
            break;
        }
        lptrSrc_addr ++;
        lptrDest_addr ++;
    }while(++lCount < Length);

    __MEM_UnProtect();
    __MEM_MultipleWriteProtect();
    __MEM_SetWriteMode(None);
    __MEM_Access_Disable(MEM_ACCESS_IAP_WRITE);
    __MEM_Disable();
    __MEM_Protect();

    return lDRV_Return;
}


