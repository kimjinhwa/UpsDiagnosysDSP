/*
 * cpuFlashMemory.c
 *
 *  Created on: May 4, 2021
 *      Author: STELLA
 */


#include "cpuFlashMemory.h"



#define CPUCLK_FREQUENCY        200
#define Bzero_SectorM_start         0x0BC000
#define Bzero_SectorM_End           0x0BCFFF

extern void InitFlash(void);
extern void SeizeFlashPump(void);


#define ramFuncSection ".TI.ramfunc"
#pragma CODE_SECTION(writeCallFlashAPI, ramFuncSection);
#pragma CODE_SECTION(FlahsError,ramFuncSection);
void writeCallFlashAPI(uint32_t flashAddress,uint16_t len);

#define  WORDS_IN_FLASH_BUFFER    0xFF
extern uint16   Buffer[WORDS_IN_FLASH_BUFFER + 1];
extern uint32   *Buffer32 ;

void FlahsError(Fapi_StatusType status)
{
    __asm("    ESTOP0");
}

void CallFlashAPI(uint16_t address,uint16_t *BufferData,uint16_t len)
{
    uint16_t i;
    InitFlash();
    SeizeFlashPump();

    for(i=0; i <= WORDS_IN_FLASH_BUFFER; i++)
    {
        Buffer[i] = HWREGH(userFlashStart+i); //Read Data from flash to ram by 16BIT
    }
    for(i=0; i < len; i++)
    {
        Buffer[address + i] = BufferData[i]; // Copy data to write  , 16Bit
    }
    writeCallFlashAPI(Bzero_SectorM_start,8);
}
void writeCallFlashAPI(uint32_t flashAddress,uint16_t len)
{

    uint16_t i;
    uint32 u32Index = 0;
    Fapi_StatusType oReturnCheck;
    Fapi_FlashStatusWordType oFlashStatusWord;
    volatile Fapi_FlashStatusType oFlashStatus;

    EALLOW;
    oReturnCheck =  Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS,CPUCLK_FREQUENCY);
    if(oReturnCheck != Fapi_Status_Success) { FlahsError(oReturnCheck); }

    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);
    if(oReturnCheck != Fapi_Status_Success) { FlahsError(oReturnCheck); }
    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                                                     (uint32 *)flashAddress);
    if(oReturnCheck != Fapi_Status_Success) { FlahsError(oReturnCheck); }

    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady) { }
    oReturnCheck = Fapi_doBlankCheck((uint32 *)flashAddress,
                                     userFlashLenght,
                                     &oFlashStatusWord);

    if(oReturnCheck != Fapi_Status_Success) { FlahsError(oReturnCheck); }

    for(i=0, u32Index = Bzero_SectorM_start;
            (u32Index < (Bzero_SectorM_start + WORDS_IN_FLASH_BUFFER)) &&
                    (oReturnCheck == Fapi_Status_Success); i+= 8, u32Index+= 8)
    {
        oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index,Buffer+i,
                                                    8,
                                                    0,
                                                    0,
                                                    Fapi_AutoEccGeneration);

        while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy) { }
        if(oReturnCheck != Fapi_Status_Success)
        {
            FlahsError(oReturnCheck);
        }
        oFlashStatus = Fapi_getFsmStatus();

        oReturnCheck = Fapi_doVerify((uint32 *)u32Index,
                                     4, Buffer32+(i/2),
                                     &oFlashStatusWord);
        if(oReturnCheck != Fapi_Status_Success)
        {
            FlahsError(oReturnCheck);
        }
        if(oReturnCheck != Fapi_Status_Success)
        {
            FlahsError(oReturnCheck);
        }
    }
    EDIS;
}
