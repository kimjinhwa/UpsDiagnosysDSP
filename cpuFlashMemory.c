/*
 * cpuFlashMemory.c
 *
 *  Created on: May 4, 2021
 *      Author: STELLA
 */


#include "cpuFlashMemory.h"



#define CPUCLK_FREQUENCY        200

extern void InitFlash(void);
extern void SeizeFlashPump(void);

#define ramFuncSection ".TI.ramfunc"
#pragma CODE_SECTION(CallFlashAPI, ramFuncSection);
#pragma CODE_SECTION(Error,ramFuncSection);

//#define  WORDS_IN_FLASH_BUFFER    0xFF
//uint16   Buffer[WORDS_IN_FLASH_BUFFER + 1];


void Error(Fapi_StatusType status)
{
    __asm("    ESTOP0");
}

void CallFlashAPI(uint16_t *Buffer,uint16_t len){
    //DCSMCOMMON_BASE

    uint16_t repeatCount=0;
    uint32   *Buffer32 ;
    Flash_disableECC(FLASH0ECC_BASE);
    Flash_setPumpPowerMode(FLASH0ECC_BASE,FLASH_PUMP_PWR_ACTIVE);
    //Flash_initModule(FLASH0ECC_BASE, FLASH0ECC_BASE, 15);

     EALLOW;
     DcsmCommonRegs.FLSEM.all = 0xA501;
     EDIS;

    InitFlash();
    SeizeFlashPump();

    Fapi_StatusType oReturnCheck;
     Fapi_FlashStatusWordType oFlashStatusWord;
     volatile Fapi_FlashStatusType oFlashStatus;

     EALLOW;
     oReturnCheck =  Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS,CPUCLK_FREQUENCY);
     if(oReturnCheck != Fapi_Status_Success)
     {
         // Check Flash API documentation for possible errors.
         Error(oReturnCheck);
     }

     oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);
     if(oReturnCheck != Fapi_Status_Success)
     {
         // Check Flash API documentation for possible errors.
         Error(oReturnCheck);
     }
     oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                    (uint32 *)userFlashStart);
     if(oReturnCheck != Fapi_Status_Success)
     {
         // Check Flash API documentation for possible errors.
         Error(oReturnCheck);
     }
     // Wait until FSM is done with erase sector operation.
     while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady)
     {
     }
     //
     // Verify that Sector C is erased. The erase step itself does verification
     // as it goes. This verify is a second verification that can be done.
     //
     oReturnCheck = Fapi_doBlankCheck((uint32 *)userFlashStart,
                    userFlashLenght,
                    &oFlashStatusWord);

     if(oReturnCheck != Fapi_Status_Success)
     {
         //
         // Check Flash API documentation for possible errors.
         // If erase command fails, use Fapi_getFsmStatus() function to get the
         // FMSTAT register contents to see if any of the EV bit, ESUSP bit,
         // CSTAT bit or VOLTSTAT bit is set. Refer to API documentation for
         // more details.
         //
         Error(oReturnCheck);
     }
     //for(i=0;i<=WORDS_IN_FLASH_BUFFER;i++)
     {
         //         Buffer[i] = i;
     }
     //HWREGH(ADCA_BASE + ADC_O_OFFTRIM )
     //int i=0;
     //*(Buffer+0) = 21; *(Buffer+1) =  21; Buffer[2] =  21; Buffer[3] =  21; Buffer[4] =  i++; Buffer[5] =  i++; Buffer[6] =  i++; Buffer[7] =  i++;

     for(repeatCount =0; repeatCount < (len/8); repeatCount++ )
     {
         //i=0;
         oReturnCheck = Fapi_issueProgrammingCommand(
                        (uint32 *)(userFlashStart + repeatCount*8) ,
                        (uint16 *)(Buffer + repeatCount *8),
                        8,
                        0,
                        0,
                        Fapi_AutoEccGeneration);

         //
         // Wait until FSM is done with program operation.
         //
         while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy)
         {
         }
         if(oReturnCheck != Fapi_Status_Success)
          {
              //
              // Check Flash API documentation for possible errors.
              //
              Error(oReturnCheck);
          }
         oFlashStatus = Fapi_getFsmStatus();



         Buffer32 = (uint32 *)(Buffer+ repeatCount*8);

         oReturnCheck = Fapi_doVerify(
                         (uint32 *)(userFlashStart+ repeatCount*8),
                         4,
                         Buffer32,
                         &oFlashStatusWord);
         int i;
         for(i=0;i<8;i++){
             if( ((uint16 *)(userFlashStart+i+ repeatCount*8)) != ((uint16 *)(Buffer+ i+repeatCount*8)) )
             {
                 oReturnCheck = Fapi_Error_Fail;
                 break;
             }
         }
          if(oReturnCheck != Fapi_Status_Success)
          {
              //
              // Check Flash API documentation for possible errors.
               //
              //Error(oReturnCheck);
          }
      }

      EDIS;
      Flash_setPumpPowerMode(FLASH0ECC_BASE,FLASH_PUMP_PWR_SLEEP);
      //(*((volatile uint16_t *)((uintptr_t)(x))))
      int16_t value;
      value = HWREGH(userFlashStart);
      if(value == 0);
      value = HWREGH(userFlashStart+1);
      if(value == 1);
}
