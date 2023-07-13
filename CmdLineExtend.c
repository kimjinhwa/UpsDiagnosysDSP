/*
 * CmdLineExtend.c
 *
 *  Created on: 2023. 7. 13.
 *      Author: STELLA
 */
#include "cmdLineExtend.h"


#pragma CODE_SECTION(myFunction,".TI.ramfunc")
void myFunction()
{
    SCIprintf("Test");
}
//
////******************************************************************************
////
////! Host interrupt service routine wrapper to make ISR compatible with
////! C2000 PIE controller.
////
////******************************************************************************
__interrupt void INT_myUSB0_ISR(void)
{
    USB0HostIntHandler();
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
