#include "driverlib.h"
#include "device.h"
#include "DiagnosysUps.h"


#define BURST       8       // write 8 to the register for a burst size of 8
#define TRANSFER    100      // [(MEM_BUFFER_SIZE/(BURST)]

extern uint16_t adcAResults_1[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_2[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_3[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_4[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_5[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_6[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_7[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_8[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_9[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_10[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_11[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_12[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_13[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_14[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_15[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_16[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_17[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_18[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_19[RESULTS_BUFFER_SIZE];   // Buffer for results
extern uint16_t adcAResults_20[RESULTS_BUFFER_SIZE];   // Buffer for results

extern uint16_t dmaCopydone;

__interrupt void dmaCh6ISR(void);
void initDMA(void);
void error();

void initDmaCopy()
{
    Interrupt_register(INT_DMA_CH6, &dmaCh6ISR);
    initDMA();  // set up the dma
    SysCtl_selectSecMaster(0, SYSCTL_SEC_MASTER_DMA);
    Interrupt_enable(INT_DMA_CH6);

}

void error(void)
{
    ESTOP0;  //Test failed!! Stop!
    for (;;);
}

void initDMA()
{
    DMA_initController();
    //const void *destAddr;
    //const void *srcAddr;
    //srcAddr = (const void *)sData;
    //destAddr = (const void *)rData;

    //
    // configure DMA CH6
    //
    //DMA_configAddresses(DMA_CH6_BASE, destAddr, srcAddr);
    DMA_configBurst(DMA_CH6_BASE,BURST,1,1);
    DMA_configTransfer(DMA_CH6_BASE,TRANSFER,1,1);
    DMA_configMode(DMA_CH6_BASE,DMA_TRIGGER_SOFTWARE, DMA_CFG_ONESHOT_DISABLE);
    DMA_setInterruptMode(DMA_CH6_BASE,DMA_INT_AT_END);
    DMA_enableTrigger(DMA_CH6_BASE);
    DMA_enableInterrupt(DMA_CH6_BASE);
}
__interrupt void dmaCh6ISR(void)
{
    DMA_stopChannel(DMA_CH6_BASE);
    // ACK to receive more interrupts from this PIE group
    EALLOW;
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
    EDIS;

    dmaCopydone = 1; // Test done.
    return;
}
