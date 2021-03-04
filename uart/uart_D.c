#include <uart/uart_D.h>
#include "uartstdio.h"
#include "uart_util.h"



__interrupt void scidTXFIFOISR(void);
__interrupt void scidRXFIFOISR(void);
void initSCIDFIFO(void);

// CN3 93:Txd, 94:RxD, SCI-D

// Received data for SCI-A
// Used for checking the received data


uint16_t rDataPointD;
uint16_t sDataD[80];
uint16_t rDataD[80];
uint16_t rDataIndexD=0;

void initSCIDFIFO(void)
{
    int i;
    for(i = 0; i < 80; i++)
    {
         sDataD[i] = i;
    }

    rDataPointD = sDataD[0];

    GPIO_setMasterCore(94, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_94_SCIRXDD);
    GPIO_setDirectionMode(94, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(94, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(94, GPIO_QUAL_ASYNC);


    GPIO_setMasterCore(93, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_93_SCITXDD);
    GPIO_setDirectionMode(93, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(93, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(93, GPIO_QUAL_ASYNC);

    //
    // 8 char bits, 1 stop bit, no parity. Baud rate is 9600.
    //
    SCI_setConfig(SCID_BASE, DEVICE_LSPCLK_FREQ, 115200, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_enableModule(SCID_BASE);
    //SCI_enableLoopback(SCID_BASE);
    SCI_disableLoopback(SCID_BASE);
    SCI_resetChannels(SCID_BASE);
    SCI_enableFIFO(SCID_BASE);

    //
    // RX and TX FIFO Interrupts Enabled
    //
    SCI_disableInterrupt(SCID_BASE, SCI_INT_RXERR);
    SCI_enableInterrupt(SCID_BASE, (SCI_INT_RXFF | SCI_INT_TXFF));

    SCI_setFIFOInterruptLevel(SCID_BASE, SCI_FIFO_TX1, SCI_FIFO_RX1);
    SCI_performSoftwareReset(SCID_BASE);

    SCI_resetTxFIFO(SCID_BASE);
    SCI_resetRxFIFO(SCID_BASE);

    Interrupt_register(INT_SCID_RX, scidRXFIFOISR);
    Interrupt_register(INT_SCID_TX, scidTXFIFOISR);


    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);// SCIC and SCID
}


//
// scidTXFIFOISR - SCID Transmit FIFO ISR
//
__interrupt void scidTXFIFOISR(void)
{
    //SCI_writeCharArray(SCID_BASE, sDataD, 2);
    //for(i = 0; i < 2; i++) { sDataD[i] = (sDataA[i] + 1) & 0x00FF; }
    SCI_clearInterruptStatus(SCID_BASE, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}
//
__interrupt void scidRXFIFOISR(void)
{
    uint16_t idx=0;
    ASSERT(SCI_isBaseValid(SCID_BASE));
    idx=SCI_getRxFIFOStatus(SCID_BASE) ;
    rDataIndexD = ((rDataIndexD+ idx > 80)) ? 0 : rDataIndexD;
    SCI_readCharArray(SCID_BASE, rDataD+rDataIndexD, idx);
    rDataIndexD+=idx;

    SCI_clearOverflowStatus(SCID_BASE);
    SCI_clearInterruptStatus(SCID_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
    //Example_PassCount++;
}



