#include <uart/uart_C.h>
#include "uartstdio.h"
#include "uart_util.h"


__interrupt void scicTXFIFOISR(void);
__interrupt void scicRXFIFOISR(void);
void initSCICFIFO(void);

// CN2 89:Txd, 90:RxD, SCI-C

// Received data for SCI-C
// Used for checking the received data

uint16_t rDataPointC;
uint16_t sDataC[80];
uint16_t rDataC[80];
uint16_t rDataIndexC=0;
//uint16_t iSerialCommantRequested=0;
void initSCICFIFO(void)
{
    int i;
    for(i = 0; i < 80; i++)
    {
         sDataC[i] = i;
    }

    rDataPointC = sDataC[0];

    GPIO_setMasterCore(90, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_90_SCIRXDC);
    GPIO_setDirectionMode(90, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(90, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(90, GPIO_QUAL_ASYNC);


    GPIO_setMasterCore(89, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_89_SCITXDC);
    GPIO_setDirectionMode(89, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(89, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(89, GPIO_QUAL_ASYNC);

    //
    // 8 char bits, 1 stop bit, no parity. Baud rate is 9600.
    //
    SCI_setConfig(SCIC_BASE, DEVICE_LSPCLK_FREQ, 115200, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_enableModule(SCIC_BASE);
    //SCI_enableLoopback(SCIC_BASE);
    SCI_disableLoopback(SCIC_BASE);
    SCI_resetChannels(SCIC_BASE);
    SCI_enableFIFO(SCIC_BASE);

    //
    // RX and TX FIFO Interrupts Enabled
    //
    SCI_disableInterrupt(SCIC_BASE, SCI_INT_RXERR);
    SCI_enableInterrupt(SCIC_BASE, (SCI_INT_RXFF | SCI_INT_TXFF));

    SCI_setFIFOInterruptLevel(SCIC_BASE, SCI_FIFO_TX1, SCI_FIFO_RX1);
    SCI_performSoftwareReset(SCIC_BASE);

    SCI_resetTxFIFO(SCIC_BASE);
    SCI_resetRxFIFO(SCIC_BASE);

    Interrupt_register(INT_SCIC_RX, scicRXFIFOISR);
    Interrupt_register(INT_SCIC_TX, scicTXFIFOISR);


    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}


//
// scicTXFIFOISR - SCIC Transmit FIFO ISR
//
__interrupt void scicTXFIFOISR(void)
{
    //SCI_writeCharArray(SCIC_BASE, sDataC, 2);
    //for(i = 0; i < 2; i++) { sDataC[i] = (sDataC[i] + 1) & 0x00FF; }
    SCI_clearInterruptStatus(SCIC_BASE, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}
//
__interrupt void scicRXFIFOISR(void)
{
    uint16_t idx=0;
    ASSERT(SCI_isBaseValid(SCIC_BASE));
    idx=SCI_getRxFIFOStatus(SCIC_BASE) ;
    rDataIndexC = ((rDataIndexC+ idx > 80)) ? 0 : rDataIndexC;
    SCI_readCharArray(SCIC_BASE, rDataC+rDataIndexC, idx);
    rDataIndexC+=idx;

    SCI_clearOverflowStatus(SCIC_BASE);
    SCI_clearInterruptStatus(SCIC_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
    //Example_PassCount++;
}




