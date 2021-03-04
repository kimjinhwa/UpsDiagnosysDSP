#include <uart/uart485_B.h>
#include "uartstdio.h"
#include "uart_util.h"



//#define 0
#define GPIO_0_B485_EN     0

__interrupt void scibTXFIFOISR(void);
__interrupt void scibRXFIFOISR(void);
void initSCIBFIFO(void);

// CN1 86:Txd, 87:RxD, SCI-D

// Received data for SCI-A
// Used for checking the received data


uint16_t rDataPointB;
uint16_t sDataB[80];
uint16_t rDataB[80];
uint16_t rDataIndexB=0;

void initSCIBFIFO(void)
{
    int i;
    for(i = 0; i < 80; i++)
    {
         sDataB[i] = i;
    }

    rDataPointB = sDataB[0];

 //GPIO_0 B485_EN   GPIO_0_GPIO0
    GPIO_setDirectionMode(GPIO_0_B485_EN, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(GPIO_0_B485_EN, GPIO_CORE_CPU1);
    GPIO_setPadConfig(GPIO_0_B485_EN , GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_B485_EN );
    GPIO_writePin(GPIO_0_B485_EN ,0);

    GPIO_setMasterCore(87, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_87_SCIRXDB);
    GPIO_setDirectionMode(87, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(87, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(87, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(86, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_86_SCITXDB);
    GPIO_setDirectionMode(86, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(86, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(86, GPIO_QUAL_ASYNC);

    //
    // 8 char bits, 1 stop bit, no parity. Baud rate is 9600.
    //
    SCI_setConfig(SCIB_BASE, DEVICE_LSPCLK_FREQ, 9600, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_enableModule(SCIB_BASE);
    //SCI_enableLoopback(SCIB_BASE);
    SCI_disableLoopback(SCIB_BASE);
    SCI_resetChannels(SCIB_BASE);
    //SCI_enableFIFO(SCIB_BASE);

    //
    // RX and TX FIFO Interrupts Enabled
    //
    SCI_disableInterrupt(SCIB_BASE, SCI_INT_RXERR);
    //SCI_enableInterrupt(SCIB_BASE, (SCI_INT_RXFF | SCI_INT_TXFF));

    //SCI_setFIFOInterruptLevel(SCIB_BASE, SCI_FIFO_TX1, SCI_FIFO_RX1);
    SCI_performSoftwareReset(SCIB_BASE);

    //Interrupt_register(INT_SCIB_RX, scibRXFIFOISR);
    //Interrupt_register(INT_SCIB_TX, scibTXFIFOISR);

    //SCI_resetTxFIFO(SCIB_BASE);
    //SCI_resetRxFIFO(SCIB_BASE);



    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);// SCIC and SCIB
}


//
// scibTXFIFOISR - SCIB Transmit FIFO ISR
//
__interrupt void scibTXFIFOISR(void)
{
    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
//
__interrupt void scibRXFIFOISR(void)
{
    uint16_t idx=0;
    ASSERT(SCI_isBaseValid(SCIB_BASE));
    idx=SCI_getRxFIFOStatus(SCIB_BASE) ;
    rDataIndexB = ((rDataIndexB+ idx > 80)) ? 0 : rDataIndexB;
    SCI_readCharArray(SCIB_BASE, rDataB+rDataIndexB, idx);
    rDataIndexB+=idx;

    SCI_clearOverflowStatus(SCIB_BASE);
    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}



