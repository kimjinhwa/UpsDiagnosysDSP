#include "driverlib.h"
#include "device.h"
#include "F2837xD_device.h"
#include "DiagnosysUps.h"

//DAC_setReferenceVoltage(uint32_t base, DAC_ReferenceVoltage source)
//DAC_setLoadMode(uint32_t base, DAC_LoadMode mode)
//DAC_setPWMSyncSignal(uint32_t base, uint16_t signal)
//DAC_getActiveValue(uint32_t base)
//DAC_setShadowValue(uint32_t base, uint16_t value)
//DAC_getShadowValue(uint32_t base)
//DAC_enableOutput(uint32_t base)
//DAC_disableOutput(uint32_t base)
//DAC_setOffsetTrim(uint32_t base, int16_t offset)
//DAC_getOffsetTrim(uint32_t base)
//DAC_lockRegister(uint32_t base, uint16_t reg)
//DAC_isRegisterLocked(uint32_t base, uint16_t reg)
//DAC_tuneOffsetTrim(uint32_t base, float32_t referenceVoltage);
int QuadratureTable[40] = {
        0x0000,         // [0]  0
        0x18F8,         // [1]  11.25
        0x30FB,         // [2]  22.50
        0x471C,         // [3]  33.75
        0x5A81,         // [4]  45.00
        0x6A6C,         // [5]  56.25
        0x7640,         // [6]  67.50
        0x7D89,         // [7]  78.75
        0x7FFF,         // [8]  90.00
        0x7D89,         // [9]  78.75
        0x7640,         // [10] 67.50
        0x6A6C,         // [11] 56.25
        0x5A81,         // [12] 45.00
        0x471C,         // [13] 33.75
        0x30FB,         // [14] 22.50
        0x18F8,         // [15] 11.25
        0x0000,         // [16] 0
        0xE708,         // [17] -11.25
        0xCF05,         // [18] -22.50
        0xB8E4,         // [19] -33.75
        0xA57F,         // [20] -45.00
        0x9594,         // [21] -56.25
        0x89C0,         // [22] -67.50
        0x8277,         // [23] -78.75
        0x8000,         // [24] -90.00
        0x8277,         // [25] -78.75
        0x89C0,         // [26] -67.50
        0x9594,         // [27] -56.25
        0xA57F,         // [28] -45.00
        0xB8E4,         // [29] -33.75
        0xCF05,         // [30] -22.50
        0xE708,         // [31] -11.25
        0x0000,         // [32] 0
        0x18F8,         // [33] 11.25
        0x30FB,         // [34] 22.50
        0x471C,         // [35] 33.75
        0x5A81,         // [36] 45.00
        0x6A6C,         // [37] 56.25
        0x7640,         // [38] 67.50
        0x7D89          // [39] 78.75
};
Uint16 sineEnable = 0;
Uint16 dacOffset;
uint16_t idx=0;                              // Index into result buffer
Uint16 dacOutput;
//EPWM_setInterruptSource(uint32_t base, uint16_t interruptSource) EPWM_INT_TBCTR_ZERO
//EPWM_setInterruptEventCount(uint32_t base, uint16_t eventCount)
//EPWM_clearEventTriggerInterruptFlag
//EPWM_enableInterruptEventCountInit(uint32_t base)
__interrupt void pwmE2ISR(void);
__interrupt void pwmE2ISR(void)
{
    //if (sineEnable != 0)
    //{
    dacOutput = dacOffset + ((QuadratureTable[idx % 0x20] ^ 0x8000) >> 5);
    // }
    //else
    //{
    //    dacOutput = dacOffset;
    //}
    DacbRegs.DACVALS.all = dacOutput;
    idx++;
    EPWM_clearEventTriggerInterruptFlag(EPWM2_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}
void setEpwm(){

    Interrupt_register(INT_EPWM2, &pwmE2ISR);
    //ADC_SAMPLING_FREQ
    //1920 60Hz
    EPWM_SignalParams pwmSignal =
    {1920, 0.5f, 0.5f, true, DEVICE_SYSCLK_FREQ, SYSCTL_EPWMCLK_DIV_2,
     EPWM_COUNTER_MODE_UP_DOWN, EPWM_CLOCK_DIVIDER_1,
     EPWM_HSCLOCK_DIVIDER_1};

    EPWM_configureSignal(EPWM2_BASE, &pwmSignal);
    /*
    EPWM_setTimeBasePeriod(EPWM2_BASE, 2000);
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setClockPrescaler(EPWM2_BASE,
                               EPWM_CLOCK_DIVIDER_1,
                               EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBaseCounter(EPWM2_BASE, 0U);
    EPWM_setCounterCompareValue(EPWM2_BASE,
                                    EPWM_COUNTER_COMPARE_A,
                                    50 );
    EPWM_setCounterCompareValue(EPWM2_BASE,
                                    EPWM_COUNTER_COMPARE_B,
                                    1950);
    */
    EPWM_setInterruptSource(EPWM2_BASE , EPWM_INT_TBCTR_ZERO );
    EPWM_setInterruptEventCount(EPWM2_BASE, 1U);
    EPWM_enableInterrupt(EPWM2_BASE);

}
void setDacCI(void)
{
    setEpwm();
    /*
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
    DacaRegs.DACCTL.bit.LOADMODE = 0;           // Load on next SYSCLK
    DacaRegs.DACVALS.all = 0x0800;              // Set mid-range
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;         // Enable DAC
    EDIS;
     */
    //VSTS_S1
    DAC_setReferenceVoltage(DACB_BASE, DAC_REF_ADC_VREFHI);
    DAC_setLoadMode(DACB_BASE,DAC_LOAD_SYSCLK);
    DAC_setShadowValue(DACB_BASE,0x0800);
    DAC_enableOutput(DACB_BASE);

}
