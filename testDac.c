#include "driverlib.h"
#include "device.h"
#include "F2837xD_device.h"


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

void setDacCI(void)
{
    //DACA_BASE
    /*
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
    DacaRegs.DACCTL.bit.LOADMODE = 0;           // Load on next SYSCLK
    DacaRegs.DACVALS.all = 0x0800;              // Set mid-range
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;         // Enable DAC
    EDIS;
    */
    DAC_setReferenceVoltage(DACA_BASE, DAC_REF_ADC_VREFHI);
    DAC_setLoadMode(DACA_BASE,DAC_LOAD_SYSCLK);
    DAC_setShadowValue(DACA_BASE,0x0800);
    //DAC_setShadowValue(DACA_BASE,0x0FFD);
    DAC_enableOutput(DACA_BASE);

}
