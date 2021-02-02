#include "driverlib.h"
#include "device.h"
#include "F2837xD_device.h"

// Defines
#define CPU_FRQ_200MHZ          1
#define ADC_SAMPLING_FREQ       100000.0L
#define EPWM_HSPCLKDIV          1           //EPWM_CLOCK is SYSCLK/(2*2)
//#define EPWM1_PERIOD            500
//#define EPWM1_DUTY_CYCLE        250
#define EPWM1_PERIOD            50000
#define EPWM1_DUTY_CYCLE        25000
#define EPWM2_PERIOD            50000
#define EPWM2_DUTY_CYCLE        25000
#define RESULTS_BUFFER_SIZE     800
#define EX_ADC_RESOLUTION       12

#define RFFT_STAGES     9
#define RFFT_SIZE       (1 << RFFT_STAGES)
#define F_PER_SAMPLE    (ADC_SAMPLING_FREQ/(float)RFFT_SIZE)

#pragma DATA_SECTION(RFFTin1Buff,"RFFTdata1")
uint16_t RFFTin1Buff[2*RFFT_SIZE];
extern uint16_t RFFTin1Buff_test[2*RFFT_SIZE];

#pragma DATA_SECTION(RFFTmagBuff,"RFFTdata2")
float RFFTmagBuff[RFFT_SIZE/2+1];

#pragma DATA_SECTION(RFFToutBuff,"RFFTdata3")
float RFFToutBuff[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("RFFTdata4")
#else
#pragma DATA_SECTION(RFFTF32Coef,"RFFTdata4")
#endif //__cplusplus
float RFFTF32Coef[RFFT_SIZE];
//
#pragma DATA_SECTION(adcAResults_1,"ADCBUFFER1")
#pragma DATA_SECTION(adcAResults_2,"ADCBUFFER1")
#pragma DATA_SECTION(adcAResults_3,"ADCBUFFER1")
#pragma DATA_SECTION(adcAResults_4,"ADCBUFFER1")
#pragma DATA_SECTION(adcAResults_5,"ADCBUFFER1")

#pragma DATA_SECTION(adcAResults_6,"ADCBUFFER2")
#pragma DATA_SECTION(adcAResults_7,"ADCBUFFER2")
#pragma DATA_SECTION(adcAResults_8,"ADCBUFFER2")
#pragma DATA_SECTION(adcAResults_9,"ADCBUFFER2")
#pragma DATA_SECTION(adcAResults_10,"ADCBUFFER2")

#pragma DATA_SECTION(adcAResults_11,"ADCBUFFER3")
#pragma DATA_SECTION(adcAResults_12,"ADCBUFFER3")
#pragma DATA_SECTION(adcAResults_13,"ADCBUFFER3")
#pragma DATA_SECTION(adcAResults_14,"ADCBUFFER3")
#pragma DATA_SECTION(adcAResults_15,"ADCBUFFER3")

#pragma DATA_SECTION(adcAResults_16,"ADCBUFFER4")
#pragma DATA_SECTION(adcAResults_17,"ADCBUFFER4")
#pragma DATA_SECTION(adcAResults_18,"ADCBUFFER4")
#pragma DATA_SECTION(adcAResults_19,"ADCBUFFER4")
#pragma DATA_SECTION(adcAResults_20,"ADCBUFFER4")

#define RAM_BANK_SIZE  0x00010000
#define RAM_ADCBUFFER1 0x00018000
#define RAM_ADCBUFFER2 0x00019000
#define RAM_ADCBUFFER3 0x0001A000
#define RAM_ADCBUFFER4 0x0001B000


uint16_t adcAResults_1[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_2[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_3[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_4[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_5[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_6[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_7[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_8[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_9[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_10[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_11[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_12[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_13[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_14[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_15[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_16[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_17[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_18[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_19[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcAResults_20[RESULTS_BUFFER_SIZE];   // Buffer for results

uint16_t index;                              // Index into result buffer
volatile uint16_t bufferFull;                // Flag to indicate buffer is full

//
// Function Prototypes
//
void initADC(void);
void initEPWM(void);
void initADCSOC(void);
__interrupt void adcA1ISR(void);

//
// Main
//
#include "fpu_rfft.h"            // Main include file
#include "math.h"
RFFT_F32_STRUCT rfft;
RFFT_ADC_F32_STRUCT rfft_adc;
RFFT_ADC_F32_STRUCT_Handle hnd_rfft_adc = &rfft_adc;
RFFT_F32_STRUCT rfft;
RFFT_F32_STRUCT_Handle hnd_rfft = &rfft;
volatile uint16_t flagInputReady = 0;
volatile uint16_t sampleIndex = 0;
uint16_t pass = 0;
uint16_t fail = 0;
#define EPSILON         0.1
void fft_routine()
{

    for(index = 0; index < 2*RFFT_SIZE; index++)
    {
        RFFTin1Buff[index] = RFFTin1Buff_test[index];
    }

    uint16_t i, j;
    float freq = 0.0;
    hnd_rfft_adc->Tail = &(hnd_rfft->OutBuf);

    hnd_rfft->FFTSize   = RFFT_SIZE;       //FFT size
    hnd_rfft->FFTStages = RFFT_STAGES;     //FFT stages

    hnd_rfft_adc->InBuf = &RFFTin1Buff[0]; //Input buffer (12-bit ADC) input
    hnd_rfft->OutBuf    = &RFFToutBuff[0]; //Output buffer
    hnd_rfft->CosSinBuf = &RFFTF32Coef[0]; //Twiddle factor
    hnd_rfft->MagBuf    = &RFFTmagBuff[0]; //Magnitude output buffer
    RFFT_f32_sincostable(hnd_rfft);        //Calculate twiddle factor
    for (i=0; i < RFFT_SIZE; i++){
        RFFToutBuff[i] = 0;                //Clean up output buffer
    }

    for (i=0; i < RFFT_SIZE/2; i++){
        RFFTmagBuff[i] = 0;                //Clean up magnitude buffer
    }
    RFFT_adc_f32(hnd_rfft_adc);        // Calculate real FFT with 12-bit
    /*
    for(i = 0; i < RFFT_SIZE; i++){
        if(fabs(RFFTgoldenOut[i] - hnd_rfft->OutBuf[i]) <= EPSILON){
            pass++;
        }else{
            fail++;
        }
    }
    */

    flagInputReady = 0;                 // Reset the flag
    RFFT_f32_mag(hnd_rfft);             //Calculate magnitude
    /*
    for(i = 0; i <= RFFT_SIZE/2; i++){
        if(fabs(RFFTgoldenMagnitude[i] - hnd_rfft->MagBuf[i]) <= EPSILON){
            pass++;
        }else{
            fail++;
        }
    }
    */
    j = 1;
    freq = RFFTmagBuff[1];
    for(i=2;i<RFFT_SIZE/2+1;i++){
        //Looking for the maximum component of frequency spectrum
        if(RFFTmagBuff[i] > freq){
            j = i;
            freq = RFFTmagBuff[i];
        }
    }
    freq =(float)F_PER_SAMPLE * (float)j;
    freq = freq;
}

#define BLINKY_LED_GPIO 31
#define PULSE_OUTPUT_GPIO 18
#define GPIO_0     0
uint16_t ToggleCount = 0;
void initLocalGpio()
{
    GPIO_setDirectionMode(BLINKY_LED_GPIO , GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(BLINKY_LED_GPIO , GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(BLINKY_LED_GPIO , GPIO_CORE_CPU1);
    GPIO_setQualificationMode(BLINKY_LED_GPIO , GPIO_QUAL_SYNC);

    GPIO_setDirectionMode(PULSE_OUTPUT_GPIO , GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(PULSE_OUTPUT_GPIO , GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(PULSE_OUTPUT_GPIO , GPIO_CORE_CPU1);
    GPIO_setQualificationMode(PULSE_OUTPUT_GPIO , GPIO_QUAL_SYNC);

    GPIO_setPadConfig(GPIO_0, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_EPWM1A);
}

inline void initBuffer()
{

    for(index = 0; index < RESULTS_BUFFER_SIZE; index++)
    {
        adcAResults_1[index] = 0; adcAResults_2[index] = 0; adcAResults_3[index] = 0; adcAResults_4[index] = 0; adcAResults_5[index] = 0;
        adcAResults_6[index] = 0; adcAResults_7[index] = 0; adcAResults_8[index] = 0; adcAResults_9[index] = 0; adcAResults_10[index] = 0;
        adcAResults_11[index] = 0; adcAResults_12[index] = 0; adcAResults_13[index] = 0; adcAResults_14[index] = 0; adcAResults_15[index] = 0;
        adcAResults_16[index] = 0; adcAResults_17[index] = 0; adcAResults_18[index] = 0; adcAResults_19[index] = 0; adcAResults_20[index] = 0;
    }
    //RAM_ADCBUFFER1
    //HWREGH(RAM_ADCBUFFER1+0)=0;
    /*
    uint32_t xx=0;
    uint32_t yy=0;
    for(xx=0;xx<5*RAM_BANK_SIZE;xx+=RAM_BANK_SIZE){
        for(yy=0;yy<5*RESULTS_BUFFER_SIZE;yy+=RESULTS_BUFFER_SIZE){
            for(index = 0; index < RESULTS_BUFFER_SIZE; index++){
                HWREGH(RAM_ADCBUFFER1+xx+yy+index) = 1;
            }
            HWREGH(RAM_ADCBUFFER1+xx+yy+index) = 1;
        }
        HWREGH(RAM_ADCBUFFER1+xx+yy+index) = 1;
    }
    */
}
void main(void)
{
    Device_init();
    Device_initGPIO();
    initLocalGpio();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Interrupt_register(INT_ADCA1, &adcA1ISR);

    initBuffer();
    fft_routine();//fft routine test

    initADC();
    initEPWM();
    initADCSOC();
    index = 0;
    bufferFull = 0;

    Interrupt_enable(INT_ADCA1);

    EINT;//enable inturrupt
    ERTM;//enable debug enable

        EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
        EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP);
    while(1)
    {
        //
        // Start ePWM1, enabling SOCA and putting the counter in up-count mode
        //
        //EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP);

        //
        // Wait while ePWM1 causes ADC conversions which then cause interrupts.
        // When the results buffer is filled, the bufferFull flag will be set.
        //
        while(bufferFull == 0)
        {
            GPIO_togglePin(BLINKY_LED_GPIO );
            DEVICE_DELAY_US(100000);
        }
        bufferFull = 0;     // Clear the buffer full flag

        //
        // Stop ePWM1, disabling SOCA and freezing the counter
        //
        //EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
        //EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);

        //
        // Software breakpoint. At this point, conversion results are stored in
        // adcAResults_1.
        //
        // Hit run again to get updated conversions.
        //
        //ESTOP0; //emulation halt
    }
}

//
// Function to configure and power up ADCA.
//
void initADC(void)
{
    /*
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0); // Set ADCCLK divider to /4
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED); // // Set resolution and signal mode (see #defines above) and load // corresponding trims.
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV); // Set pulse positions to late
    ADC_enableConverter(ADCA_BASE); // Power up the ADC and then delay for 1 ms
     */
    int xx;
    for(xx=0;xx<4*0x80;xx+=0x80){
        ADC_setPrescaler(ADCA_BASE+xx, ADC_CLK_DIV_4_0); // Set ADCCLK divider to /4
        ADC_setMode(ADCA_BASE+xx, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED); // // Set resolution and signal mode (see #defines above) and load // corresponding trims.
        ADC_setInterruptPulseMode(ADCA_BASE+xx, ADC_PULSE_END_OF_CONV); // Set pulse positions to late
        ADC_enableConverter(ADCA_BASE+xx); // Power up the ADC and then delay for 1 ms
    };
    DEVICE_DELAY_US(1000);
}

//
// Function to configure ePWM1 to generate the SOC.
//
void initEPWM(void)
{
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A); // Disable SOCA
    EPWM_setADCTriggerSource(EPWM1_BASE,EPWM_SOC_A,EPWM_SOC_TBCTR_PERIOD);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);
    //EPWM_setCountrCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0x0800);
    EPWM_setTimeBasePeriod(EPWM1_BASE, 0x1000);//EPWM_setTimeBasePeriod(EPWM1_BASE, 0x30d4);//At 8Khz
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
}

    //EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP );
//
// Function to configure ADCA's SOC0 to be triggered by ePWM1.
//
void initADCSOC(void)
{
    //AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    // For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz // SYSCLK rate) will be used.
    //ADCA_BASE
    // max number 8 for adca, 8 * 44 sysclk cycle  352 * 5us = 1.762uS
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 15);//VIN_R
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 15);//VSTS_R
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 15);//VO_R
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 15);//IIN_R
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN4, 15);//IINV_R
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN5, 15);//IO_R
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN14, 15);//THR1
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER7, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN15, 15);//THR2

    //ADCB_BASE
       ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 15);//VIN_S
       ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 15);//VSTS_S1
       ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 15);//VO_S
       ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 15);//IIN_S

    //ADCC_BASE
       ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 15);//VO_T
       ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 15);//IO_S
       ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN4, 15);//IINV_R

    //ADCD_BASE
       ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 15);//VBAT
       ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 15);//VSTS_S
       ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 15);//IBAT
       ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 15);//IO_T
       ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN4, 15);//THR2
    //
    // Set SOC0 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    ADC_setBurstModeConfig(ADCA_BASE,ADC_TRIGGER_EPWM1_SOCA , 8);
    ADC_setSOCPriority(ADCA_BASE,ADC_PRI_ALL_HIPRI);
    ADC_enableBurstMode(ADCA_BASE);

    ADC_setBurstModeConfig(ADCB_BASE,ADC_TRIGGER_EPWM1_SOCA , 4);
    ADC_setSOCPriority(ADCB_BASE,ADC_PRI_ALL_HIPRI);
    ADC_enableBurstMode(ADCB_BASE);

    ADC_setBurstModeConfig(ADCC_BASE,ADC_TRIGGER_EPWM1_SOCA , 3);
    ADC_setSOCPriority(ADCC_BASE,ADC_PRI_ALL_HIPRI);
    ADC_enableBurstMode(ADCC_BASE);

    ADC_setBurstModeConfig(ADCD_BASE,ADC_TRIGGER_EPWM1_SOCA , 5);
    ADC_setSOCPriority(ADCD_BASE,ADC_PRI_ALL_HIPRI);
    ADC_enableBurstMode(ADCD_BASE);
    //ADCA
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER7);//
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    //ADCB
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER3);//
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);

    //ADCC
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);//
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);

    //ADCD
    ADC_setInterruptSource(ADCD_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER4);//
    ADC_enableInterrupt(ADCD_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
}

//
// ADC A Interrupt 1 ISR
//
__interrupt void adcA1ISR(void)
{
    if( ADC_getInterruptStatus(ADCB_BASE,ADC_INT_NUMBER1)){
        adcAResults_1[index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
        adcAResults_2[index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_3[index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2);
        adcAResults_4[index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER3);
        adcAResults_5[index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4);
        adcAResults_6[index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5);
        adcAResults_7[index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER6);
        adcAResults_8[index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER7);
    }
    else fail++;
    if( ADC_getInterruptStatus(ADCB_BASE,ADC_INT_NUMBER1)){
        adcAResults_9[index] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);
        adcAResults_10[index] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_11[index] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2);
        adcAResults_12[index] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER3);
    }
    else fail++;
    if( ADC_getInterruptStatus(ADCC_BASE,ADC_INT_NUMBER1)){
        adcAResults_13[index] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
        adcAResults_14[index] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_15[index] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER3);
    }
    else fail++;
    if( ADC_getInterruptStatus(ADCD_BASE,ADC_INT_NUMBER1)){
        adcAResults_16[index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0);
        adcAResults_17[index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_18[index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER2);
        adcAResults_19[index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER3);
        adcAResults_20[index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER4);
    }
    else fail++;
    index++;
    // Trigger to soc 1

    //
    // Set the bufferFull flag if the buffer is full
    //
    if(RESULTS_BUFFER_SIZE <= index)
    {
        index = 0;
        bufferFull = 1;
    }
    if (ToggleCount++ >= 15)
    {
        GPIO_togglePin(PULSE_OUTPUT_GPIO );
        ToggleCount = 0;
    }

    //
    // Clear the interrupt flag
    //
    int xx;
    for(xx=0;xx<4*0x80;xx+=0x80)
        ADC_clearInterruptStatus(ADCA_BASE+xx, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    for(xx=0;xx<4*0x80;xx+=0x80){
        if(true == ADC_getInterruptOverflowStatus(ADCA_BASE+xx, ADC_INT_NUMBER1))
        {
            ADC_clearInterruptOverflowStatus(ADCA_BASE+xx, ADC_INT_NUMBER1);
            ADC_clearInterruptStatus(ADCA_BASE+xx, ADC_INT_NUMBER1);
        }
    }

    //
    // Acknowledge the interrupt
    // ADCA1 and  ADCB1 and  ADCC1 and  ADCD1 are GROUP1
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//#############################################################################
//
// FILE:   adc_ex2_soc_epwm.c
//
// TITLE:  ADC ePWM Triggering
//
//! \addtogroup driver_example_list
//! <h1>ADC ePWM Triggering</h1>
//!
//! This example sets up ePWM1 to periodically trigger a conversion on ADCA.
//!
//! \b External \b Connections \n
//!  - A0 should be connected to a signal to convert
//!
//! \b Watch \b Variables \n
//! - \b adcAResults_1 - A sequence of analog-to-digital conversion samples from
//!   pin A0. The time between samples is determined based on the period
//!   of the ePWM timer.
//!
//
//#############################################################################
// $TI Release: F2837xD Support Library v3.11.00.00 $
// $Release Date: Sun Oct  4 15:55:24 IST 2020 $
// $Copyright:
// Copyright (C) 2013-2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
    //
    //EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_STOP_FREEZE); // Freeze the counter
    //EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1 ,EPWM_HSCLOCK_DIVIDER_1 );
    //EPWM_setTimeBasePeriod(EPWM1_BASE, 0x30d4);//At 8Khz
    //EPWM_setInterruptEventCount(EPWM1_BASE, 1);
    //EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_STOP_FREEZE); // Freeze the counter
    //EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A); // Disable SOCA
    //EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA); // Configure the SOC to occur on the first up-count event
    //EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);
    //EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0x0800); // Set the compare A value to 2048 and the period to 4096
    //EPWM_setTimeBasePeriod(EPWM1_BASE, 0x1000);
