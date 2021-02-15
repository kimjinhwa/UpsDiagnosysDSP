#include "driverlib.h"
#include "device.h"
#include "F2837xD_device.h"

#include "fpu_rfft.h"            // Main include file
#include "math.h"
#include "DiagnosysUps.h"
#include "dmaCopy.h"
// Defines
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

#define BLINKY_LED_GPIO 31
#define PULSE_OUTPUT_GPIO 18
#define GPIO_0     0
#define EPSILON         0.1

#define DAC_TEST_USED   1

#ifdef DAC_TEST_USED
extern void setDacCI(void);
#endif

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


//TIMER
uint32_t cpuTimer0IntCount;

// FFT
RFFT_F32_STRUCT rfft;
RFFT_ADC_F32_STRUCT rfft_adc;
RFFT_ADC_F32_STRUCT_Handle hnd_rfft_adc = &rfft_adc;
RFFT_F32_STRUCT rfft;
RFFT_F32_STRUCT_Handle hnd_rfft = &rfft;

uint16_t pass = 0;
uint16_t fail = 0;

// Main routine
uint16_t index;                              // Index into result buffer
uint16_t request_fft=0x00;
volatile uint16_t bufferFull;                // Flag to indicate buffer is full
void initADC(void);
void initEPWM(void);
void initADCSOC(void);
void fft_routine(void);
__interrupt void adcA1ISR(void);

uint16_t ToggleCount = 0;
void fft_routine(void)
{

    for(index = 0; index < 2*RFFT_SIZE; index++)
    {
        RFFTin1Buff[index] = RFFTin1Buff_test[index];
    }

    uint16_t i, j;
    float freq = 0.0;
    //float calThd[9];
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

    RFFT_f32_mag(hnd_rfft);             //Calculate magnitude
    j = 1;
    freq = RFFTmagBuff[1];
    //int k;
    float sumTotal=0.0;
    float maxValue=0.0;
        i=2;
        for(;i<RFFT_SIZE/2+1;i++){
            //Looking for the maximum component of frequency spectrum
            if(RFFTmagBuff[i] > freq){
                j = i;
                freq = RFFTmagBuff[i];
            }
        }
        maxValue=RFFTmagBuff[j];

        freq =(float)F_PER_SAMPLE * (float)j;
        i=2;
        for(;i<RFFT_SIZE/2+1;i++){
           sumTotal+=RFFTmagBuff[i]*RFFTmagBuff[i];
        }
        sumTotal=sqrt(sumTotal);
    float THD=sumTotal/maxValue;
    /*
    for(k=0;k<9;k++){
        for(;i<RFFT_SIZE/2+1;i++){
            //Looking for the maximum component of frequency spectrum
            if(RFFTmagBuff[i] > freq){
                j = i;
                freq = RFFTmagBuff[i];
            }
        }
        freq =(float)F_PER_SAMPLE * (float)j;
        calThd[k]=RFFTmagBuff[j];
        freq =0.0;
        i= j+1+ (256/8) ;
    }
    */
    //for(k=1;k<9;k++)sumTotal+=calThd[k];
    //float THD=sumTotal/calThd[0];
    THD=THD;
}

//Z= power((2*abs(Y)),2);Y(1)=0
//THD = sqrt(sum(Z))/max((2*abs(Y)))

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

}

__interrupt void cpuTimer0ISR(void)
{
    cpuTimer0IntCount++;
    GPIO_togglePin(BLINKY_LED_GPIO );
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
void init_timer0()
{
    uint32_t temp;
    temp = (uint32_t)(DEVICE_SYSCLK_FREQ / 1000000 * 1000000);
    CPUTimer_setPeriod(CPUTIMER1_BASE, temp);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_startTimer(CPUTIMER1_BASE);
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
}
    //uint32_t elespedTime_b ;
    //uint32_t elespedTime_e ;
    //elespedTime_b = CPUTimer_getTimerCount(CPUTIMER1_BASE);
    //elespedTime_e = CPUTimer_getTimerCount(CPUTIMER1_BASE);
    //CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    //elespedTime_b = CpuTimer0Regs.TIM.all;
    //elespedTime_e = CpuTimer0Regs.TIM.all;
    //elespedTime_b = elespedTime_b -elespedTime_e;
    //CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    //elespedTime_b = CpuTimer0Regs.TIM.all;

#include "uart/uart.h"
//
uint16_t dmaCopydone;
void main(void)
{
    Device_init();
    Device_initGPIO();
    initLocalGpio();
    Interrupt_initModule();
    Interrupt_initVectorTable();


    fft_routine(); // <note_1> 105519  sysclk is need. fft routine test

    initBuffer();
    index=0;

    init_timer0();
    Interrupt_register(INT_TIMER1, &cpuTimer0ISR);
    Interrupt_register(INT_ADCA1, &adcA1ISR);

    setDacCI();
    initSCICFIFO();
    initADC();

    initEPWM();
    initADCSOC();
    initDmaCopy();
    index = 0;
    bufferFull = 0;

    Interrupt_enable(INT_ADCA1);
    Interrupt_enable(INT_TIMER1);
    Interrupt_enable(INT_EPWM2);
    Interrupt_enable(INT_SCIC_RX);
    Interrupt_enable(INT_DMA_CH6);
    //Interrupt_enable(INT_SCIC_TX);

    EINT;//enable inturrupt
    ERTM;//enable debug enable
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    //DMA_startChannel(DMA_CH6_BASE);
    //Test for fft...if request_fft is assigned then copy data to fft memory also.
    //UARTprintf((uint16_t *)"dir");
    DMA_configAddresses(DMA_CH6_BASE,RFFTin1Buff , (const void *)adcAResults_3);
    DMA_startChannel(DMA_CH6_BASE);
    SCI_writeCharArray(SCIC_BASE , (uint16_t *)"12345", 5);
    const void *srcAddr;
    while(1)
    {
        if(bufferFull ){
            // FFT Memory Filled with valid data.
            //request_fft
            bufferFull = 0;
            srcAddr =(const void *)(adcAResults_1 + (int)(request_fft/5)*0x1000 + RESULTS_BUFFER_SIZE*(request_fft%5)  ) ;
            // HWREGH(RAM_ADCBUFFER1 + (int)(request_fft/5)*0x1000 + RESULTS_BUFFER_SIZE*(request_fft%5) ) ;
            DMA_configAddresses(DMA_CH6_BASE,RFFTin1Buff , srcAddr);

            int i=0;
            while(!dmaCopydone){
                i++;
                DMA_forceTrigger(DMA_CH6_BASE); //DEVICE_DELAY_US(1);
            }
            request_fft++;// FFT data was Requested, And finished. It will be restart when receive valid request number again.
            if(request_fft>20)request_fft=0;
        }
        if(dmaCopydone)// After bufferFull then excute fft_routine   68uS
        {
            fft_routine(); // <note_1> 105519  sysclk is need. fft routine test
            dmaCopydone=0;
            DMA_startChannel(DMA_CH6_BASE);
        }
        //GPIO_togglePin(BLINKY_LED_GPIO );
        //DEVICE_DELAY_US(500000);
    }
}

//
// Function to configure and power up ADCA.
//
void initADC(void)
{
    int xx;
    for(xx=0;xx<4*0x80;xx+=0x80){
        ADC_setPrescaler(ADCA_BASE+xx, ADC_CLK_DIV_4_0); // Set ADCCLK divider to /4
        ADC_setMode(ADCA_BASE+xx, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED); // // Set resolution and signal mode (see #defines above) and load // corresponding trims.
        ADC_setInterruptPulseMode(ADCA_BASE+xx, ADC_PULSE_END_OF_CONV); // Set pulse positions to late
        ADC_enableConverter(ADCA_BASE+xx); // Power up the ADC and then delay for 1 ms
    };
    DEVICE_DELAY_US(1000);
}

// Function to configure ePWM1 to generate the SOC.
void initEPWM(void)
{
EPWM_SignalParams pwmSignal =
            {ADC_SAMPLING_FREQ, 0.5f, 0.5f, true, DEVICE_SYSCLK_FREQ, SYSCTL_EPWMCLK_DIV_2,
             EPWM_COUNTER_MODE_DOWN, EPWM_CLOCK_DIVIDER_1,
            EPWM_HSCLOCK_DIVIDER_1};
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    EPWM_setADCTriggerSource(EPWM1_BASE,EPWM_SOC_A,EPWM_SOC_TBCTR_ZERO);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);
    EPWM_configureSignal(EPWM1_BASE, &pwmSignal);
}

// Function to configure ADCA's SOC0 to be triggered by ePWM1.
void initADCSOC(void)
{
    //AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    // For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz // SYSCLK rate) will be used.
    //ADCA_BASE
    // max number 8 for adca, 8 * 44 sysclk cycle  352 * 5us = 1.762uS
#ifndef DAC_TEST_USED
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 15);//VIN_R
#endif
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 15);//VSTS_R
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 15);//VO_R
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 15);//IIN_R
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN4, 15);//IINV_R
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN5, 15);//IO_R
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN14, 15);//THR1
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER7, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN15, 15);//THR2

    //ADCB_BASE
       ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 15);//VIN_S
       //ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 15);//VSTS_S1
       ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 15);//VO_S
       ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 15);//IIN_S

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
#ifndef DAC_TEST_USED
    ADC_setBurstModeConfig(ADCA_BASE,ADC_TRIGGER_EPWM1_SOCA , 8);
#else
    ADC_setBurstModeConfig(ADCA_BASE,ADC_TRIGGER_EPWM1_SOCA , 7);
#endif
    ADC_setSOCPriority(ADCA_BASE,ADC_PRI_ALL_HIPRI);
    ADC_enableBurstMode(ADCA_BASE);

    ADC_setBurstModeConfig(ADCB_BASE,ADC_TRIGGER_EPWM1_SOCA , 3);
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
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);//
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
//extern Uint16 sineEnable ;
//extern Uint16 dacOffset;
//extern int QuadratureTable[40];
__interrupt void adcA1ISR(void)  // note_2
{
    GPIO_togglePin(PULSE_OUTPUT_GPIO );
    if( ADC_getInterruptStatus(ADCB_BASE,ADC_INT_NUMBER1)){
#ifndef DAC_TEST_USED
        adcAResults_1[index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
#endif
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
        //adcAResults_10[index] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_10[index] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_11[index] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2);
    }
    else fail++;
    if( ADC_getInterruptStatus(ADCC_BASE,ADC_INT_NUMBER1)){
        adcAResults_12[index] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
        adcAResults_13[index] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_14[index] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER3);
    }
    else fail++;
    if( ADC_getInterruptStatus(ADCD_BASE,ADC_INT_NUMBER1)){
        adcAResults_15[index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0);
        adcAResults_16[index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_17[index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER2);
        adcAResults_18[index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER3);
        adcAResults_19[index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER4);
    }
    else fail++;
    index++;
    // Set the bufferFull flag if the buffer is full
    if(RESULTS_BUFFER_SIZE <= index)
    {
        // write data to fft array.
        // if request_fft is set to 21 or above... This data do not write.
        // It takes 11309 cpu cycle.
        //for(index=0;index<RESULTS_BUFFER_SIZE ;index++)
        //    RFFTin1Buff[index] =  HWREGH(RAM_ADCBUFFER1 + (int)(request_fft/5)*0x1000 + RESULTS_BUFFER_SIZE*(request_fft%5) +index) ;
        index = 0;
        bufferFull = 1;
    }
    if (ToggleCount++ >= 15)
    {
        //GPIO_togglePin(PULSE_OUTPUT_GPIO );
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
    // Acknowledge the interrupt
    // ADCA1 and  ADCB1 and  ADCC1 and  ADCD1 are GROUP1
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
