// Adc Data 버퍼는 1024개
// 데이라 PWM :  8Khz
// 따라서 FFT수행 간격은 1024*8khz

#include "driverlib.h"
#include "device.h"

#include <string.h>
#include "usb_hal.h"
#include "usblib.h"
#include "usbmsc.h"
#include "host/usbhost.h"
#include "host/usbhmsc.h"
#include "c2000ware_libraries.h"
#include "scistdio.h"
#include "fatfs/src/ff.h"

#include "board.h"
#include "F2837xD_device.h"
#include "inc/hw_ipc.h"
#include "fpu_rfft.h"            // Main include file
#include "math.h"
#include "DiagnosysUps.h"
#include <uart/uart485_B.h>
#include <uart/uart_C.h>
#include <uart/uart_D.h>
#include <uart/uart_util.h>
#include "ds1338z_Rtc.h"
#include "cpuFlashMemory.h"


#include "usblib.h"

//#include "dmaCopy.h"
// Defines
#pragma DATA_SECTION(RFFTin1Buff,"RFFTdata1")
uint16_t RFFTin1Buff[2*RFFT_SIZE];
//extern uint16_t RFFTin1Buff_test[2*RFFT_SIZE];

#pragma DATA_SECTION(RFFTmagBuff,"RFFTdata2")
float RFFTmagBuff[RFFT_SIZE/2+1];

#pragma DATA_SECTION(RFFToutBuff,"RFFTdata3")
float RFFToutBuff[RFFT_SIZE];

#pragma DATA_SECTION(RFFTF32Coef,"RFFTdata4")
float RFFTF32Coef[RFFT_SIZE];

#pragma DATA_SECTION(adcAResults_1,"ADCBUFFER1")
#pragma DATA_SECTION(adcAResults_2,"ADCBUFFER1")
#pragma DATA_SECTION(adcAResults_3,"ADCBUFFER1")
#pragma DATA_SECTION(adcAResults_4,"ADCBUFFER1")

#pragma DATA_SECTION(adcAResults_5,"ADCBUFFER2")
#pragma DATA_SECTION(adcAResults_6,"ADCBUFFER2")
#pragma DATA_SECTION(adcAResults_7,"ADCBUFFER2")
#pragma DATA_SECTION(adcAResults_8,"ADCBUFFER2")

#pragma DATA_SECTION(adcAResults_9,"ADCBUFFER3")
#pragma DATA_SECTION(adcAResults_10,"ADCBUFFER3")
#pragma DATA_SECTION(adcAResults_11,"ADCBUFFER3")
#pragma DATA_SECTION(adcAResults_12,"ADCBUFFER3")

#pragma DATA_SECTION(adcAResults_13,"ADCBUFFER4")
#pragma DATA_SECTION(adcAResults_14,"ADCBUFFER4")
#pragma DATA_SECTION(adcAResults_15,"ADCBUFFER4")
#pragma DATA_SECTION(adcAResults_16,"ADCBUFFER4")

#pragma DATA_SECTION(adcAResults_17,"ADCBUFFER5")
#pragma DATA_SECTION(adcAResults_18,"ADCBUFFER5")
#pragma DATA_SECTION(adcAResults_19,"ADCBUFFER5")
#pragma DATA_SECTION(adcAResults_20,"ADCBUFFER5")

#define userFlashStart         0xBE000

// Save PWM Frequency For ADC Period.
// Saved 4byte, 00 01 Reading Frequency
// for example at 80,0000
// 00 -> 0x3880 01-> 0x0001
#define SystemInitFlash         0xBE000+24


struct st_fft_result {
    float maxValue;
    float freq;
    float THD ;
};

#pragma  DATA_SECTION(offsetValue,"GETBUFFER")
uint16_t offsetValue[24];

#pragma DATA_SECTION(fft_result,"PUTBUFFER")
struct st_fft_result fft_result[5][20];

#pragma DATA_SECTION(time,"PUTBUFFER")
struct rtctime_t time;

#pragma DATA_SECTION(request_fft,"PUTBUFFER")
uint16_t request_fft;
//RtcTime

#define RAM_BANK_SIZE  0x00010000
#define RAM_ADCBUFFER1 0x00018000
#define RAM_ADCBUFFER2 0x00019000
#define RAM_ADCBUFFER3 0x0001A000
#define RAM_ADCBUFFER4 0x0001B000

#define BLINKY_LED_GPIO     81
#define PULSE_OUTPUT_GPIO 18

#define EPSILON         0.1

#define DAC_TEST_USED   1

#ifdef DAC_TEST_USED
extern void setDacCI(void);
#endif

#define SD_DETECT 57
#define SPI_CS 61

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
uint16_t fail = 0;  // Adc Fail Count. Not used
static uint16_t isMemCpyDoneFFT;// When this is set, Interrupt routine copy next data for fftroutine.

uint16_t adc_index;                              // Index into result buffer



#pragma CODE_SECTION(myFunction,".TI.ramfunc")
void myFunction()
{
    SCIprintf("Test");
}

void initADC(void);
void initEPWM(void);
void EPWM_changeClock(uint32_t base, float32_t ClkInHz );
void initADCSOC(void);
void fft_routine(void);
__interrupt void adcA1ISR(void);
void INT_myUSB0_ISR(void);
__interrupt void pwmE3ISR(void);
void MSCCallback(tUSBHMSCInstance *psMSCInstance, uint32_t ui32Event, void *pvEventData);

//! <table>
//! <caption id="multi_row">Performance Data</caption>
//! <tr><th> Samples <th> Cycles
//! <tr><td> 64      <td> 1295
//! <tr><td> 128     <td> 2769
//! <tr><td> 256     <td> 6059
//! <tr><td> 512     <td> 13360  * 5ns -> 66800ns ->66us
//! <tr><td> 1024    <td> 29466
//! </table>
uint32_t ToggleCount = 0;  //For LED Status Toggle
const void *adcsrcAddr[20];  // Contain Adcbuffer address
const void *srcAddr;         //
static float32_t pwmFrequency;  //Adc Reading Frequency.
                                //This value is defined dafault, And can modified from CLI at CPU2



#define PATH_BUF_SIZE   80
#define CMD_BUF_SIZE    64
static char g_cCwdBuf[PATH_BUF_SIZE] = "/";
static char g_cTmpBuf[PATH_BUF_SIZE];
static char g_cCmdBuf[CMD_BUF_SIZE];

static FATFS g_sFatFs;
static DIR g_sDirObject;
static FILINFO g_sFileInfo;
static FIL g_sFileObject;
typedef struct
{
    FRESULT fresult;
    char *pcResultStr;
}
tFresultString;
#define FRESULT_ENTRY(f)        { (f), (#f) }
tFresultString g_sFresultStrings[] =
{
    FRESULT_ENTRY(FR_OK),
    FRESULT_ENTRY(FR_NOT_READY),
    FRESULT_ENTRY(FR_NO_FILE),
    FRESULT_ENTRY(FR_NO_PATH),
    FRESULT_ENTRY(FR_INVALID_NAME),
    FRESULT_ENTRY(FR_INVALID_DRIVE),
    FRESULT_ENTRY(FR_DENIED),
    FRESULT_ENTRY(FR_EXIST),
    FRESULT_ENTRY(FR_RW_ERROR),
    FRESULT_ENTRY(FR_WRITE_PROTECTED),
    FRESULT_ENTRY(FR_NOT_ENABLED),
    FRESULT_ENTRY(FR_NO_FILESYSTEM),
    FRESULT_ENTRY(FR_INVALID_OBJECT),
    FRESULT_ENTRY(FR_MKFS_ABORTED)
};

#define NUM_FRESULT_CODES (sizeof(g_sFresultStrings) / sizeof(tFresultString))
tUSBHMSCInstance *g_psMSCInstance = 0;
DECLARE_EVENT_DRIVER(g_sUSBEventDriver, 0, 0, USBHCDEvents);
static tUSBHostClassDriver const * const g_ppHostClassDrivers[] =
{
    &g_sUSBHostMSCClassDriver,
    &g_sUSBEventDriver
};

#define NUM_CLASS_DRIVERS       (sizeof(g_ppHostClassDrivers)                 /\
                                 sizeof(g_ppHostClassDrivers[0]))
typedef enum
{
    //
    // No device is present.
    //
    STATE_NO_DEVICE,

    //
    // Mass storage device is being enumerated.
    //
    STATE_DEVICE_ENUM,

    //
    // Mass storage device is ready.
    //
    STATE_DEVICE_READY,

    //
    // An unsupported device has been attached.
    //
    STATE_UNKNOWN_DEVICE,

    //
    // A power fault has occurred.
    //
    STATE_POWER_FAULT
} tState;
volatile tState g_eState;
volatile tState g_eUIState;
tUSBMode g_eCurrentUSBMode;
void ModeCallback(uint32_t ui32Index, tUSBMode eMode)
{
    //
    // Save the new mode.
    //

    g_eCurrentUSBMode = eMode;
}
void
USBHCDEvents(void *pvData)
{
    tEventInfo *pEventInfo;

    //
    // Cast this pointer to its actual type.
    //
    pEventInfo = (tEventInfo *)pvData;

    switch(pEventInfo->ui32Event)
    {
        //
        // New keyboard detected.
        //
    case USB_EVENT_UNKNOWN_CONNECTED:
    {
        //
        // An unknown device was detected.
        //
        g_eState = STATE_UNKNOWN_DEVICE;

        break;
    }

    //
    // Keyboard has been unplugged.
    //
    case USB_EVENT_DISCONNECTED:
    {
        //
        // Unknown device has been removed.
        //
        g_eState = STATE_NO_DEVICE;

        break;
    }

    case USB_EVENT_POWER_FAULT:
    {
        //
        // No power means no device is present.
        //
        g_eState = STATE_POWER_FAULT;

        break;
    }

    default:
    {
        break;
    }
    }
}
/////////  END  USB  //

void fft_routine(void)
{
    float sumTotal=0.0;
    float maxValue=0.0;
    float freq = 0.0;
    uint16_t i, j;

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

    int nth=0;
    for(nth=0;nth<5;nth++)
    {
            fft_result[nth][request_fft].THD=(float)0;
            fft_result[nth][request_fft].freq=(float)0;
            fft_result[nth][request_fft].maxValue=(float)0;
    }

    j = 1;  // j is maxValue position
    freq = RFFTmagBuff[1];
    i=2;   // i is the start position



    for(nth=0;nth<5;nth++){
        for(;i<RFFT_SIZE/2+1;i++){
            //Looking for the maximum component of frequency spectrum
            if(RFFTmagBuff[i] > freq){
                j = i;
                freq = RFFTmagBuff[i];
            }
        }
        if(nth==0)maxValue=RFFTmagBuff[j];
        fft_result[nth][request_fft].maxValue = RFFTmagBuff[j];
        freq =(float)F_PER_SAMPLE * (float)j;
        fft_result[nth][request_fft].freq = freq;
        j++;
        i = j; //최대값을 찾은 다음 부터 다시 시작한다.
        freq = RFFTmagBuff[i];
    }

    i=2;
    //for(;i<RFFT_SIZE/2+1;i++){
   for(;i<RFFT_SIZE/2;i++){
       sumTotal += RFFTmagBuff[i]*RFFTmagBuff[i];
    }
    sumTotal=sqrt(sumTotal);
    float THD=sumTotal/maxValue;
    fft_result[0][request_fft].THD = THD;

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
    //THD=THD;
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

}

inline void initBuffer()
{
    uint16_t index;                              // Index into result buffer
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
    if(cpuTimer0IntCount > 5)
    {
        cpuTimer0IntCount =0;
        GPIO_togglePin(BLINKY_LED_GPIO );
    }

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

//
void setupCpu2(){
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL5_SCI,3,SYSCTL_CPUSEL_CPU1);//SCI_C is 3
    GPIO_setPinConfig(GPIO_90_SCIRXDC);
    GPIO_setPinConfig(GPIO_89_SCITXDC);


    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL6_SPI,1,SYSCTL_CPUSEL_CPU1);//SPI_A is 1
	GPIO_setPinConfig(GPIO_57_GPIO57); //GPIO57 -> SD_DETECT Pinmux
	GPIO_setPinConfig(GPIO_61_GPIO61); //GPIO61 -> SPI_CS Pinmux

	GPIO_setPinConfig(GPIO_58_SPISIMOA);
	GPIO_setPinConfig(GPIO_59_SPISOMIA);
	GPIO_setPinConfig(GPIO_60_SPICLKA);
	GPIO_setPinConfig(GPIO_61_SPISTEA);

	//SD_DETECT initialization
	GPIO_setDirectionMode(SD_DETECT, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(SD_DETECT, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(SD_DETECT, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(SD_DETECT, GPIO_QUAL_SYNC);

	//SPI_CS initialization
	GPIO_setDirectionMode(SPI_CS, GPIO_DIR_MODE_OUT);
	GPIO_setPadConfig(SPI_CS, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(SPI_CS, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(SPI_CS, GPIO_QUAL_SYNC);


	MemCfg_setGSRAMControllerSel(MEMCFG_SECT_GS2,MEMCFG_GSRAMCONTROLLER_CPU1);
	MemCfg_setGSRAMControllerSel(MEMCFG_SECT_GS3,MEMCFG_GSRAMCONTROLLER_CPU1);
	MemCfg_setGSRAMControllerSel(MEMCFG_SECT_GS4,MEMCFG_GSRAMCONTROLLER_CPU1);
	MemCfg_setGSRAMControllerSel(MEMCFG_SECT_GS5,MEMCFG_GSRAMCONTROLLER_CPU1);
	MemCfg_setGSRAMControllerSel(MEMCFG_SECT_GS7,MEMCFG_GSRAMCONTROLLER_CPU1);
	//MemCfg_unlockConfig(MEMCFG_SECT_GS5);
    //SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL5_SCI,3,SYSCTL_CPUSEL_CPU2);//SCI_C is 3
    //EALLOW;
    //DevCfgRegs.CPUSEL5.bit.SCI_C = 1;
    //EDIS;
}
void cheeck_ipc()
{
    //IPC 21번 시간요청
    uint16_t i;
    uint16_t response=0;
    //uint32_t value;
    uint32_t command;
    uint32_t data;
    uint16_t flashInitValue[8];// [0] store frequency High 16bit
                               // [1] store Frequency Low 16Bit
    for(i=0;i<8;i++)flashInitValue[i]= 0x00;
    command = HWREG(IPC_BASE +  IPC_O_RECVCOM)  ;
    data = HWREG(IPC_BASE +  IPC_O_RECVDATA)  ;
    if(  ((HWREG(IPC_BASE + IPC_O_STS)) & IPC_SET_IPC30) == IPC_SET_IPC30 )   // CPU2에서 뭔가를 요청한다.
    {
        //IPCRECVCMD
        //IPCRECVDATA
        //IPCRECVDADDR
        //IPCLOCALREPLY
        // CMD에 1의 값이 실려 오면 읽는 주파수를 변경하라는 메세지 이며 DATA에서  값을 취한다.
        // IPCRECVDATA에 유효한 값이 실려 오면 이 값으로 설정한다.
        // IPCLOCALREPLY에는 설정후 결과값을 리턴한다.
        //최종적으로 ACK를 보내서 Local의 Status를 Clear하며 Remote의 Flag를 Clear한다.
        if(command==1)
        {
            data = HWREG(IPC_BASE +  IPC_O_RECVDATA)  ;
            //여기서 필요한 펑션을 수행한다.
            flashInitValue[0] = (uint16_t)(data & 0x0000FFFF );
            flashInitValue[1] = data >> 16;
            if(data>0){
                DINT;
                CallFlashAPI(userFlashStart+24,flashInitValue,8);
                EINT;
                //initEPWM();
                //EPWM_changeClock(EPWM1_BASE,(float32_t) data);
            }
            data = HWREG(userFlashStart+24);
            if(data == 0xFFFFFFFF) data  = ADC_SAMPLING_FREQ;
            HWREG(IPC_BASE +  IPC_O_LOCALREPLY) = data  ;//SystemInitFlash
            HWREG(IPC_BASE +  IPC_O_ACK) = IPC_SET_IPC30  ;
        }
        else if(command==2)
        {
            request_fft =data;// 인터럽트루틴에게 복사할 버퍼의 위치를 알려 준다.
            isMemCpyDoneFFT=0;  //이렇게 하면 인터럽트에서 메모리를 복사한다.
            while(!response )
            {
                response= isMemCpyDoneFFT;
                SysCtl_delay(1);
            }
            HWREG(IPC_BASE +  IPC_O_SENDDATA) = ToggleCount ; //use Test Print
                                                                //adc_index;// request_fft;//HWREGH( (unsigned long)srcAddr + index++);//100  ;
            HWREG(IPC_BASE +  IPC_O_ACK) = IPC_SET_IPC30  ;
            DEVICE_DELAY_US(30);// CPU2에서 데이타를 갖고 갈수 있는 시간을 준다. 데이타 갯수 = 5ns*((1024*2cycle) +4cycle)=
        }
    }

    if(  ((HWREG(IPC_BASE + IPC_O_STS)) & IPC_SET_IPC21) == IPC_SET_IPC21 )   // 시간을 메모리에 옮겨 달라는 요청을 받으면 ...
    {
        ds1338_read_time(&time);
        HWREG(IPC_BASE + IPC_O_ACK) = IPC_ACK_IPC21;
    }
    if(  ((HWREG(IPC_BASE + IPC_O_STS)) & IPC_SET_IPC22) == IPC_SET_IPC22 )
    {
        //FFT메모리를 읽기 위함이다.
        //FFT가 수행되지 않는 상태에서 읽으면 되니까.
        //잠깐의 Delay를  주면 되겠다
        HWREG(IPC_BASE + IPC_O_ACK) = IPC_ACK_IPC22;
        DEVICE_DELAY_US(20);
    }
    if(  ((HWREG(IPC_BASE + IPC_O_STS)) & IPC_SET_IPC23) == IPC_SET_IPC23 )   // OFFSET값이 변경되었음을 알려 준다.
    {
        for(i=0;i<20;i++)HWREGH(userFlashStart+i)=offsetValue[i] ;
        DINT;
        CallFlashAPI(userFlashStart,offsetValue,24);
        EINT;
        HWREG(IPC_BASE + IPC_O_ACK) = IPC_ACK_IPC23;
    }

}
void MSCCallback(tUSBHMSCInstance *psMSCInstance, uint32_t ui32Event, void *pvEventData)
{
    //
    // Determine the event.
    //
    switch(ui32Event)
    {
        //
        // Called when the device driver has successfully enumerated an MSC
        // device.
        //
    case MSC_EVENT_OPEN:
    {
        //
        // Proceed to the enumeration state.
        //
        g_eState = STATE_DEVICE_ENUM;
        break;
    }

    //
    // Called when the device driver has been unloaded due to error or
    // the device is no longer present.
    //
    case MSC_EVENT_CLOSE:
    {
        //
        // Go back to the "no device" state and wait for a new connection.
        //
        g_eState = STATE_NO_DEVICE;

        break;
    }

    default:
    {
        break;
    }
    }
}



void main(void)
{
    Device_init();
    Device_initGPIO();

    adcsrcAddr[0] =(const void *)(adcAResults_1 ), adcsrcAddr[1] =(const void *)(adcAResults_2 ), adcsrcAddr[2] =(const void *)(adcAResults_3 ), adcsrcAddr[3] =(const void *)(adcAResults_4 ),
    adcsrcAddr[4] =(const void *)(adcAResults_5 ), adcsrcAddr[5] =(const void *)(adcAResults_6 ), adcsrcAddr[6] =(const void *)(adcAResults_7 ), adcsrcAddr[7] =(const void *)(adcAResults_8 ),
    adcsrcAddr[8] =(const void *)(adcAResults_9 ), adcsrcAddr[9] =(const void *)(adcAResults_10 ),adcsrcAddr[10] = (const void *)(adcAResults_11 ), adcsrcAddr[11] =(const void *)(adcAResults_12 ),
    adcsrcAddr[12] =(const void *)(adcAResults_13 ),adcsrcAddr[13] = (const void *)(adcAResults_14 ),adcsrcAddr[14] = (const void *)(adcAResults_15 ),adcsrcAddr[15] = (const void *)(adcAResults_16 ),
    adcsrcAddr[16] =(const void *)(adcAResults_17 ),adcsrcAddr[17] = (const void *)(adcAResults_18 ),adcsrcAddr[18] = (const void *)(adcAResults_19 ),adcsrcAddr[19] = (const void *)(adcAResults_20 );



    setupCpu2();

    initLocalGpio();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    Board_init();
    C2000Ware_libraries_init();
    g_eState = STATE_NO_DEVICE;
    g_eUIState = STATE_NO_DEVICE;
    USBGPIOEnable();
    //USBHCDInit(0,g_pHCDPool, HCD_MEMORY_SIZE);

    I2caRegs.I2CCLKH = 0x00;
    request_fft=0;


#ifdef _STANDALONE
#ifdef _FLASH
    //Device_bootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
#else
    //Device_bootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
#endif //_FLASH

#else // _STANDALONE
#ifdef _FLASH
    //Device_bootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
#endif
#endif //_STANDALONE

    fft_routine(); // <note_1> 105519  sysclk is need. fft routine test

    initBuffer();

    init_timer0();
    Interrupt_register(INT_TIMER1, &cpuTimer0ISR);
    Interrupt_register(INT_ADCA1, &adcA1ISR);

#ifdef DAC_TEST_USED
    setDacCI();
#endif

    initSCIDFIFO();
    initSCICFIFO();
    initSCIBFIFO();
    initADC();

    SCIStdioConfig(SCIC_BASE, 115200,
                   SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ));

    initEPWM();

    initADCSOC();
    //Not use Dma copy...
    //initDmaCopy();

    initI2CFIFO();

    isMemCpyDoneFFT =0; // It is used for

    Interrupt_enable(INT_ADCA1);
    Interrupt_enable(INT_TIMER1);
    Interrupt_enable(INT_EPWM2);
    Interrupt_enable(INT_SCIB_RX);
    //Interrupt_enable(INT_SCIB_TX);

    //Interrupt_enable(INT_SCIC_RX);
    Interrupt_enable(INT_SCID_RX);

    //Interrupt_enable(INT_DMA_CH6);

    Interrupt_enable(INT_I2CA_FIFO);
    //Interrupt_enable(INT_SCIC_TX);


    EINT;//enable inturrupt
    ERTM;//enable debug enable
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    DEVICE_DELAY_US(128000);// 125*1024 128us at 8Khz    128us
                              //한번의 ADC를 다 읽는데 걸리는 시간이다.


    ds1338_read_time(&time);
    if(time.year < 23 ){
        make_time(&time,23,7,12,15,01,01);
        ds1338_write_time(&time);
    }

    myFunction();
    SCIprintf("\n\nUSB Mass Storage Host program\n");
    SCIprintf("Type \'help\' for help.\n\n");
    USBHCDRegisterDrivers(0, g_ppHostClassDrivers, NUM_CLASS_DRIVERS);
    g_psMSCInstance = USBHMSCDriveOpen(0, (tUSBHMSCCallback)MSCCallback);
    f_mount(0, &g_sFatFs);

    while(1)
    {
        cheeck_ipc();  // CPU2에서 IPC의 요청이 있는지를 확인한다.
        if(isMemCpyDoneFFT)// After memoryCopyComplete then excute fft_routine   68uS
        {
            // CPU2에서 사용하고 있지 않으면...
           if(((HWREG(IPC_BASE + IPC_O_STS)) & (1UL << (request_fft+1))) == 0U  )
           {
                HWREG(IPC_BASE + IPC_O_SET)= 1UL << (request_fft+1);
                fft_routine(); // <note_1> 105519  sysclk is need. fft routine test
                HWREG(IPC_BASE + IPC_O_CLR) = 1UL << (request_fft+1);
                request_fft++;// FFT data was Requested, And finished. It will be restart when receive valid request number again.
                if(request_fft >= 20)request_fft=0;
                isMemCpyDoneFFT=0;
           }
           else{};//현재의 메모리를 CPU에서 사용하고 있는경우..
        }
    }
}

        //GPIO_togglePin(BLINKY_LED_GPIO );
        //DEVICE_DELAY_US(500000);
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

    for(xx=0;xx<20;xx++) offsetValue[xx] = HWREGH(userFlashStart+xx);

    //offsetValue
    //signed int trimValue ;
    //signed int Value ;

    //    EALLOW;
    //trimValue = 21;//HWREGH(userFlashStart) ;
    //HWREGH(ADCA_BASE + ADC_O_OFFTRIM ) += 0x70;
    //HWREGH(ADCA_BASE + ADC_O_OFFTRIM ) -=  120;//(trimValue*16)/4;
    //Value += 0x70;
    //Value = Value + (trimValue*16)/32;
    //Value = Value/32;
    //trimValue = trimValue -Value;
    //HWREGH(ADCA_BASE + ADC_O_OFFTRIM ) = Value;
    //EDIS;

    DEVICE_DELAY_US(1000);
}

void EPWM_changeClock(uint32_t base, float32_t ClkInHz )
{
    float32_t tbClkInHz = 0.0F;
    uint16_t tbPrdVal = 0U, cmpAVal = 0U, cmpBVal = 0U;
    EPWM_SignalParams sigParams =
            {ClkInHz , 0.5f, 0.5f, true, DEVICE_SYSCLK_FREQ, SYSCTL_EPWMCLK_DIV_2,
             EPWM_COUNTER_MODE_DOWN, EPWM_CLOCK_DIVIDER_1,
            EPWM_HSCLOCK_DIVIDER_1};

    EPWM_SignalParams *signalParams = &sigParams  ;

    ASSERT(EPWM_isBaseValid(base));

    tbClkInHz = ((float32_t)signalParams->sysClkInHz /
                (1U << ((uint16_t)signalParams->epwmClkDiv +
                 (uint16_t)signalParams->tbClkDiv)));

    tbClkInHz /= (2U * (uint16_t)signalParams->tbHSClkDiv);

    tbPrdVal = (uint16_t)((tbClkInHz / signalParams->freqInHz) - 1.0f);
    cmpAVal = (uint16_t)((tbPrdVal + 1U) -
                   ((float32_t)signalParams->dutyValA * (tbPrdVal + 1U)));
    cmpBVal = (uint16_t)((tbPrdVal + 1U) -
                       ((float32_t)signalParams->dutyValB * (tbPrdVal + 1U)));
    // Configure TBPRD value
    EPWM_setTimeBasePeriod(base, tbPrdVal);
    // Set Compare values
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, cmpAVal);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, cmpBVal);
}
// Function to configure ePWM1 to generate the SOC.
void initEPWM(void)
{
    pwmFrequency= (float32_t) HWREG(userFlashStart+24);
    if(pwmFrequency < 0  || pwmFrequency > 2000000 ) pwmFrequency= ADC_SAMPLING_FREQ;

    //EPWM1_BASE

    EPWM_SignalParams pwmSignal =
            {pwmFrequency, 0.5f, 0.5f, true, DEVICE_SYSCLK_FREQ, SYSCTL_EPWMCLK_DIV_2,
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
    ADC_setBurstModeConfig(ADCA_BASE,ADC_TRIGGER_EPWM1_SOCA , 8);
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
    ToggleCount++;
    if( ADC_getInterruptStatus(ADCA_BASE,ADC_INT_NUMBER1)){
        adcAResults_1[adc_index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
        adcAResults_2[adc_index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_3[adc_index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2);
        adcAResults_4[adc_index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER3);
        adcAResults_5[adc_index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4);
        adcAResults_6[adc_index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5);
        adcAResults_7[adc_index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER6);
        adcAResults_8[adc_index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER7);
    }
    else fail++;
    if( ADC_getInterruptStatus(ADCB_BASE,ADC_INT_NUMBER1)){
        adcAResults_9[adc_index] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);
        //adcAResults_10[adc_index] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_10[adc_index] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_11[adc_index] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2);
    }
    else fail++;
    if( ADC_getInterruptStatus(ADCC_BASE,ADC_INT_NUMBER1)){
        adcAResults_12[adc_index] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
        adcAResults_13[adc_index] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_14[adc_index] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER3);
    }
    else fail++;
    if( ADC_getInterruptStatus(ADCD_BASE,ADC_INT_NUMBER1)){
        adcAResults_15[adc_index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0);
        adcAResults_16[adc_index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER1);
        adcAResults_17[adc_index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER2);
        adcAResults_18[adc_index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER3);
        adcAResults_19[adc_index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER4);
    }
    else fail++;
    adc_index++;
    // Set the memoryCopyComplete flag if the buffer is full:w

    int i;
    if(!isMemCpyDoneFFT ){
        srcAddr = adcsrcAddr[request_fft];
        uint16_t index;
        index = adc_index;
        for(i=0;i < RESULTS_BUFFER_SIZE;i++){
            if( index >= RESULTS_BUFFER_SIZE ) index=0;
            //메모리는 현재위치 다음데이타가 가장 마지막 데이타이다 이미 증가가 되어 있는 상태이다.
            HWREGH(RFFTin1Buff+i) = HWREGH( (unsigned long)srcAddr + index++);
        }
        isMemCpyDoneFFT  = 1;
        //memoryCopyComplete=1;
    }
    if(RESULTS_BUFFER_SIZE <= adc_index)
    {
        adc_index = 0;
        // write data to fft array.
        // if request_fft is set to 21 or above... This data do not write.
        // It takes 11309 cpu cycle.
        //for(adc_index=0;adc_index<RESULTS_BUFFER_SIZE ;adc_index++)
        //    RFFTin1Buff[adc_index] =  HWREGH(RAM_ADCBUFFER1 + (int)(request_fft/5)*0x1000 + RESULTS_BUFFER_SIZE*(request_fft%5) +adc_index) ;
        //memoryCopyComplete = 1;
    }
    if (ToggleCount >= pwmFrequency/2)
    {
        GPIO_togglePin(PULSE_OUTPUT_GPIO );
        GPIO_togglePin(BLINKY_LED_GPIO );
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

//******************************************************************************
//
//! Host interrupt service routine wrapper to make ISR compatible with
//! C2000 PIE controller.
//
//******************************************************************************
__interrupt void
INT_myUSB0_ISR(void)
{
    //USB0HostIntHandler();
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
