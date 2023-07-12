#include "ds1338z_Rtc.h"

//
// Defines
//
uint16_t sData[8];                  // Send data buffer
uint16_t rData[8];                  // Receive data buffer
                                    // data stream to check received data
#define SLAVE_ADDRESS   0x68

void initI2CFIFO(void);
__interrupt void i2cFIFOISR(void);

int read_rtc(uint8_t addr);
void write_eeprom(uint8_t addr,uint8_t *wdata,uint8_t dataCount );
int receiveDataCount;
uint8_t isReadFinished=0;

void initI2CFIFO()
{
    //I2CA -> myI2C0 Pinmux
    GPIO_setPinConfig(GPIO_32_SDAA);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(32, GPIO_QUAL_ASYNC);
    GPIO_setPinConfig(GPIO_33_SCLA);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(33, GPIO_QUAL_ASYNC);

    Interrupt_register(INT_I2CA_FIFO, &i2cFIFOISR);

    I2C_disableModule(I2CA_BASE);
    I2C_initMaster(I2CA_BASE, DEVICE_SYSCLK_FREQ, 400000, I2C_DUTYCYCLE_50);
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE /*I2C_MASTER_SEND_MODE*/);
    I2C_setSlaveAddress(I2CA_BASE, 104);
    I2C_disableLoopback(I2CA_BASE);
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);
    I2C_setDataCount(I2CA_BASE, 2);
    I2C_setAddressMode(I2CA_BASE, I2C_ADDR_MODE_7BITS);
    I2C_enableFIFO(I2CA_BASE);
    I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_RXFF);
    I2C_setFIFOInterruptLevel(I2CA_BASE, I2C_FIFO_TX1, I2C_FIFO_RX7);
    I2C_enableInterrupt(I2CA_BASE, I2C_INT_RXFF);
    I2C_setEmulationMode(I2CA_BASE, I2C_EMULATION_STOP_SCL_LOW);
    I2C_enableModule(I2CA_BASE);
}

__interrupt void i2cFIFOISR(void)
{
   uint16_t i;
   if((I2C_getInterruptStatus(I2CA_BASE) & I2C_INT_RXFF) != 0)
   {
       for(i = 0; i < receiveDataCount; i++)
       {
           rData[i] = I2C_getData(I2CA_BASE);
       }
       while(!(I2C_getStatus(I2CA_BASE) & I2C_STS_STOP_CONDITION));
       I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_RXFF);
       isReadFinished=1;
   }
   Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}

void make_time(struct rtctime_t *time, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    time->year = year;
    time->month = month;
    time->day = day;
    time->hour = hour;
    time->minute = minute;
    time->second = second;
}
/**
 * \brief Output a string representation of a struct rtctime_t object.
 *
 * Requires a buffer size of at least 20 characters.
 *
 * \param time A pointer to a struct rtctime_t instance.
 * \param buf A pointer to a character buffer in which to store the string.
 */
void format_time_str(struct rtctime_t *time, char *buf)
{
    // Year
    buf[0] = '2';
    if (time->year < 10)
    {
        buf[1] = '0';
        buf[2] = '0';
        buf[3] = '0' + time->year;
    }
    else if (time->year < 100)
    {
        buf[1] = '0';
        buf[2] = '0' + (time->year / 10);
        buf[3] = '0' + (time->year % 10);
    }
    else
    {
        buf[1] = '1';
        buf[2] = '0' + ((time->year - 100) / 10);
        buf[3] = '0' + ((time->year - 100) % 10);
    }
    buf[4] = '-';

    // Month
    if (time->month < 10)
    {
        buf[5] = '0';
        buf[6] = '0' + time->month;
    }
    else
    {
        buf[5] = '0' + (time->month / 10);
        buf[6] = '0' + (time->month % 10);
    }
    buf[7] = '-';

    // Day
    if (time->day < 10)
    {
        buf[8] = '0';
        buf[9] = '0' + time->day;
    }
    else
    {
        buf[8] = '0' + (time->day / 10);
        buf[9] = '0' + (time->day % 10);
    }
    buf[10] = 'T';

    // Hour
    if (time->hour < 10)
    {
        buf[11] = '0';
        buf[12] = '0' + time->hour;
    }
    else
    {
        buf[11] = '0' + (time->hour / 10);
        buf[12] = '0' + (time->hour % 10);
    }
    buf[13] = ':';

    // Minute
    if (time->minute < 10)
    {
        buf[14] = '0';
        buf[15] = '0' + time->minute;
    }
    else
    {
        buf[14] = '0' + (time->minute / 10);
        buf[15] = '0' + (time->minute % 10);
    }
    buf[16] = ':';

    // Second
    if (time->second < 10)
    {
        buf[17] = '0';
        buf[18] = '0' + time->second;
    }
    else
    {
        buf[17] = '0' + (time->second / 10);
        buf[18] = '0' + (time->second % 10);
    }

    buf[19] = 0;
}
int i2c_read(uint8_t addr ,uint8_t *buf, uint8_t num)
{
    uint16_t data_addr ;
    receiveDataCount = num;
    data_addr = addr & 0xff;
    I2C_setSlaveAddress(I2CA_BASE,SLAVE_ADDRESS);
    while(I2C_getStopConditionStatus(I2CA_BASE));//1. TX Data보내고 STOP 후 0으로 리셋됨
    while(I2C_isBusBusy(I2CA_BASE));//2. 0:Bus free. 1: Bus busy
    I2C_enableModule(I2CA_BASE);
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE /*I2C_MASTER_SEND_MODE*/);
    I2C_setDataCount(I2CA_BASE,1);
    I2C_putData(I2CA_BASE, data_addr);//5. Data Address  //I2caRegs.I2CDXR.all = data_addr;
    I2C_sendStartCondition(I2CA_BASE);//I2caRegs.I2CMDR.all = 0x2620;       //6. START Send && CLK Send && TX Mode SET && I2C Enable
    // NACKNOD 0 // FREE  0 // STT 1 //  reserved
    // STP 0  Stop Condition bit // MST 1 Master mode bit // TRX 1 transmitter mode bit // XA 0 expanded address
    // RM 0  repeat mode // DLB 0 Digital Loop Backmode // IRS 1 I2C module reset bit -- // STB 0 A Start Condition
    // FDF 0 Free data format // BC 0 0 0  8 BIT
    while(!(I2C_getStatus(I2CA_BASE) & I2C_STS_REG_ACCESS_RDY));//while(I2caRegs.I2CSTR.bit.ARDY==0); //7. 보낸 데이터 보내고 I2C 모듈 레지스터에 액세스 할 준비가되었음을 나타냄.
    while(I2C_getStopConditionStatus(I2CA_BASE)); //while(I2C_getStopConditionStatus(I2CA_BASE));
    I2C_setConfig(I2CA_BASE,I2C_CONTROLLER_RECEIVE_MODE /*I2C_MASTER_RECEIVE_MODE*/);
    I2C_setDataCount(I2CA_BASE,receiveDataCount);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE); //I2caRegs.I2CMDR.all = 0x2C20;   //11. START Send && CLK Send && RX Mode SET && I2C Enable && STOP Send

    while(!isReadFinished)DEVICE_DELAY_US(1);
    isReadFinished=0;
    uint16_t i;
    for(i=0;i<receiveDataCount;i++) buf[i]=rData[i];
    return 0;
}

void i2c_write(uint8_t addr,uint8_t *wdata,uint8_t dataCount )
{
    int i=0;
    //for(i=0;i<dataCount;i++)buf[i]  =  wdata[i];
    uint16_t data_addr ;
    data_addr = addr & 0xff;
    dataCount++;// For Address count increase
    I2C_enableModule(I2CA_BASE);
    I2C_setSlaveAddress(I2CA_BASE,SLAVE_ADDRESS);
    I2C_setEmulationMode(I2CA_BASE,I2C_EMULATION_FREE_RUN);
    while(I2C_getStopConditionStatus(I2CA_BASE));//1. TX Data보내고 STOP 후 0으로 리셋됨
    while(I2C_isBusBusy(I2CA_BASE));//2. 0:Bus free. 1: Bus busy
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE /*I2C_MASTER_SEND_MODE*/);
    I2C_setDataCount(I2CA_BASE,dataCount);
    I2C_putData(I2CA_BASE, data_addr);//5. Data Address  //I2caRegs.I2CDXR.all = data_addr;
    for(i=0;i<dataCount;i++) I2C_putData(I2CA_BASE, *(wdata+i) );
    I2C_sendStartCondition(I2CA_BASE);//I2caRegs.I2CMDR.all = 0x6E20;       //6. START Send && CLK Send && TX Mode SET && I2C Enable
    while(I2C_isBusBusy(I2CA_BASE));//2. 0:Bus free. 1: Bus busy
    while(I2C_getStopConditionStatus(I2CA_BASE));//1. TX Data보내고 STOP 후 0으로 리셋됨
    I2C_sendStopCondition(I2CA_BASE); //I2caRegs.I2CMDR.all = 0x2C20;   //11. START Send && CLK Send && RX Mode SET && I2C Enable && STOP Send
    // NACKNOD 0 // FREE  0 // STT 1 //  reserved
    // STP 0  Stop Condition bit // MST 1 Master mode bit // TRX 1 transmitter mode bit // XA 0 expanded address
    // RM 0  repeat mode // DLB 0 Digital Loop Backmode // IRS 1 I2C module reset bit -- // STB 0 A Start Condition
    // FDF 0 Free data format // BC 0 0 0  8 BIT
}
uint8_t ds1338_read_time(struct rtctime_t *time)
{
	uint8_t buf[7];
	uint8_t res = i2c_read(0, buf, 7);
	if (res) return 2;

	time->second = decode_bcd(buf[0]);
	time->minute = decode_bcd(buf[1]);

	if (buf[2] & DS1338_HOUR_12)
	{
		time->hour = ((buf[2] >> 4) & 0x01) * 12 + ((buf[2] >> 5) & 0x01) * 12;
	}
	else
	{
		time->hour = decode_bcd(buf[2]);
	}

	time->day = decode_bcd(buf[4]);
	time->month = decode_bcd(buf[5] );
	time->year = 100 * ((buf[5] >> 7) & 0x01) + decode_bcd(buf[6]);

	return 0;
}
uint8_t ds1338_write_time(struct rtctime_t *time)
{
	uint8_t buf[8];

	buf[0] = encode_bcd(time->second);
	buf[1] = encode_bcd(time->minute);
	buf[2] = encode_bcd(time->hour); // Time always stored in 24-hour format
	buf[3] = 1;						 // Not used
	buf[4] = encode_bcd(time->day);
	buf[5] = encode_bcd(time->month);
	buf[6] = encode_bcd((time->year) % 100);
	i2c_write(0, buf, 8);
	return 0;
}
