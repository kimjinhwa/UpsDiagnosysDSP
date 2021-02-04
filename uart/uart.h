#include "driverlib.h"
#include "device.h"
#include "F2837xD_device.h"


void initSCICFIFO(void);
void sendData(void);
extern void UARTprintf(const uint16_t *pcString, ...);
extern void UARTvprintf(const uint16_t *pcString, va_list vaArgP);
