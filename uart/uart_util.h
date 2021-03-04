#include "driverlib.h"
#include "device.h"
#include "F2837xD_device.h"

#ifndef PATH_BUF_SIZE
#define PATH_BUF_SIZE   80     // Defines the size of the buffers that hold the
                               // path, or temporary data from the SD card.
                               // There are two buffers allocated of this size.
                               // The buffer size must be large enough to hold
                               // the longest expected full path name,
                               // including the file name, and a trailing null
                               // character.
#endif

extern void UARTprintf(uint32_t base,const uint16_t *pcString, ...);
#ifndef GPIO_0_B485_EN
#define GPIO_0_B485_EN     0
#endif
