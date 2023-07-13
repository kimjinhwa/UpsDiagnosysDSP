#ifndef CMDLINEEXTEND_H_
#define CMDLINEEXTEND_H_
#include "cmdline.h"
#include "usb_hal.h"
#include "usblib.h"
#include "usbmsc.h"
#include "host/usbhost.h"
#include "host/usbhmsc.h"
#include "scistdio.h"
#include "fatfs/src/ff.h"

void myFunction();
__interrupt void INT_myUSB0_ISR(void);

#endif /* CMDLINEEXTEND_H_ */
