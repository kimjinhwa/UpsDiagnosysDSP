#ifndef CMDLINEEXTEND_H_
#define CMDLINEEXTEND_H_

#include <string.h>
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
void MSCCallback(tUSBHMSCInstance *psMSCInstance, uint32_t ui32Event, void *pvEventData);
void ReadLine(void);
const char * StringFromFresult(FRESULT fresult);

int Cmd_help(int argc, char *argv[]);
int Cmd_cat(int argc, char *argv[]);
int Cmd_cd(int argc, char *argv[]);
int Cmd_ls(int argc, char *argv[]);
int Cmd_pwd(int argc, char *argv[]);
int Cmd_ver(int argc, char *argv[]);

void MSCCallback(tUSBHMSCInstance *psMSCInstance, uint32_t ui32Event, void *pvEventData);
void ReadLine(void);
void ReadLineEx(void);
void initUsbDriver();

#endif /* CMDLINEEXTEND_H_ */
