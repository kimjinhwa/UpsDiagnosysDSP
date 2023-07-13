/*
 * CmdLineExtend.c
 *
 *  Created on: 2023. 7. 13.
 *      Author: STELLA
 */
#include "cmdLineExtend.h"
#include "version.h"


#pragma CODE_SECTION(myFunction,".TI.ramfunc")
void myFunction()
{
    SCIprintf("Test");
}
//
////******************************************************************************
////
////! Host interrupt service routine wrapper to make ISR compatible with
////! C2000 PIE controller.
////
////******************************************************************************
#define PATH_BUF_SIZE   80
#define CMD_BUF_SIZE    64
static char g_cCwdBuf[PATH_BUF_SIZE] = "/";
static char g_cTmpBuf[PATH_BUF_SIZE];
static char g_cCmdBuf[CMD_BUF_SIZE];
__interrupt void INT_myUSB0_ISR(void)
{
    USB0HostIntHandler();
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

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
tCmdLineEntry g_psCmdTable[] =
{
    { "help",   Cmd_help,      " : Display list of commands" },
    { "h",      Cmd_help,   "    : alias for help" },
    { "?",      Cmd_help,   "    : alias for help" },
    { "ls",     Cmd_ls,      "   : Display list of files" },
    { "chdir",  Cmd_cd,         ": Change directory" },
    { "cd",     Cmd_cd,      "   : alias for chdir" },
    { "pwd",    Cmd_pwd,      "  : Show current working directory" },
    { "cat",    Cmd_cat,      "  : Show contents of a text file" },
    { "ver",    Cmd_ver,      "  : Show Firmware Version" },
    { 0, 0, 0 }
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

int Cmd_ver(int argc, char *argv[])
{
    SCIprintf("\nFirmware Version \n");
    SCIprintf(version);
    SCIprintf("\n");

    return(0);
}
int Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Print some header text.
    //
    SCIprintf("\nAvailable commands\n");
    SCIprintf("------------------\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_psCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        SCIprintf("%s%s\n", pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    //
    // Return success.
    //
    return(0);
}
int Cmd_cat(int argc, char *argv[])
{
    FRESULT fresult;
    unsigned short usBytesRead;

    //
    // Do not attempt to do anything if there is not a drive attached.
    //
    if(g_eState != STATE_DEVICE_READY)
    {
        return(FR_NOT_READY);
    }

    //
    // First, check to make sure that the current path (CWD), plus the file
    // name, plus a separator and trailing null, will all fit in the temporary
    // buffer that will be used to hold the file name.  The file name must be
    // fully specified, with path, to FatFs.
    //
    if(strlen(g_cCwdBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_cTmpBuf))
    {
        SCIprintf("Resulting path name is too long\n");
        return(0);
    }

    //
    // Copy the current path to the temporary buffer so it can be manipulated.
    //
    strcpy(g_cTmpBuf, g_cCwdBuf);

    //
    // If not already at the root level, then append a separator.
    //
    if(strcmp("/", g_cCwdBuf))
    {
        strcat(g_cTmpBuf, "/");
    }

    //
    // Now finally, append the file name to result in a fully specified file.
    //
    strcat(g_cTmpBuf, argv[1]);

    //
    // Open the file for reading.
    //
    fresult = f_open(&g_sFileObject, g_cTmpBuf, FA_READ);

    //
    // If there was some problem opening the file, then return an error.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    //
    // Enter a loop to repeatedly read data from the file and display it, until
    // the end of the file is reached.
    //
    do
    {
        //
        // Read a block of data from the file.  Read as much as can fit in the
        // temporary buffer, including a space for the trailing null.
        //
        fresult = f_read(&g_sFileObject, g_cTmpBuf, sizeof(g_cTmpBuf) - 1,
                         &usBytesRead);

        //
        // If there was an error reading, then print a newline and return the
        // error to the user.
        //
        if(fresult != FR_OK)
        {
            SCIprintf("\n");
            return(fresult);
        }

        //
        // Null terminate the last block that was read to make it a null
        // terminated string that can be used with printf.
        //
        g_cTmpBuf[usBytesRead] = 0;

        //
        // Print the last chunk of the file that was received.
        //
        SCIprintf("%s", g_cTmpBuf);

        //
        // Continue reading until less than the full number of bytes are read.
        // That means the end of the buffer was reached.
        //
    }
    while(usBytesRead == sizeof(g_cTmpBuf) - 1);

    //
    // Return success.
    //
    return(0);
}
int Cmd_cd(int argc, char *argv[])
{
    unsigned int uIdx;
    FRESULT fresult;

    //
    // Do not attempt to do anything if there is not a drive attached.
    //
    if(g_eState != STATE_DEVICE_READY)
    {
        return(FR_NOT_READY);
    }

    //
    // Copy the current working path into a temporary buffer so it can be
    // manipulated.
    //
    strcpy(g_cTmpBuf, g_cCwdBuf);

    //
    // If the first character is /, then this is a fully specified path, and it
    // should just be used as-is.
    //
    if(argv[1][0] == '/')
    {
        //
        // Make sure the new path is not bigger than the cwd buffer.
        //
        if(strlen(argv[1]) + 1 > sizeof(g_cCwdBuf))
        {
            SCIprintf("Resulting path name is too long\n");
            return(0);
        }

        //
        // If the new path name (in argv[1])  is not too long, then copy it
        // into the temporary buffer so it can be checked.
        //
        else
        {
            strncpy(g_cTmpBuf, argv[1], sizeof(g_cTmpBuf));
        }
    }

    //
    // If the argument is .. then attempt to remove the lowest level on the
    // CWD.
    //
    else if(!strcmp(argv[1], ".."))
    {
        //
        // Get the index to the last character in the current path.
        //
        uIdx = strlen(g_cTmpBuf) - 1;

        //
        // Back up from the end of the path name until a separator (/) is
        // found, or until we bump up to the start of the path.
        //
        while((g_cTmpBuf[uIdx] != '/') && (uIdx > 1))
        {
            //
            // Back up one character.
            //
            uIdx--;
        }

        //
        // Now we are either at the lowest level separator in the current path,
        // or at the beginning of the string (root).  So set the new end of
        // string here, effectively removing that last part of the path.
        //
        g_cTmpBuf[uIdx] = 0;
    }

    //
    // Otherwise this is just a normal path name from the current directory,
    // and it needs to be appended to the current path.
    //
    else
    {
        //
        // Test to make sure that when the new additional path is added on to
        // the current path, there is room in the buffer for the full new path.
        // It needs to include a new separator, and a trailing null character.
        //
        if(strlen(g_cTmpBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_cCwdBuf))
        {
            SCIprintf("Resulting path name is too long\n");
            return(0);
        }

        //
        // The new path is okay, so add the separator and then append the new
        // directory to the path.
        //
        else
        {
            //
            // If not already at the root level, then append a /
            //
            if(strcmp(g_cTmpBuf, "/"))
            {
                strcat(g_cTmpBuf, "/");
            }

            //
            // Append the new directory to the path.
            //
            strcat(g_cTmpBuf, argv[1]);
        }
    }

    //
    // At this point, a candidate new directory path is in chTmpBuf.  Try to
    // open it to make sure it is valid.
    //
    fresult = f_opendir(&g_sDirObject, g_cTmpBuf);

    //
    // If it can't be opened, then it is a bad path.  Inform user and return.
    //
    if(fresult != FR_OK)
    {
        SCIprintf("cd: %s\n", g_cTmpBuf);
        return(fresult);
    }

    //
    // Otherwise, it is a valid new path, so copy it into the CWD.
    //
    else
    {
        strncpy(g_cCwdBuf, g_cTmpBuf, sizeof(g_cCwdBuf));
    }

    //
    // Return success.
    //
    return(0);
}
int Cmd_ls(int argc, char *argv[])
{
    uint32_t ui32TotalSize;
    uint32_t ui32FileCount;
    uint32_t ui32DirCount;
    FRESULT fresult;
    FATFS *pFatFs;

    //
    // Do not attempt to do anything if there is not a drive attached.
    //
    if(g_eState != STATE_DEVICE_READY)
    {
        return(FR_NOT_READY);
    }

    //
    // Open the current directory for access.
    //
    fresult = f_opendir(&g_sDirObject, g_cCwdBuf);

    //
    // Check for error and return if there is a problem.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    ui32TotalSize = 0;
    ui32FileCount = 0;
    ui32DirCount = 0;

    //
    // Enter loop to enumerate through all directory entries.
    //
    while(1)
    {
        //
        // Read an entry from the directory.
        //
        fresult = f_readdir(&g_sDirObject, &g_sFileInfo);

        //
        // Check for error and return if there is a problem.
        //
        if(fresult != FR_OK)
        {
            return(fresult);
        }

        //
        // If the file name is blank, then this is the end of the listing.
        //
        if(!g_sFileInfo.fname[0])
        {
            break;
        }

        //
        // If the attribute is directory, then increment the directory count.
        //
        if(g_sFileInfo.fattrib & AM_DIR)
        {
            ui32DirCount++;
        }

        //
        // Otherwise, it is a file.  Increment the file count, and add in the
        // file size to the total.
        //
        else
        {
            ui32FileCount++;
            ui32TotalSize += g_sFileInfo.fsize;
        }

        //
        // Print the entry information on a single line with formatting to show
        // the attributes, date, time, size, and name.
        //
        SCIprintf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9u  %s\n",
                 (g_sFileInfo.fattrib & AM_DIR) ? (uint32_t)'D' : (uint32_t)'-',
                 (g_sFileInfo.fattrib & AM_RDO) ? (uint32_t)'R' : (uint32_t)'-',
                 (g_sFileInfo.fattrib & AM_HID) ? (uint32_t)'H' : (uint32_t)'-',
                 (g_sFileInfo.fattrib & AM_SYS) ? (uint32_t)'S' : (uint32_t)'-',
                 (g_sFileInfo.fattrib & AM_ARC) ? (uint32_t)'A' : (uint32_t)'-',
                 (uint32_t)((g_sFileInfo.fdate >> 9) + 1980),
                 (uint32_t)((g_sFileInfo.fdate >> 5) & 15),
                 (uint32_t)(g_sFileInfo.fdate & 31),
                 (uint32_t)((g_sFileInfo.ftime >> 11)),
                 (uint32_t)((g_sFileInfo.ftime >> 5) & 63),
                 (uint32_t)(g_sFileInfo.fsize),
                 g_sFileInfo.fname);
    }

    //
    // Print summary lines showing the file, dir, and size totals.
    //
    SCIprintf("\n%4u File(s),%10u bytes total\n%4u Dir(s)",
               ui32FileCount, ui32TotalSize, ui32DirCount);

    //
    // Get the free space.
    //
    fresult = f_getfree("/", &ui32TotalSize, &pFatFs);

    //
    // Check for error and return if there is a problem.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    //
    // Display the amount of free space that was calculated.
    //
    SCIprintf(", %10uK bytes free\n", ui32TotalSize * pFatFs->sects_clust / 2);

    //
    // Made it to here, return with no errors.
    //
    return(0);
}
int Cmd_pwd(int argc, char *argv[])
{
    //
    // Do not attempt to do anything if there is not a drive attached.
    //
    if(g_eState != STATE_DEVICE_READY)
    {
        return(FR_NOT_READY);
    }

    //
    // Print the CWD to the console.
    //
    SCIprintf("%s\n", g_cCwdBuf);

    //
    // Return success.
    //
    return(0);
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
void ReadLine(void)
{
    uint32_t ulIdx, ulPrompt;
    uint8_t ui8Char;
    tState eStateCopy;

    //
    // Start reading at the beginning of the command buffer and print a prompt.
    //
    g_cCmdBuf[0] = '\0';
    ulIdx = 0;
    ulPrompt = 1;

    //
    // Loop forever.  This loop will be explicitly broken out of when the line
    // has been fully read.
    //
    while(1)
    {

        //
        // See if a mass storage device has been enumerated.
        //
        if(g_eState == STATE_DEVICE_ENUM)
        {
            //
            // Take it easy on the Mass storage device if it is slow to
            // start up after connecting.
            //
            if(USBHMSCDriveReady(g_psMSCInstance) != 0)
            {
                //
                // Wait about 100ms before attempting to check if the
                // device is ready again.
                //
                SysCtl_delay(SysCtl_getClock(DEVICE_OSCSRC_FREQ)/30);

                break;
            }

            //
            // Reset the working directory to the root.
            //
            g_cCwdBuf[0] = '/';
            g_cCwdBuf[1] = '\0';

            //
            // Attempt to open the directory.  Some drives take longer to
            // start up than others, and this may fail (even though the USB
            // device has enumerated) if it is still initializing.
            //
            f_mount(0, &g_sFatFs);
            if(f_opendir(&g_sDirObject, g_cCwdBuf) == FR_OK)
            {
                //
                // The drive is fully ready, so move to that state.
                //
                g_eState = STATE_DEVICE_READY;
            }
        }

        //
        // See if the state has changed.  We make a copy of g_eUIState to
        // prevent a compiler warning about undefined order of volatile
        // accesses.
        //
        eStateCopy = g_eUIState;
        if(g_eState != eStateCopy)
        {
            //
            // Determine the new state.
            //
            switch(g_eState)
            {
                //
                // A previously connected device has been disconnected.
                //
            case STATE_NO_DEVICE:
            {
                if(g_eUIState == STATE_UNKNOWN_DEVICE)
                {
                    SCIprintf("\nUnknown device disconnected.\n");
                }
                else
                {
                    SCIprintf("\nMass storage device disconnected.\n");
                }
                ulPrompt = 1;
                break;
            }

            //
            // A mass storage device is being enumerated.
            //
            case STATE_DEVICE_ENUM:
            {
                break;
            }

            //
            // A mass storage device has been enumerated and initialized.
            //
            case STATE_DEVICE_READY:
            {
                SCIprintf("\nMass storage device connected.\n");
                ulPrompt = 1;
                break;
            }

            //
            // An unknown device has been connected.
            //
            case STATE_UNKNOWN_DEVICE:
            {
                SCIprintf("\nUnknown device connected.\n");
                ulPrompt = 1;
                break;
            }

            //
            // A power fault has occurred.
            //
            case STATE_POWER_FAULT:
            {
                SCIprintf("\nPower fault.\n");
                ulPrompt = 1;
                break;
            }
            }

            //
            // Save the current state.
            //
            g_eUIState = g_eState;
        }

        //
        // Print a prompt if necessary.
        //
        if(ulPrompt)
        {
            //
            // Print the prompt based on the current state.
            //
            if(g_eState == STATE_DEVICE_READY)
            {
                SCIprintf("%s> %s", g_cCwdBuf, g_cCmdBuf);
            }
            else if(g_eState == STATE_UNKNOWN_DEVICE)
            {
                SCIprintf("UNKNOWN> %s", g_cCmdBuf);
            }
            else
            {
                SCIprintf("NODEV> %s", g_cCmdBuf);
            }

            //
            // The prompt no longer needs to be printed.
            //
            ulPrompt = 0;
        }

        //
        // Loop while there are characters that have been received from the
        // SCI.
        //
        while(SCI_isDataAvailableNonFIFO(SCIC_BASE))
        {
            //
            // Read the next character from the SCI.
            //
            ui8Char = SCI_readCharBlockingNonFIFO(SCIC_BASE);

            //
            // See if this character is a backspace and there is at least one
            // character in the input line.
            //
            if((ui8Char == '\b') && (ulIdx != 0))
            {
                //
                // Erase the last character from the input line.
                //
                SCIprintf("\b \b");
                ulIdx--;
                g_cCmdBuf[ulIdx] = '\0';
            }

            //
            // See if this character is a newline.
            //
            else if((ui8Char == '\r') || (ui8Char == '\n'))
            {
                //
                // Return to the caller.
                //
                SCIprintf("\n");
                return;
            }

            //
            // See if this character is an escape or Ctrl-U.
            //
            else if((ui8Char == 0x1b) || (ui8Char == 0x15))
            {
                //
                // Erase all characters in the input buffer.
                //
                while(ulIdx)
                {
                    SCIprintf("\b \b");
                    ulIdx--;
                }
                g_cCmdBuf[0] = '\0';
            }

            //
            // See if this is a printable ASCII character.
            //
            else if((ui8Char >= ' ') && (ui8Char <= '~') &&
                    (ulIdx < (sizeof(g_cCmdBuf) - 1)))
            {
                //
                // Add this character to the input buffer.
                //
                g_cCmdBuf[ulIdx++] = ui8Char;
                g_cCmdBuf[ulIdx] = '\0';
                SCIprintf("%c", (uint32_t)ui8Char);
            }
        }

        //
        // Run the main routine of the Host controller driver.
        //
        USBHCDMain();
    }
}
const char * StringFromFresult(FRESULT fresult)
{
    uint16_t ui16dx;

    //
    // Enter a loop to search the error code table for a matching error code.
    //
    for(ui16dx = 0; ui16dx < NUM_FRESULT_CODES; ui16dx++)
    {
        //
        // If a match is found, then return the string name of the error code.
        //
        if(g_sFresultStrings[ui16dx].fresult == fresult)
        {
            return(g_sFresultStrings[ui16dx].pcResultStr);
        }
    }

    //
    // At this point no matching code was found, so return a string indicating
    // unknown error.
    //
    return("UNKNOWN ERROR CODE");
}
void ReadLineEx(void){
    int iStatus;
    //while(1)
    {
        //
        // Get a line of text from the user.
        //
        ReadLine();
        if(g_cCmdBuf[0] == '\0')
        {
            //continue;
            return;
        }

        //
        // Pass the line from the user to the command processor.
        // It will be parsed and valid commands executed.
        //
        iStatus = CmdLineProcess(g_cCmdBuf);

        //
        // Handle the case of bad command.
        //
        if(iStatus == CMDLINE_BAD_CMD)
        {
            SCIprintf("Bad command!\n");
        }

        //
        // Handle the case of too many arguments.
        //
        else if(iStatus == CMDLINE_TOO_MANY_ARGS)
        {
            SCIprintf("Too many arguments for command processor!\n");
        }

        //
        // Otherwise the command was executed.  Print the error
        // code if one was returned.
        //
        else if(iStatus != 0)
        {
            SCIprintf("Command returned error code %s\n",
                       StringFromFresult((FRESULT)iStatus));
        }
    }
}
void initUsbDriver(){
    g_eState = STATE_NO_DEVICE;
    g_eUIState = STATE_NO_DEVICE;
    USBHCDRegisterDrivers(0, g_ppHostClassDrivers, NUM_CLASS_DRIVERS);
    g_psMSCInstance = USBHMSCDriveOpen(0, (tUSBHMSCCallback)MSCCallback);
    f_mount(0, &g_sFatFs);

}
/////////  END  USB  //
