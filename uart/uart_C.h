#include "driverlib.h"
#include "device.h"
#include "F2837xD_device.h"


void initSCICFIFO(void);
void initSCICFIFO(void){

    GPIO_setMasterCore(28, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_90_SCIRXDC);
    GPIO_setDirectionMode(90, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(90, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(90, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(89, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_89_SCITXDC);
    GPIO_setDirectionMode(89, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(89, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(89, GPIO_QUAL_ASYNC);
}

