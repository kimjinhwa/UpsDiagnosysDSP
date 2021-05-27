/*
 * cpuFlashMemory.h
 *
 *  Created on: May 4, 2021
 *      Author: STELLA
 */

#ifndef CPUFLASHMEMORY_H_
#define CPUFLASHMEMORY_H_

#include "driverlib.h"
#include "device.h"
#include "F2837xD_device.h"

#include "F021_F2837xD_C28x.h"
#define userFlashStart         0xBE000
#define userFlashLenght         0x100

void CallFlashAPI(uint32_t flashAddress, uint16_t *Buffer,uint16_t len);
#endif /* CPUFLASHMEMORY_H_ */
