/*
 * usb_msc.h
 *
 *  Created on: Feb 15, 2018
 *      Author: khockuba
 */

#ifndef SRC_MAIN_DRIVERS_USB_MSC_H_
#define SRC_MAIN_DRIVERS_USB_MSC_H_

#include "platform.h"

uint8_t startMsc(void);
void mscButtonInit(void);
void mscCheck(void);
bool mscButton(void);

#endif /* SRC_MAIN_DRIVERS_USB_MSC_H_ */
