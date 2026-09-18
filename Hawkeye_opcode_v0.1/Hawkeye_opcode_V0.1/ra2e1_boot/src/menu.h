/*
 * menu.h
 *
 *  Created on: 2 Oct 2019
 *      Author: b3800274
 */

#include "hal_data.h"
#include ".\comms\comms.h"
#include <stdio.h>

#ifndef MENU_H_
#define MENU_H_

void menu(void);
int  firmware_update_via_xmodem(void);

#endif /* MENU_H_ */
