/*
 * bsp_led.c
 *
 *  Created on: Sep 18, 2024
 *      Author: AXQ0527A
 */

#include <led/bsp_led.h>
void LED_Init(void)
{
    R_IOPORT_Open (&g_ioport_ctrl, g_ioport.p_cfg);
}
