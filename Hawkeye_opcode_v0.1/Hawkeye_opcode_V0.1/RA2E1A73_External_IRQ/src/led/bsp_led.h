/*
 * bsp_led.h
 *
 *  Created on: Sep 18, 2024
 *      Author: AXQ0527A
 */



#ifndef __BSP_LED_H
#define __BSP_LED_H
#include "hal_data.h"

#define LED1_ON R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_10,BSP_IO_LEVEL_LOW)
#define LED2_ON R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_12,BSP_IO_LEVEL_LOW)


#define LED1_OFF R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_10,BSP_IO_LEVEL_HIGH)
#define LED2_OFF R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_12,BSP_IO_LEVEL_HIGH)

#define LED1_TOGGLE R_PORT1->PODR ^= 1<<(BSP_IO_PORT_01_PIN_10 & 0xFF)
#define LED2_TOGGLE R_PORT1->PODR ^= 1<<(BSP_IO_PORT_01_PIN_12 & 0xFF)


void LED_Init(void);

#endif /* LED_BSP_LED_H_ */
