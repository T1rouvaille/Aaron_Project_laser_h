/*
 * bsp_led.h
 *
 *  Created on: Sep 18, 2024
 *      Author: AXQ0527A
 */



#ifndef __BSP_LED_H
#define __BSP_LED_H
#include "hal_data.h"

/* LED OFF (低电平有效) */
#define LED1_ON    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_02_PIN_06, BSP_IO_LEVEL_LOW)
#define LED2_ON    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_02_PIN_07, BSP_IO_LEVEL_LOW)
#define LED3_ON    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_09_PIN_15, BSP_IO_LEVEL_LOW)
#define EN1_OFF    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_09_PIN_13, BSP_IO_LEVEL_LOW)
#define Power_En_OFF    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_08, BSP_IO_LEVEL_LOW)
#define Batt_OFF    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_09, BSP_IO_LEVEL_LOW)

/* LED ON (高电平有效) */
#define LED1_OFF   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_02_PIN_06, BSP_IO_LEVEL_HIGH)
#define LED2_OFF   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_02_PIN_07, BSP_IO_LEVEL_HIGH)
#define LED3_OFF   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_09_PIN_15, BSP_IO_LEVEL_HIGH)
#define EN1_ON   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_09_PIN_13, BSP_IO_LEVEL_HIGH)
#define Power_En_ON   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_08, BSP_IO_LEVEL_HIGH)
#define Batt_ON   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_09, BSP_IO_LEVEL_HIGH)

/* LED TOGGLE (寄存器直接翻转) */
#define LED1_TOGGLE    R_PORT2->PODR ^= 1 << (BSP_IO_PORT_02_PIN_06 & 0xFF)
#define LED2_TOGGLE    R_PORT2->PODR ^= 1 << (BSP_IO_PORT_02_PIN_07 & 0xFF)
#define LED3_TOGGLE    R_PORT9->PODR ^= 1 << (BSP_IO_PORT_09_PIN_15 & 0xFF)

void LED_Init(void);
/* ===== LED 状态枚举 ===== */
typedef enum {
    LED_PATTERN_ALL_ON = 0,   // 123全开
    LED_PATTERN_12_ON,        // 12开，3关
    LED_PATTERN_1_ON,         // 1开，23关
    LED_PATTERN_BLINK,         // 全闪
    LED_PATTERN_2_ON,
    LED_PATTERN_3_ON,
    LED_PATTERN_DEFAULT
} led_pattern_t;

/* ===== LED 控制函数声明 ===== */
void led_set_pattern(led_pattern_t pattern);
#endif /* LED_BSP_LED_H_ */
