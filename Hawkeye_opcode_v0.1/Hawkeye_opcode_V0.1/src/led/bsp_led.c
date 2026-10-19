/*
 * bsp_led.c
 *
 *  Created on: Sep 18, 2024
 *      Author: AXQ0527A
 */

#include <led/bsp_led.h>
#include <agt/bsp_agt_timing.h>
#include "hawkeye_config.h"

void LED_Init(void)
{
    R_IOPORT_Open (&g_ioport_ctrl, g_ioport.p_cfg);
}


/* ===== LED 控制函数 ===== */
void led_set_pattern(led_pattern_t pattern)
{
    switch(pattern)
    {
        case LED_PATTERN_ALL_ON:
            LED1_ON;
            LED2_ON;
            LED3_ON;
            break;

        case LED_PATTERN_12_ON:
            LED1_ON;
            LED2_ON;
            LED3_OFF;
            break;

        case LED_PATTERN_1_ON:
            LED1_ON;
            LED2_OFF;
            LED3_OFF;
            break;

        case LED_PATTERN_BLINK:
        {
            static uint8_t blink_phase = 0;
            blink_phase ^= 1;
            if (blink_phase)
            {
                LED1_ON;
                LED2_ON;
                LED3_ON;
            }
            else
            {
                LED1_OFF;
                LED2_OFF;
                LED3_OFF;
            }
            break;
        }

        case LED_PATTERN_2_ON:
            LED1_OFF;
            LED2_ON;
            LED3_OFF;
            break;

        case LED_PATTERN_3_ON:
            LED1_OFF;
            LED2_OFF;
            LED3_ON;
            break;
        case LED_PATTERN_DEFAULT:
            LED1_OFF;
            LED2_OFF;
            LED3_OFF;
            break;

        default:
            LED1_OFF;
            LED2_OFF;
            LED3_OFF;
            break;
    }
}
