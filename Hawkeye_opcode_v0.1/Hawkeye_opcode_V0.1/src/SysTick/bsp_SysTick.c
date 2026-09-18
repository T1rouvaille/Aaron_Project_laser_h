/*
 * bsp_SysTick.c
 *
 *  Created on: Sep 23, 2024
 *      Author: AXQ0527A
 */

#include <SysTick/bsp_SysTick.h>

static __IO uint32_t IT_nums;
static uint32_t IT_Period;

void SysTick_Init(uint32_t IT_frequency)
{
    IT_Period = SystemCoreClock / IT_frequency;
    uint32_t err = SysTick_Config (IT_Period);
    assert(err==0);
}

void SysTick_Delay(uint32_t delay, sys_delay_units_t unit)
{
    uint32_t SumTime = delay * unit;
    IT_nums = SumTime/IT_Period;
    while (IT_nums != 0);
}

extern void SysTick_Handler(void);

void SysTick_Handler(void)
{
    if (IT_nums != 0x00)
    {
        IT_nums--;
    }
}
