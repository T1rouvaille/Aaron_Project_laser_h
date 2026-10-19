/*
 * bsp_SysTick.h
 *
 *  Created on: Sep 23, 2024
 *      Author: AXQ0527A
 */

#ifndef _BSP_SYSTICK_H_
#define _BSP_SYSTICK_H_
#include "hal_data.h"
#include "stdint.h"
typedef enum
{
    SYS_DELAY_UNITS_SECONDS = 200000000, ///< Requested delay amount

    SYS_DELAY_UNITS_MILLISECONDS = 200000, ///< Requested delay amount

    SYS_DELAY_UNITS_MICROSECONDS = 200 ///< Requested delay amount

} sys_delay_units_t;
void SysTick_Init(uint32_t IT_frequency);
void SysTick_Delay(uint32_t delay, sys_delay_units_t unit);
#endif /* SYSTICK_BSP_SYSTICK_H_ */
