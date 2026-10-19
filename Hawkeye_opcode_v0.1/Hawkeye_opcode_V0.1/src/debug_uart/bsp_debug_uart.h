/*
 * bsp_debug_uart.h
 *
 *  Created on: Sep 18, 2024
 *      Author: AXQ0527A
 */

#ifndef _BSP_DEBUG_UART_H_
#define _BSP_DEBUG_UART_H_
#include "stdio.h"
#include "hal_data.h"
#include <stdbool.h>
#include <iic/iic.h>
#include "flash/flash.h"
#include "hawkeye_config.h"

extern volatile int reference_voltage;

void Debug_UART9_Init(void);
void debug_uart9_callback (uart_callback_args_t * p_args);

void uart9_send_blocking(const char *msg);
void Debug_UART9_SendBlocking(const uint8_t *data, uint32_t length);
void Debug_UART9_ProcessReceivedData(void);

#endif /* DEBUG_UART_BSP_DEBUG_UART_H_ */
