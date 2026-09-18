/*
 * bsp_key.h
 *
 *  Created on: Sep 18, 2024
 *      Author: AXQ0527A
 */

#ifndef __BSP_IRQ_KEY_H
#define __BSP_IRQ_KEY_H

#include "stdint.h"
#include "hal_data.h"

#define KEY1_SW2_PIN    BSP_IO_PORT_00_PIN_15

#define KEY_ON  1
#define KEY_OFF 0

void Key_Init(void);
uint32_t Key_Scan(bsp_io_port_pin_t key);
void Key_IRQ_Init(void);
void key_external_irq_callback(external_irq_callback_args_t *p_args);


#endif /* KEY_BSP_KEY_H_ */
