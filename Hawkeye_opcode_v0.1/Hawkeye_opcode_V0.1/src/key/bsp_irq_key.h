/*
 * bsp_key.h
 *
 *  Created on: Sep 18, 2024
 *      Author: AXQ0527A
 */

#ifndef __BSP_IRQ_KEY_H
#define __BSP_IRQ_KEY_H
#include <SysTick/bsp_SysTick.h>
#include "stdint.h"
#include "hal_data.h"
#include "debug_uart/bsp_debug_uart.h"
#include <agt/bsp_agt_timing.h>
#include "common_utils.h"
#include <led/bsp_led.h>
#include "stdbool.h"
#include "stdio.h"
/* ===== 扫描参数 ===== */


/* ===== 有效电平定义 =====
 * 如果按下时为低电平 → 设为 0
 * 如果按下时为高电平 → 设为 1
 */
/* 按键事件类型 */
typedef enum {
    KEY_NONE = 0,
    KEY_SHORT_PRESS,
    KEY_LONG_PRESS
} key_event_type_t;

/* 按键结构体 */
typedef struct {
    bsp_io_port_pin_t pin;    // IO 引脚
    uint8_t stable_level;     // 稳定电平
    uint8_t last_level;       // 上一次采样电平
    uint16_t debounce_time;   // 消抖计时
    uint16_t press_time;      // 按下持续时间
    uint8_t long_sent;        // 长按是否已触发
    uint8_t short_sent;       // 短按是否已触发 (消抖完成即触发)
    key_event_type_t event;   // 当前事件
    const char *name;         // 按键名字（可选用于调试）
} key_t;

/* 外部按键数组 */
extern key_t keys[];
extern const uint8_t key_count;
extern volatile uint8_t gLaserOn[3];
extern volatile uint8_t g_lock_led_override;   /* 锁定开关 HIGH 时 LED 强制关闭标志 */
extern volatile uint8_t g_led_show_all;        /* 重新锁定后 LED 全亮展示标志 */
void key_en_delay_task_1ms(void);
/* API */
void key_init(void);
void key_scan_task(void);
key_event_type_t key_get_event(uint8_t idx);
void key_handle_events(void);
void laser1_set(uint8_t on);
void laser2_set(uint8_t on);
void laser3_set(uint8_t on);
uint8_t laser_any_on(void);
void lock_sw_read(void);
void lock_sw_boot_check(void);
void key_restore_on_boot(void);
bool laser_serial_ctrl(uint8_t idx, bool on);
bool laser_serial_ctrl_all(bool on);
#endif /* KEY_BSP_KEY_H_ */
