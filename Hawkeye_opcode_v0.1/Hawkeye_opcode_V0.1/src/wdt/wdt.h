/*
 * wdt.h
 *
 *  Created on: 2026年3月3日
 *      Author: AT403
 */

#ifndef WDT_WDT_H_
#define WDT_WDT_H_
#include "hal_data.h"
#include "debug_uart/bsp_debug_uart.h"
#include "hawkeye_config.h"
/* -----------------------------------------------------------------------
 * 公共 API
 * ----------------------------------------------------------------------- */

/**
 * @brief  初始化并启动 WDT，同时通过串口打印上次复位原因
 */
void wdt_init(void);

/**
 * @brief  喂狗，在主循环 gSystemTickFlag 分支入口每 1ms 调用一次
 */
void wdt_feed(void);

/**
 * @brief  主动触发软件复位（打印原因后立即复位）
 * @param  reason  错误描述字符串，可为 NULL
 */
void wdt_force_reset(const char *reason);


#endif /* WDT_WDT_H_ */
