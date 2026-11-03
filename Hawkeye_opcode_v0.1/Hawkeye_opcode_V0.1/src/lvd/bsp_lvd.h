/*
 * bsp_lvd.h
 *
 *  异常断电（拔电池）掉电抢救模块 — 接口声明
 *
 *  Created on: 2026年11月3日
 *      Author: AT403
 */

/**
 * @file    bsp_lvd.h
 * @brief   掉电抢救：LVD1 监测 3.3V 轨，电压跌到阈值时触发 NMI，
 *          先砍激光/LED 负载，再写回统计（仅统计，不记关机原因）。
 *          适用于 RA2E1 + FSP（LVD1 走 NMI，无需改 vector_data）。
 */

#ifndef LVD_BSP_LVD_H_
#define LVD_BSP_LVD_H_

#include "stdint.h"

/**
 * @brief 初始化掉电抢救：配置 LVD1 阈值/中断模式 + 注册 NMI 回调。
 *        在 hal_entry() 初始化阶段调用，放在 flash_init() 完成之后
 *        （确保掉电回调里 flash_save_stats() 可用）。
 */
void lvd_powerloss_init(void);

#endif /* LVD_BSP_LVD_H_ */
