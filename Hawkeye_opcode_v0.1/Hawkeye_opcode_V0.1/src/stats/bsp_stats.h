/**
 * @file  bsp_stats.h
 * @brief 运行统计模块 — 在RAM中实时累计，关机前写入Flash
 */

#ifndef STATS_BSP_STATS_H_
#define STATS_BSP_STATS_H_

#include "stdint.h"
#include "stdbool.h"
#include "flash/flash.h"
#include "hawkeye_config.h"

/* ======================================================================
 *  对外接口
 * ====================================================================== */

/**
 * @brief 初始化统计模块：从Flash加载历史数据到RAM累加器
 *        在 hal_entry() 初始化阶段调用，放在 flash_init() 之后
 */
void stats_init(void);

/**
 * @brief 1秒节拍任务：更新所有时长计数器
 *        由 hal_entry() 主循环中的 1000ms 任务槽调用
 *        需传入当前状态快照，避免重复读全局变量
 *
 * @param laser_on      三路激光当前开关状态 (gLaserOn[3] 的拷贝)
 * @param angle_mode    当前角度模式 (gAngleMode)
 * @param bat_state_low 当前电池低压指示 (始终 false, LOW 状态已移除)
 */
void stats_tick_1s(const uint8_t laser_on[3],
                   uint8_t       angle_mode,
                   bool          bat_state_low);

/**
 * @brief 记录某路激光被开启一次（按键按下时调用）
 * @param ch  激光通道索引 0/1/2
 */
void stats_laser_key_press(uint8_t ch);

/**
 * @brief 将RAM中的统计数据写入Flash（关机前调用）
 */
void stats_flush_to_flash(void);

/**
 * @brief 通过串口打印所有统计数据（AT+GET_STATS 指令调用）
 */
void stats_print_all(void);

/**
 * @brief 清空RAM和Flash中的统计数据（AT+CLEAR_STATS 指令调用，需先解锁）
 */
void stats_clear(void);
/* 供 flash_save_poweroff_and_stats 直接取指针，避免额外拷贝 */
const flash_stats_t *stats_get_ptr(void);
bool stats_tick_pending(void);   /* 返回并清除 1s 标志 */
#endif /* STATS_BSP_STATS_H_ */
