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
 * @param ntc_adc       控制板 NTC ADC 原始值（用于温度分区统计）
 */
void stats_tick_1s(const uint8_t laser_on[3],
                   uint8_t       angle_mode,
                   bool          bat_state_low,
                   uint16_t      ntc_adc);

/**
 * @brief 记录某路激光被开启一次（按键按下时调用）
 * @param ch  激光通道索引 0/1/2
 */
void stats_laser_key_press(uint8_t ch);

/* ======================================================================
 *  temp 段温度分区计数（开机/运行中/关机, 带回差, 单位=次数）
 * ====================================================================== */

/**
 * @brief 开机检测一次: 按当前 NTC 温度判定初始分区并计数, 同时初始化状态机
 * @param ntc_adc  控制板 NTC ADC 原始值
 */
void stats_temp_band_init(uint16_t ntc_adc);

/**
 * @brief 运行中每秒调用: 带回差跟踪温度分区切换, 进入新分区时计数 +1
 * @param ntc_adc  控制板 NTC ADC 原始值
 */
void stats_temp_band_track(uint16_t ntc_adc);

/**
 * @brief 关机检测一次: 按当前 NTC 温度判定分区并计数
 * @param ntc_adc  控制板 NTC ADC 原始值
 */
void stats_temp_band_shutdown(uint16_t ntc_adc);

/* ======================================================================
 *  Protection 段检测计数（运行时累积, 关机前随 stats 一起落盘）
 * ====================================================================== */
typedef enum
{
    STATS_PROT_NTC_HOT_LASER = 0,   /* NTC Hot Laser    @24 (控制板 NTC 高温) */
    STATS_PROT_NTC_COLD_LASER,      /* NTC Cold Laser   @26 (控制板 NTC 低温) */
    STATS_PROT_NTC_FROZEN_PACK,     /* NTC Frozen Pack  @28 (电池包 NTC 冻结) */
    STATS_PROT_H1_OVERCUR,          /* H1 Over Current  @40 */
    STATS_PROT_V1_OVERCUR,          /* V1 Over Current  @44 */
    STATS_PROT_V2_OVERCUR,          /* V2 Over Current  @46 */
} stats_prot_event_t;

/**
 * @brief 记录一次保护检测事件（边沿触发, 由检测方在事件发生瞬间调用一次）
 * @param ev  事件类型
 */
void stats_protection_event(stats_prot_event_t ev);

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
