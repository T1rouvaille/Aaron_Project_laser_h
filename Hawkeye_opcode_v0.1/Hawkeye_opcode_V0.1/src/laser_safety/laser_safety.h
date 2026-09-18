/**
 * @file    laser_safety.h
 * @brief   Hawkeye 激光安全模块 — 带电插拔 / 激光管故障检测
 *
 * 从 bsp_agt_timing.c 的 laser_adjust_duty() 中抽出的安全子模块，
 * 负责"合法开启 vs 带电插入"以及"运行中带电拔出"的判断，
 * 触发时调用 system_power_off_request() 锁存异常关机。
 *
 * 通道索引 (laser_idx) 约定:
 *   0 = V2 (FRONT)
 *   1 = H  (SIDE)
 *   2 = V1 (HORIZ)
 *
 * @version 0.0.0.1
 * @date    2026-09-18
 */

#ifndef LASER_SAFETY_H_
#define LASER_SAFETY_H_

#include <stdint.h>
#include <stdbool.h>

/* ======================================================================
 *  安全评估结果
 * ====================================================================== */
typedef enum
{
    LASER_SAFETY_NORMAL = 0,    /* 正常, 继续调光 */
    LASER_SAFETY_SKIP,          /* PD 故障, 跳过本次调光 */
} laser_safety_result_t;

/* ======================================================================
 *  对外接口
 * ====================================================================== */

/**
 * @brief 合法开启时标记"建立期" (参数 glaser_idx = gLaserOn[] 索引, 0/1/2)。
 *        内部映射 gLaserOn 索引 → laser_idx: 0→1(H), 1→2(V1), 2→0(V2)。
 */
void laser_opening_set(uint8_t glaser_idx);

/**
 * @brief 复位某通道的"曾经接通"标志 (激光关闭时调用), 同时清除建立期和消抖计数。
 *        参数 glaser_idx = gLaserOn[] 索引 (0/1/2)。
 */
void laser_was_connected_reset(uint8_t glaser_idx);

/**
 * @brief 带电插拔 / 激光管故障检测, 每个调光周期调用一次。
 *
 * @param laser_idx   通道索引 (0=V2, 1=H, 2=V1)
 * @param avg_pd      本周期 PD 均值 (mV)
 * @param current_mA  本周期电流 (mA)
 * @return LASER_SAFETY_NORMAL 正常, 继续调光;
 *         LASER_SAFETY_SKIP   PD 过低(故障/拔出), 跳过本次调光。
 *
 * 检测逻辑:
 *   - 带电插入: 电流从无到有(从未接通→当前>=阈值), 且不在合法开启建立期内 → 关机。
 *   - 带电拔出: PD≈0 且电流≈0(断路), 且此前电流曾正常流过 → 连续确认后关机。
 *   - 开机未接激光管: 电流从未流过 → 不误关机。
 */
laser_safety_result_t laser_safety_hotplug_check(int laser_idx,
                                                 int avg_pd,
                                                 int current_mA);

#endif /* LASER_SAFETY_H_ */
