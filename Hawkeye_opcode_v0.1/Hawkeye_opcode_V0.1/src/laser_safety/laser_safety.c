/**
 * @file    laser_safety.c
 * @brief   Hawkeye 激光安全模块实现 — 带电插拔 / 激光管故障检测
 *
 * 状态索引均为 laser_idx: 0=V2(FRONT), 1=H(SIDE), 2=V1(HORIZ)。
 * 逻辑与拆分前 bsp_agt_timing.c 的 laser_adjust_duty() 完全等价。
 *
 * @version 0.0.0.1
 * @date    2026-09-18
 */
#include "laser_safety.h"
#include "hawkeye_config.h"
#include <adc/bsp_adc.h>   /* system_power_off_request(), power_off_reason_t */

/* 带电插拔消抖计数：连续 LASER_HOTPLUG_CONFIRM_COUNT 次 PD≈0 且电流过低才判插拔 */
static uint8_t hotplug_confirm_count[3] = {0, 0, 0};

/* "曾经接通"标志: 只要电流曾经正常流过(>= LASER_HOTPLUG_CURRENT_MA)即置位,
 * 用于区分"开机未接激光管(电流从未流过)"与"运行中被拔掉(电流从有到无)"。
 * 注意: 不要求 PD 正常, 因为 PD 采样异常(激光管故障)时电流仍可正常。 */
static bool laser_was_connected[3] = {false, false, false};

/* "合法开启建立期"计数: 合法开启命令后前 N 个调光周期内的电流建立视为合法,
 * 用于区分"合法开启导致的电流建立"与"带电插入导致的电流建立"。
 * 每次激光调光周期调用递减 1, 减到 0 后若才检测到电流建立 → 判带电插。 */
static uint8_t laser_opening_cnt[3] = {0, 0, 0};

/* 合法开启建立期覆盖的调光周期数 (约 N × 采样周期) */
#define LASER_OPEN_ESTABLISH_CYCLES  (3U)

void laser_opening_set(uint8_t glaser_idx)
{
    if (glaser_idx < 3)
    {
        int laser_idx = (glaser_idx + 1) % 3;
        laser_opening_cnt[laser_idx]     = LASER_OPEN_ESTABLISH_CYCLES;
        laser_was_connected[laser_idx]   = false;
        hotplug_confirm_count[laser_idx] = 0;
    }
}

void laser_was_connected_reset(uint8_t glaser_idx)
{
    if (glaser_idx < 3)
    {
        int laser_idx = (glaser_idx + 1) % 3;
        laser_was_connected[laser_idx]   = false;
        laser_opening_cnt[laser_idx]     = 0;
        hotplug_confirm_count[laser_idx] = 0;
    }
}

laser_safety_result_t laser_safety_hotplug_check(int laser_idx,
                                                 int avg_pd,
                                                 int current_mA)
{
    /* 合法开启建立期递减: 每次调光周期递减 1 */
    if (laser_opening_cnt[laser_idx] > 0)
        laser_opening_cnt[laser_idx]--;

    /* ===== 电流建立边沿检测 (区分合法开启 vs 带电插入) =====
     * 电流从无(从未接通)到有(当前 >= LASER_HOTPLUG_CURRENT_MA),
     * 且不在合法开启建立期内 → 带电插入关机 */
    if (current_mA >= LASER_HOTPLUG_CURRENT_MA && !laser_was_connected[laser_idx])
    {
        if (laser_opening_cnt[laser_idx] == 0)
            system_power_off_request(POWER_OFF_REASON_LASER_HOTPLUG);
    }

    /* ===== 激光管故障/带电拔检测 =====
     * 独立于参考电压, 未设置参考电压时也生效
     * 故障:  PD 过低但电流仍存在 → 仅跳过调光
     * 插拔:  PD 过低且电流≈0(断路), 且此前电流曾正常流过 → 连续确认后异常关机
     *        ("曾经接通"标志区分开机未接管与运行中被拔掉) */
    if (avg_pd < PD_FAULT_THRESHOLD)
    {
        if (current_mA < LASER_HOTPLUG_CURRENT_MA)
        {
            /* 只有"曾经接通"(电流曾正常)才判插拔; 开机就无管(电流从未流过)不误关机 */
            if (laser_was_connected[laser_idx] &&
                hotplug_confirm_count[laser_idx] < LASER_HOTPLUG_CONFIRM_COUNT)
            {
                hotplug_confirm_count[laser_idx]++;
                if (hotplug_confirm_count[laser_idx] >= LASER_HOTPLUG_CONFIRM_COUNT)
                    system_power_off_request(POWER_OFF_REASON_LASER_HOTPLUG);
            }
        }
        else
        {
            /* 电流正常 → 管子确实接通 (PD 低属于激光管故障, 非插拔) */
            hotplug_confirm_count[laser_idx] = 0;
            laser_was_connected[laser_idx]   = true;
        }
        return LASER_SAFETY_SKIP;
    }

    /* PD 正常: 清零插拔消抖计数, 防止残留 */
    hotplug_confirm_count[laser_idx] = 0;

    /* 电流正常 → 标记"曾经接通" */
    if (current_mA >= LASER_HOTPLUG_CURRENT_MA)
        laser_was_connected[laser_idx] = true;

    return LASER_SAFETY_NORMAL;
}
