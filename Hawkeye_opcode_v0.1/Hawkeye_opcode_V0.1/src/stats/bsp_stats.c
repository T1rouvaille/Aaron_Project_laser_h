/**
 * @file  bsp_stats.c
 * @brief 运行统计模块实现
 *
 * 设计原则：
 *  - RAM 累加器实时更新，无 Flash 写操作（延长 Flash 寿命）
 *  - 仅在关机前调用 stats_flush_to_flash() 一次性写入
 *  - 所有时间单位为"秒"，由主循环 1000ms 任务驱动
 */

#include "stats/bsp_stats.h"
#include "debug_uart/bsp_debug_uart.h"
#include "stdio.h"
#include "string.h"
#include "iic/iic.h"   /* 引入 gAngleMode / angle_mode_t */

/* ======================================================================
 *  RAM 累加器（掉电丢失，启动时从 Flash 恢复）
 * ====================================================================== */
static flash_stats_t g_stats;
static volatile bool g_stats_tick_flag = false;
/* ======================================================================
 *  stats_init
 * ====================================================================== */
void stats_init(void)
{

    flash_load_stats(&g_stats);

    R_AGT_Open(&g_agt1_ctrl, &g_agt1_cfg);
    R_AGT_Start(&g_agt1_ctrl);

}

/* ======================================================================
 *  stats_laser_key_press — 按键开启/关闭激光时各调用一次（开+关各 +1）
 * ====================================================================== */
void stats_laser_key_press(uint8_t ch)
{
    if (ch < 3U)
    {
        g_stats.laser_on_cnt[ch]++;
    }
}

/* ======================================================================
 *  stats_protection_event — 记录一次保护检测事件（边沿触发）
 * ====================================================================== */
void stats_protection_event(stats_prot_event_t ev)
{
    switch (ev)
    {
        case STATS_PROT_NTC_HOT_LASER:
            g_stats.ntc_hot_laser++;
            break;
        case STATS_PROT_NTC_COLD_LASER:
            g_stats.ntc_cold_laser++;
            break;
        case STATS_PROT_NTC_FROZEN_PACK:
            g_stats.ntc_frozen_pack++;
            break;
        case STATS_PROT_H1_OVERCUR:
            g_stats.h1_over_cur++;
            break;
        case STATS_PROT_V1_OVERCUR:
            g_stats.v1_over_cur++;
            break;
        case STATS_PROT_V2_OVERCUR:
            g_stats.v2_over_cur++;
            break;
        default:
            break;
    }
}

/* ======================================================================
 *  temp 段温度分区状态机（带回差, 单位=次数）
 *
 *  分区定义（控制板 NTC ADC, 温度越高 ADC 越小）：
 *    high   → ADC <  NTC_TEMP_HIGH_ENTER_ADC (2004, >40°C)
 *    normal → ADC ∈ [2004, 2850] (0~40°C)
 *    low    → ADC >  NTC_TEMP_LOW_ENTER_ADC  (2850, <0°C)
 *
 *  回差 NTC_TEMP_HYSTERESIS(400)：进入分区用严格阈值, 退出需跨过回差带,
 *  避免温度在阈值附近抖动导致反复计数。
 * ====================================================================== */
typedef enum
{
    TEMP_BAND_NORMAL = 0,
    TEMP_BAND_HIGH,
    TEMP_BAND_LOW,
} temp_band_t;

static temp_band_t s_temp_band = TEMP_BAND_NORMAL;

static temp_band_t temp_band_classify(uint16_t ntc_adc)
{
    if (ntc_adc < NTC_TEMP_HIGH_ENTER_ADC) return TEMP_BAND_HIGH;
    if (ntc_adc > NTC_TEMP_LOW_ENTER_ADC)  return TEMP_BAND_LOW;
    return TEMP_BAND_NORMAL;
}

static void temp_band_count(temp_band_t band)
{
    switch (band)
    {
        case TEMP_BAND_HIGH:   g_stats.temp_high_cnt++;   break;
        case TEMP_BAND_LOW:    g_stats.temp_low_cnt++;    break;
        case TEMP_BAND_NORMAL:
        default:               g_stats.temp_normal_cnt++; break;
    }
}

void stats_temp_band_init(uint16_t ntc_adc)
{
    s_temp_band = temp_band_classify(ntc_adc);
    temp_band_count(s_temp_band);
}

void stats_temp_band_shutdown(uint16_t ntc_adc)
{
    temp_band_count(temp_band_classify(ntc_adc));
}

void stats_temp_band_track(uint16_t ntc_adc)
{
    switch (s_temp_band)
    {
        case TEMP_BAND_HIGH:
            /* 温度回落到 <40°C-回差 → 回 normal */
            if (ntc_adc > (NTC_TEMP_HIGH_ENTER_ADC + NTC_TEMP_HYSTERESIS))
            {
                s_temp_band = TEMP_BAND_NORMAL;
                temp_band_count(TEMP_BAND_NORMAL);
            }
            break;

        case TEMP_BAND_LOW:
            /* 温度回升到 >0°C+回差 → 回 normal */
            if (ntc_adc < (NTC_TEMP_LOW_ENTER_ADC - NTC_TEMP_HYSTERESIS))
            {
                s_temp_band = TEMP_BAND_NORMAL;
                temp_band_count(TEMP_BAND_NORMAL);
            }
            break;

        case TEMP_BAND_NORMAL:
        default:
            /* 变热越过 40°C → high */
            if (ntc_adc < NTC_TEMP_HIGH_ENTER_ADC)
            {
                s_temp_band = TEMP_BAND_HIGH;
                temp_band_count(TEMP_BAND_HIGH);
            }
            /* 变冷越过 0°C → low */
            else if (ntc_adc > NTC_TEMP_LOW_ENTER_ADC)
            {
                s_temp_band = TEMP_BAND_LOW;
                temp_band_count(TEMP_BAND_LOW);
            }
            break;
    }
}

/* ======================================================================
 *  stats_tick_1s — 每秒调用一次，更新所有时长
 * ====================================================================== */
void stats_tick_1s(const uint8_t laser_on[3],
                   uint8_t       angle_mode,
                   bool          bat_state_low,
                   uint16_t      ntc_adc)
{
    (void)bat_state_low; /* 不再使用 LED 闪烁时长统计 */

    /* 激光通道定义：gLaserOn 实际顺序为 [H, V1, V2]（与 power_calibration/bsp_adc/bsp_agt 一致） */
    uint8_t h  = laser_on[0];
    uint8_t v1 = laser_on[1];
    uint8_t v2 = laser_on[2];

    /* Slope = 倾角 > 4°（即 4~10° 或 >10° 模式） */
    bool slope = (angle_mode == ANGLE_MODE_4_10) || (angle_mode == ANGLE_MODE_10_90);

    /* 1. 设备总运行时长 */
    g_stats.total_run_s++;

    /* 2. V1 单独开启时长 */
    if (v1 && !v2 && !h)
    {
        g_stats.v1_only_s++;
    }

    /* 4. V2 单独开启时长 */
    if (!v1 && v2 && !h)
    {
        g_stats.v2_only_s++;
    }

    /* 5. H 单独开启时长 */
    if (!v1 && !v2 && h)
    {
        g_stats.h_only_s++;
    }

    /* 6. V1 + H 同时开启时长（仅这两路）*/
    if (v1 && !v2 && h)
    {
        g_stats.v1_h_only_s++;
    }

    /* 7. V2 + H 同时开启时长（仅这两路）*/
    if (!v1 && v2 && h)
    {
        g_stats.v2_h_only_s++;
    }

    /* 8. V1 + V2 同时开启时长（仅这两路）*/
    if (v1 && v2 && !h)
    {
        g_stats.v1_v2_only_s++;
    }

    /* 9. 三路同时开启时长 */
    if (v1 && v2 && h)
    {
        g_stats.all3_laser_s++;
    }

    /* 9b. Slope 段：激光组合 × (倾角 > 4°) 交叉时长（仅 7 个非 H2 组合） */
    if (slope)
    {
        if (!v1 && !v2 && h)  { g_stats.slope_h_s++; }      /* H1       @112 */
        if (v1 && !v2 && !h)  { g_stats.slope_v1_s++; }     /* V1       @120 */
        if (!v1 && v2 && !h)  { g_stats.slope_v2_s++; }     /* V2       @124 */
        if (v1 && !v2 && h)   { g_stats.slope_v1_h_s++; }   /* H1&V1    @132 */
        if (!v1 && v2 && h)   { g_stats.slope_v2_h_s++; }   /* H1&V2    @136 */
        if (v1 && v2 && !h)   { g_stats.slope_v1_v2_s++; }  /* V1&V2    @148 */
        if (v1 && v2 && h)    { g_stats.slope_all3_s++; }   /* H1&V1&V2 @160 */
    }

    /* 10. 角度模式时长 */
    switch (angle_mode)
    {
        case ANGLE_MODE_0_4:
            g_stats.angle_0_4_s++;
            break;
        case ANGLE_MODE_4_10:
            g_stats.angle_4_10_s++;
            break;
        case ANGLE_MODE_10_90:
            g_stats.angle_10_90_s++;
            break;
        case ANGLE_MODE_UNCAL:
        default:
            g_stats.angle_0_4_s++;   /* 未校准模式按 0-4° 计（无独立 uncal 概念） */
            break;
    }

    /* 11. temp 段：温度分区切换跟踪（带回差, 单位=次数, 不依赖激光状态） */
    stats_temp_band_track(ntc_adc);
}

/* ======================================================================
 *  stats_flush_to_flash — 关机前调用
 * ====================================================================== */
void stats_flush_to_flash(void)
{

}

/* ======================================================================
 *  stats_print_all — AT+GET_STATS 串口打印
 * ====================================================================== */

/* 辅助：将秒数格式化为 "XXXh YYm ZZs" */
static void format_hms(uint32_t seconds, char *buf, uint8_t buf_len)
{
    uint32_t h = seconds / 3600U;
    uint32_t m = (seconds % 3600U) / 60U;
    uint32_t s = seconds % 60U;
    snprintf(buf, buf_len, " %luh %02lum %02lus",
             (unsigned long)h,
             (unsigned long)m,
             (unsigned long)s);
}

void stats_print_all(void)
{
    char msg[80];
    char hms[24];

    uart9_send_blocking("+RESP:STATS BEGIN\r\n");

    /* ---- 设备总运行时长 ---- */
    format_hms(g_stats.total_run_s, hms, sizeof(hms));
    snprintf(msg, sizeof(msg), "+STATS:TOTAL_RUN=%s\r\n", hms);
    uart9_send_blocking(msg);

    /* ---- 各路激光开启次数（按键测试）---- */
    snprintf(msg, sizeof(msg),
             "+STATS:LASER_CNT H=%lu V1=%lu V2=%lu\r\n",
             (unsigned long)g_stats.laser_on_cnt[0],
             (unsigned long)g_stats.laser_on_cnt[1],
             (unsigned long)g_stats.laser_on_cnt[2]);
    uart9_send_blocking(msg);

    /* ---- V1 单独开启时长 ---- */
    format_hms(g_stats.v1_only_s, hms, sizeof(hms));
    snprintf(msg, sizeof(msg), "+STATS:V1_ONLY=%s\r\n", hms);
    uart9_send_blocking(msg);

    /* ---- V2 单独开启时长 ---- */
    format_hms(g_stats.v2_only_s, hms, sizeof(hms));
    snprintf(msg, sizeof(msg), "+STATS:V2_ONLY=%s\r\n", hms);
    uart9_send_blocking(msg);

    /* ---- H 单独开启时长 ---- */
    format_hms(g_stats.h_only_s, hms, sizeof(hms));
    snprintf(msg, sizeof(msg), "+STATS:H_ONLY=%s\r\n", hms);
    uart9_send_blocking(msg);

    /* ---- V1+H 同时开启时长 ---- */
    format_hms(g_stats.v1_h_only_s, hms, sizeof(hms));
    snprintf(msg, sizeof(msg), "+STATS:V1_H=%s\r\n", hms);
    uart9_send_blocking(msg);

    /* ---- V2+H 同时开启时长 ---- */
    format_hms(g_stats.v2_h_only_s, hms, sizeof(hms));
    snprintf(msg, sizeof(msg), "+STATS:V2_H=%s\r\n", hms);
    uart9_send_blocking(msg);

    /* ---- V1+V2 同时开启时长 ---- */
    format_hms(g_stats.v1_v2_only_s, hms, sizeof(hms));
    snprintf(msg, sizeof(msg), "+STATS:V1_V2=%s\r\n", hms);
    uart9_send_blocking(msg);

    /* ---- 三路同时开启时长 ---- */
    format_hms(g_stats.all3_laser_s, hms, sizeof(hms));
    snprintf(msg, sizeof(msg), "+STATS:ALL3_LASER=%s\r\n", hms);
    uart9_send_blocking(msg);

    /* ---- 角度模式时长 ---- */
    format_hms(g_stats.angle_0_4_s, hms, sizeof(hms));
    snprintf(msg, sizeof(msg), "+STATS:ANGLE_0_4=%s\r\n", hms);
    uart9_send_blocking(msg);

    format_hms(g_stats.angle_4_10_s, hms, sizeof(hms));
    snprintf(msg, sizeof(msg), "+STATS:ANGLE_4_10=%s\r\n", hms);
    uart9_send_blocking(msg);

    format_hms(g_stats.angle_10_90_s, hms, sizeof(hms));
    snprintf(msg, sizeof(msg), "+STATS:ANGLE_10_90=%s\r\n", hms);
    uart9_send_blocking(msg);

    uart9_send_blocking("+RESP:STATS END\r\n");
}

const flash_stats_t *stats_get_ptr(void)
{
    g_stats.magic = FLASH_STATS_MAGIC;   /* 确保magic有效 */
    return &g_stats;
}

/* ======================================================================
 *  stats_clear
 * ====================================================================== */
void stats_clear(void)
{
    memset(&g_stats, 0, sizeof(flash_stats_t));
    g_stats.magic = FLASH_STATS_MAGIC;
    flash_clear_stats();
}
void agt1_stats_callback(timer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    g_stats_tick_flag = true;
}
bool stats_tick_pending(void)
{
    if (g_stats_tick_flag)
    {
        g_stats_tick_flag = false;
        return true;
    }
    return false;
}
