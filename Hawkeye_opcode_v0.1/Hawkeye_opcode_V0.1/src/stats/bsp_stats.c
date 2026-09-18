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
 *  stats_laser_key_press — 按键开启激光时调用
 * ====================================================================== */
void stats_laser_key_press(uint8_t ch)
{
    if (ch < 3U)
    {
        g_stats.laser_on_cnt[ch]++;
    }
}

/* ======================================================================
 *  stats_tick_1s — 每秒调用一次，更新所有时长
 * ====================================================================== */
void stats_tick_1s(const uint8_t laser_on[3],
                   uint8_t       angle_mode,
                   bool          bat_state_low)
{
    (void)bat_state_low; /* 不再使用 LED 闪烁时长统计 */

    /* 激光通道定义：[V1, V2, H] */
    uint8_t v1 = laser_on[0];
    uint8_t v2 = laser_on[1];
    uint8_t h = laser_on[2];

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
            g_stats.uncal_run_s++;
            break;
    }
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
             "+STATS:LASER_CNT V1=%lu V2=%lu H=%lu\r\n",
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

    /* ---- 未校准模式时长 ---- */
    format_hms(g_stats.uncal_run_s, hms, sizeof(hms));
    snprintf(msg, sizeof(msg), "+STATS:UNCAL_RUN=%s\r\n", hms);
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
