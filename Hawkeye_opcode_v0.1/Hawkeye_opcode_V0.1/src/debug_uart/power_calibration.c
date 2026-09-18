/**
 * @file    power_calibration.c
 * @brief   光功率自动校准 — 核心逻辑实现
 *
 * 协议流程（PC 侧循环）:
 *   while (未到位) {
 *       读取光功率计 → 发 AT+POWCAL=<CH>,<power_x100>
 *       收到 +POWCAL:<CH>,ADJ,<ref_mV> → 等待稳定 → 再次测量
 *       收到 +POWCAL:<CH>,DONE,<ref_mV> → 该路完成
 *       收到 +POWCAL:<CH>,SAT,<ref_mV>  → 饱和，无法达到目标
 *   }
 *
 * 调压逻辑:
 *   error = target - measured_power (x100, 0.01mW)
 *   |error| > THRESH_LARGE (0.50mW) → 大步进 (200mV)
 *   |error| > THRESH_MED   (0.20mW) → 中步进 (50mV)
 *   |error| > THRESH_SMALL (0.05mW) → 小步进 (10mV)
 *   error ∈ [-THRESH_SMALL, 0]        → 到位 (仅允许正误差)
 */

#include "power_calibration.h"
#include "flash/flash.h"
#include "agt/bsp_agt_timing.h"     /* g_pd_avg[3] */
#include "key/bsp_irq_key.h"        /* gLaserOn[3] */
#include "hawkeye_config.h"

/* ======================================================================
 *  外部引用 — 三路参考电压（定义在 bsp_debug_uart.c）
 * ====================================================================== */
extern volatile int reference_voltage;   /* V2 (FRONT) */
extern volatile int reference_voltage1;  /* H  (SIDE)  */
extern volatile int reference_voltage2;  /* V1 (HORIZ) */

/* ======================================================================
 *  全局常量
 * ====================================================================== */
const char * const powcal_ch_names[POWCAL_CH_NUM] = { "H", "V1", "V2" };

/* 目标光功率 (x100, 0.01mW) — 对应 H/V1/V2 */
const int powcal_targets[POWCAL_CH_NUM] = {
    POWCAL_TARGET_H,    /* H:  2.00mW */
    POWCAL_TARGET_V1,   /* V1: 1.00mW */
    POWCAL_TARGET_V2    /* V2: 1.00mW */
};

/* ======================================================================
 *  通道状态
 * ====================================================================== */
static powcal_status_t powcal_state[POWCAL_CH_NUM] = {
    POWCAL_STAT_IDLE,
    POWCAL_STAT_IDLE,
    POWCAL_STAT_IDLE
};

/* ======================================================================
 *  保护用历史数据 — 检测"升压但光功率不变"
 * ====================================================================== */
static int last_power[POWCAL_CH_NUM] = {0, 0, 0};  /* 上次上报的光功率 */
static int last_ref_mv[POWCAL_CH_NUM] = {0, 0, 0}; /* 上次的参考电压 */
static int no_resp_cnt[POWCAL_CH_NUM]  = {0, 0, 0}; /* 连续无响应次数 */

/* ======================================================================
 *  ch → gLaserOn 索引映射
 *  H(SIDE)→0, V1(HORIZ)→1, V2(FRONT)→2
 * ====================================================================== */
static int get_laser_idx(powcal_channel_t ch)
{
    /* H=gLaserOn[0], V1=gLaserOn[1], V2=gLaserOn[2] → 恰好 ch==idx */
    return (int)ch;
}

/* ======================================================================
 *  内部辅助 — 获取通道对应的参考电压指针
 * ====================================================================== */
static int* get_ref_ptr(powcal_channel_t ch)
{
    switch (ch) {
        case POWCAL_CH_H:  return (int *)&reference_voltage1;  /* SIDE  → H  */
        case POWCAL_CH_V1: return (int *)&reference_voltage2;  /* HORIZ → V1 */
        case POWCAL_CH_V2: return (int *)&reference_voltage;   /* FRONT → V2 */
        default:           return NULL;
    }
}

/* ======================================================================
 *  核心函数: 根据光功率误差调整参考电压
 *
 *  返回: ADJ=已调整需继续测量 / DONE=到位 / SAT=饱和
 * ====================================================================== */
powcal_status_t powcal_process(powcal_channel_t ch, int power)
{
    if (ch >= POWCAL_CH_NUM) return POWCAL_STAT_IDLE;

    int target  = powcal_targets[ch];
    int *ref_mv = get_ref_ptr(ch);
    if (NULL == ref_mv) return POWCAL_STAT_IDLE;

    /* ==================================================================
     *  保护 #3: 激光未打开 → 拒绝调节
     * ================================================================== */
    {
        int idx = get_laser_idx(ch);
        if (idx >= 0 && idx < 3 && gLaserOn[idx] == 0U)
        {
            powcal_state[ch] = POWCAL_STAT_LASER_OFF;
            return POWCAL_STAT_LASER_OFF;
        }
    }

    /* ==================================================================
     *  保护 #1: PD 初始值过低 (<50mV) → 拒绝调节
     * ================================================================== */
    int pd_base = (int)g_pd_avg[ch];
    if (pd_base < POWCAL_PD_MIN_MV)
    {
        powcal_state[ch] = POWCAL_STAT_PD_LOW;
        return POWCAL_STAT_PD_LOW;
    }

    /* ==================================================================
     *  首次校准：从当前 PD 均值初始化参考电压
     * ================================================================== */
    if (powcal_state[ch] == POWCAL_STAT_IDLE)
    {
        *ref_mv             = pd_base;
        last_ref_mv[ch]     = *ref_mv;
        /* 不设置 last_power — 首次调用还未调节，power 就是基准值，
         * 设为相等会导致保护 #2 误判为 NO_RESP。
         * last_power 保持初始值 0，首次比较 power<=0 恒为 false。 */
        powcal_state[ch]    = POWCAL_STAT_ADJ;
    }

    int error = target - power;  /* >0 功率不足需升压, <0 功率过大需降压 */

    /* ===== 到位判断 (仅允许 +0.05mW 正误差) =====
     * power >= target 且 power <= target + 0.05mW → DONE
     * power <  target → 不通过，需继续升压 */
    if (error <= 0 && error >= -POWCAL_THRESH_SMALL) {
        powcal_state[ch] = POWCAL_STAT_DONE;
        return POWCAL_STAT_DONE;
    }

    int abs_err = (error >= 0) ? error : -error;

    /* ===== 计算步进电压 (mV) ===== */
    int step_mv;
    if (abs_err > POWCAL_THRESH_LARGE) {
        step_mv = POWCAL_STEP_LARGE_MV;     /* 200mV */
    } else if (abs_err > POWCAL_THRESH_MED) {
        step_mv = POWCAL_STEP_MED_MV;       /* 50mV */
    } else {
        step_mv = POWCAL_STEP_SMALL_MV;     /* 10mV */
    }

    /* ===== 调整参考电压 ===== */
    int new_ref = *ref_mv;
    if (error > 0) {
        /* 功率不足 → 升高参考电压 */
        new_ref += step_mv;
    } else {
        /* 功率过大 → 降低参考电压 */
        new_ref -= step_mv;
    }

    /* ===== 限幅 ===== */
    if (new_ref > POWCAL_VOLT_MAX_MV) {
        new_ref = POWCAL_VOLT_MAX_MV;
    }
    if (new_ref < POWCAL_VOLT_MIN_MV) {
        new_ref = POWCAL_VOLT_MIN_MV;
    }

    /* ==================================================================
     *  保护 #2: 电压升高但光功率未响应
     *  连续3次无响应才拒绝，单次不响应仅记数，容忍测量波动
     * ================================================================== */
    if (new_ref > last_ref_mv[ch] && power <= last_power[ch])
    {
        no_resp_cnt[ch]++;
        if (no_resp_cnt[ch] >= 6)
        {
            /* 连续3次升压但光功率不涨 → 激光不响应 */
            powcal_state[ch] = POWCAL_STAT_NO_RESP;
            return POWCAL_STAT_NO_RESP;
        }
    }
    else
    {
        /* 功率有响应 → 重置连续无响应计数 */
        no_resp_cnt[ch] = 0;
    }

    *ref_mv = new_ref;

    /* ===== 更新历史数据 ===== */
    last_ref_mv[ch] = *ref_mv;
    last_power[ch]  = power;

    /* ===== 饱和检测: 到上限且功率仍不足 ===== */
    if (new_ref >= POWCAL_VOLT_MAX_MV && error > POWCAL_THRESH_SMALL) {
        powcal_state[ch] = POWCAL_STAT_SAT;
        return POWCAL_STAT_SAT;
    }

    /* ===== 谷底检测: 到下限且功率仍过大 ===== */
    if (new_ref <= POWCAL_VOLT_MIN_MV && error < -POWCAL_THRESH_SMALL) {
        powcal_state[ch] = POWCAL_STAT_SAT;  /* 实际是反向饱和 */
        return POWCAL_STAT_SAT;
    }

    powcal_state[ch] = POWCAL_STAT_ADJ;
    return POWCAL_STAT_ADJ;
}

/* ======================================================================
 *  保存校准结果到 Flash
 * ====================================================================== */
bool powcal_save(void)
{
    fsp_err_t err = flash_save_reference(reference_voltage,
                                         reference_voltage1,
                                         reference_voltage2);
    return (err == FSP_SUCCESS);
}

/* ======================================================================
 *  查询接口
 * ====================================================================== */
int powcal_get_ref_mv(powcal_channel_t ch)
{
    int *ref_mv = get_ref_ptr(ch);
    return (ref_mv != NULL) ? *ref_mv : 0;
}

powcal_status_t powcal_get_status(powcal_channel_t ch)
{
    if (ch >= POWCAL_CH_NUM) return POWCAL_STAT_IDLE;
    return powcal_state[ch];
}

void powcal_reset(powcal_channel_t ch)
{
    if (ch < POWCAL_CH_NUM) {
        powcal_state[ch]  = POWCAL_STAT_IDLE;
        last_power[ch]    = 0;
        last_ref_mv[ch]   = 0;
        no_resp_cnt[ch]   = 0;
    }
}
