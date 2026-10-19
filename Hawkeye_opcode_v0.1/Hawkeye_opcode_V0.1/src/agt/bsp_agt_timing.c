/*
 * bsp_agt_timing.c
 *
 *  Created on: Sep 23, 2024
 *      Author: AXQ0527A
 */
#include <agt/bsp_agt_timing.h>
#include <key/bsp_irq_key.h>
#include <led/bsp_led.h>
#include <iic/iic.h>
#include "hal_data.h"
#include "r_gpt.h"
#include "timer_pwm.h"
#include "hawkeye_config.h"
#include <laser_safety/laser_safety.h>
#include <adc/bsp_adc.h>
#include "wdt/wdt.h"
#include <adc/bsp_adc.h>
/* ===== 低温保护辅助: PID输出时自动缩放, 保持ISR相位时序 ===== */
static uint32_t lt_scale_duty(uint32_t duty)
{
    return g_ntc_low_temp ? ((duty * NTC_LOW_TEMP_DUTY_PERCENT) / 100) : duty;
}
/* ===== 高温保护状态 ===== */
static uint16_t          overheat_ms        = 0U;   /* 高温模式内部计时（ms） */
static uint8_t           overheat_blink_cnt = 0U;   /* 已完成的完整闪烁次数 */
static bool              overheat_phase_on  = false;/* 当前是 ON 半周期 */
static overheat_source_t g_overheat_source;          /* 记录触发源 */


volatile uint32_t g_task1_ms = 0;
volatile uint32_t g_task2_ms = 0;
typedef struct {
    int kp;
    int ki;
    int kd;
    int integral;
    int last_error;
    int integral_limit;
} pid_t;

static pid_t laser_pid[3] = {
    {4, 1, 0, 0, 0, 80000},
    {4, 1, 0, 0, 0, 80000},
    {4, 1, 0, 0, 0, 80000},
};

/* ===== 恒流模式PID（电流闭环）===== */
static pid_t current_pid[3] = {
    {2, 1, 0, 0, 0, 50000},
    {2, 1, 0, 0, 0, 50000},
    {2, 1, 0, 0, 0, 50000},
};

/* ===== 恒流模式标志（一旦触发永不退出）===== */
static bool current_limiting[3] = {false, false, false};

/* 电流超限消抖计数：连续 CURRENT_LIMIT_CONFIRM_COUNT 次才锁存，防止上电暂态尖峰误触发 */
static uint8_t current_limit_count[3] = {0, 0, 0};

static int   last_duty_check[3]    = {0, 0, 0};  // 低温检测用：上次记录的duty值
static int   last_pd_check[3]      = {0, 0, 0};  // 低温检测用：上次记录的PD值

extern volatile bool g_print_enabled;

volatile uint8_t gSystemTickFlag = 0;   // 1ms节拍标志

uint16_t pd_samples [SAMPLE_COUNT] = {0};
uint16_t ld1_samples[SAMPLE_COUNT] = {0};
uint16_t ld2_samples[SAMPLE_COUNT] = {0};

uint16_t pd_samples1 [SAMPLE_COUNT] = {0};
uint16_t ld1_samples1[SAMPLE_COUNT] = {0};
uint16_t ld2_samples1[SAMPLE_COUNT] = {0};

uint16_t pd_samples2 [SAMPLE_COUNT] = {0};
uint16_t ld1_samples2[SAMPLE_COUNT] = {0};
uint16_t ld2_samples2[SAMPLE_COUNT] = {0};

volatile uint32_t pwm_cycle_counter = 0;

/* ===== 4-10度模式闪烁控制（GPT9 = 1ms）===== */
static uint16_t blink_cnt_4_10 = 0;
static uint8_t  blink_on_4_10  = 1;
/* =====10 - 90度模式闪烁控制（GPT9 = 1ms）===== */
static uint16_t t10_t_ms = 0;
static uint8_t  t10_blink200k_on = 1;
static uint8_t  t10_sub_ms = 0;

volatile bool pwm_in_high_phase = false;
volatile bool sampled_this_cycle = false;
volatile bool laser_enabled = false;
volatile bool laser_just_enabled = false;

volatile uint8_t sample_delay_cnt = 0;
volatile bool sample_delay_active = false;

uint8_t sample_index = 0;
bool ready_to_adjust = false;

/* ===== 目标参考值 ===== */
extern volatile int reference_voltage;
extern volatile int reference_voltage1;
extern volatile int reference_voltage2;

/* ===== duty ===== */
uint32_t duty00  = 60;
uint32_t duty11 = 60;
uint32_t duty22 = 60;

volatile bool adc_sample_request = false;  // 采样请求标志

/* ===== 全局 PD 均值，供光功率校准模块读取 ===== */
volatile uint16_t g_pd_avg[3] = {0, 0, 0};  /* [0]=H(SIDE) [1]=V1(HORIZ) [2]=V2(FRONT) */

/* ===== 全局电流均值 (mA)，索引与 g_pd_avg 一致: [0]=H(SIDE) [1]=V1(HORIZ) [2]=V2(FRONT) ===== */
volatile int32_t g_current_mA[3] = {0, 0, 0};

void GPT_Timing_Init(void)
{
    R_GPT_Open(&g_timer9_ctrl, &g_timer9_cfg);


}


void GPT9_CALLBACK(timer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    gSystemTickFlag = 1;

    if (gAngleMode == ANGLE_MODE_OVERHEAT)
    {
        laser_mode_overheat();   /* 高温模式不依赖 laser_any_on() */
        return;
    }

    /* ===== 校准期间激光闪烁: 每50ms翻转一次 ===== */
    if (g_calib_laser_flash)
    {
        /* 跳过正常模式 PWM，避免 8.47kHz 调制干扰闪烁节拍 */
        static uint16_t calib_flash_timer = 0;
        if (++calib_flash_timer >= 50)
        {
            calib_flash_timer = 0;
            g_calib_laser_on = !g_calib_laser_on;
            if (g_calib_laser_on)
            {
                /* 亮相位: 8.47kHz=100%, 200kHz duty=60 */
                set_laser1_8470_intensity(6500, GPT_IO_PIN_GTIOCA);
                set_laser2_8470_intensity(6500, TIMER_PIN);
                set_laser3_8470_intensity(6500, GPT_IO_PIN_GTIOCA);
                set_laser1_200k_intensity(LASER_IDLE_DUTY, TIMER_PIN);
                set_laser2_200k_intensity(LASER_DEFAULT_DUTY, TIMER_PIN);
                set_laser3_200k_intensity(LASER_IDLE_DUTY, GPT_IO_PIN_GTIOCA);
            }
            else
            {
                /* 暗相位: 8.47kHz=100%, 200kHz duty=20 (载波维持) */
                set_laser1_8470_intensity(6500, GPT_IO_PIN_GTIOCA);
                set_laser2_8470_intensity(6500, TIMER_PIN);
                set_laser3_8470_intensity(6500, GPT_IO_PIN_GTIOCA);
                set_laser1_200k_intensity(LASER_IDLE_DUTY, TIMER_PIN);
                set_laser2_200k_intensity(LASER_IDLE_DUTY, TIMER_PIN);
                set_laser3_200k_intensity(LASER_IDLE_DUTY, GPT_IO_PIN_GTIOCA);
            }
        }
        return;  /* 校准闪烁期间不执行正常模式 PWM */
    }

    if (laser_any_on())
    {
        if (g_ntc_low_temp)
        {
            /* 低温保护: 先缩放duty变量, 让模式函数各相位乘数正确级联 */
            uint32_t save00 = duty00, save11 = duty11, save22 = duty22;
            duty00 = (duty00 * NTC_LOW_TEMP_DUTY_PERCENT) / 100;
            duty11 = (duty11 * NTC_LOW_TEMP_DUTY_PERCENT) / 100;
            duty22 = (duty22 * NTC_LOW_TEMP_DUTY_PERCENT) / 100;
            laser_pwm_tick_isr();
            duty00 = save00;
            duty11 = save11;
            duty22 = save22;
        }
        else
        {
            laser_pwm_tick_isr();
        }
    }
}

void laser_adjust_duty(int avg_pd,
                       int reference,
                       uint32_t *duty,
                       int duty_max,
                       int threshold,
                       laser_set_pwm_func_t set_func,
                       uint32_t pin,
                       const char *tag,
                       int current_mA,
                       int current_limit_mA)
{
    /* 通道名 → laser_idx 映射: "V2"(FRONT)=0, "H"(SIDE)=1, "V1"(HORIZ)=2 */
    int laser_idx;
    if      (tag[0] == 'V' && tag[1] == '2') laser_idx = 0;
    else if (tag[0] == 'H')                  laser_idx = 1;
    else                                     laser_idx = 2;

    /* ===== 带电插拔 / 激光管故障检测 (laser_safety 模块) =====
     * 独立于参考电压, 未设置参考电压时也生效。
     * SKIP = PD 过低(故障/拔出), 跳过本次调光。 */
    if (laser_safety_hotplug_check(laser_idx, avg_pd, current_mA) == LASER_SAFETY_SKIP)
        return;

    if (reference <= 0) return;

    /* 低温模式: 同步缩放PID目标, 使PD收敛到更低的光功率 */
    if (g_ntc_low_temp)
    {
        reference = (reference * NTC_LOW_TEMP_REF_PERCENT) / 100;
    }

    int pd_error  = avg_pd - reference;
    if (abs(pd_error) > PD_ABNORMAL_THRESHOLD) return;

    /* 记录更新 */
    last_duty_check[laser_idx] = (int)(*duty);
    last_pd_check[laser_idx]   = avg_pd;

    /* ==========================================================
     * 电流采样合理性校验：低占空比下电流不应超限，
     * 若出现说明电流采样读数异常，跳过本次调光
     * ========================================================== */
    if (current_mA > current_limit_mA &&
        *duty <= CURRENT_SENSE_FAULT_DUTY_THRESHOLD)
    {
        return;
    }

    /* ==========================================================
     * 恒流模式触发：电流超限后永不退出
     * 增加消抖：连续 CURRENT_LIMIT_CONFIRM_COUNT 次超限才锁存，
     * 单次上电暂态尖峰不触发。
     * ========================================================== */
    if (current_mA > current_limit_mA)
    {
        if (!current_limiting[laser_idx])
        {
            current_limit_count[laser_idx]++;
            if (current_limit_count[laser_idx] >= CURRENT_LIMIT_CONFIRM_COUNT)
            {
                /* 确认超限：锁存并重置两套PID积分 */
                current_limiting[laser_idx]       = true;
                current_limit_count[laser_idx]    = 0;
                current_pid[laser_idx].integral   = 0;
                current_pid[laser_idx].last_error = 0;
                laser_pid[laser_idx].integral     = 0;
            }
        }
    }
    else
    {
        /* 未锁存前电流回落，清零消抖计数 */
        if (!current_limiting[laser_idx])
            current_limit_count[laser_idx] = 0;
    }

    /* ==========================================================
     * 恒流模式：以 current_limit_mA 为目标持续PID
     * 增大duty → 提高电流，减小duty → 降低电流
     * ========================================================== */
    if (current_limiting[laser_idx])
    {
        /* cur_error > 0：电流超限，需降duty
         * cur_error < 0：电流未到限，可升duty */
        int cur_error = current_mA - current_limit_mA;

        pid_t *cpid = &current_pid[laser_idx];

        if (abs(cur_error) > 20)
        {
            /* 爬升阶段：纯P快速响应 */
            cpid->integral   = 0;
            cpid->last_error = cur_error;

            int p_out = cpid->kp * cur_error;

            int delta = 0;
            if      (p_out >  50) delta = -1;   /* 超限，降duty */
            else if (p_out < -50) delta = +1;   /* 未到限，升duty */

            if (delta != 0)
            {
                if (delta < 0 && *duty == 0U) return;
                *duty = (uint32_t)((int)(*duty) + delta);
                if (*duty > (uint32_t)duty_max) *duty = (uint32_t)duty_max;
                set_func(lt_scale_duty(*duty), (uint8_t)pin);
            }
        }
        else
        {
            /* 稳定阶段：完整PID */
            cpid->integral += cur_error;
            if (cpid->integral >  cpid->integral_limit) cpid->integral =  cpid->integral_limit;
            if (cpid->integral < -cpid->integral_limit) cpid->integral = -cpid->integral_limit;

            int derivative   = cur_error - cpid->last_error;
            cpid->last_error = cur_error;

            int p_term = cpid->kp * cur_error;
            int i_term = cpid->ki * cpid->integral / 1000;
            int d_term = cpid->kd * derivative;
            int output = p_term + i_term + d_term;

            int delta = 0;
            if (cur_error > 0)
            {
                /* 超限：按比例降duty，每2mA降1个duty，最少-1，最多-10 */
                delta = -(cur_error / 2);
                if (delta == 0)  delta = -1;
                if (delta < -10) delta = -10;
            }
            else if (output < -10)
            {
                /* 未到限：慢升 */
                delta = +1;
            }

            if (delta == 0) return;
            if (delta < 0 && *duty == 0U) return;
            *duty = (uint32_t)((int)(*duty) + delta);
            if (*duty > (uint32_t)duty_max) *duty = (uint32_t)duty_max;
            set_func(lt_scale_duty(*duty), (uint8_t)pin);
        }
        return;  /* 恒流模式处理完毕，永不跌落到恒压PID */
    }

    /* ==========================================================
     * 恒压模式：光功率PID（以 avg_pd 跟踪 reference）
     * ========================================================== */
    pid_t *pid = &laser_pid[laser_idx];

    if (abs(pd_error) > threshold)
    {
        /* 爬升阶段：纯P */
        pid->integral   = 0;
        pid->last_error = pd_error;

        int p_out = pid->kp * pd_error;

        int delta = 0;
        if (p_out < -50)
        {
            /* PD低于目标：按比例升duty */
            delta = -(pd_error / 400);   /* pd_error为负，结果为正 */
            if (delta == 0)  delta = +1;
            if (delta >  5)  delta = +5;
        }
        else if (p_out > 50)
        {
            /* PD高于目标：按比例降duty */
            delta = -(pd_error / 100);   /* pd_error为正，结果为负 */
            if (delta == 0)  delta = -1;
            if (delta < -5)  delta = -5;
        }




        if (delta != 0)
        {
            if (delta < 0 && *duty == 0U) return;
            *duty = (uint32_t)((int)(*duty) + delta);
            if (*duty > (uint32_t)duty_max) *duty = (uint32_t)duty_max;
            set_func(lt_scale_duty(*duty), (uint8_t)pin);
        }
        return;
    }

    /* 稳定阶段：完整PID */
    pid->integral += pd_error;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    int derivative  = pd_error - pid->last_error;
    pid->last_error = pd_error;

    int p_term = pid->kp * pd_error;
    int i_term = pid->ki * pid->integral / 1000;
    int d_term = pid->kd * derivative;
    int output = p_term + i_term + d_term;

    int delta = 0;
    if      (output < -100) delta = +1;
    else if (output >  100) delta = -1;

    if (delta == 0) return;
    if (delta < 0 && *duty == 0U) return;
    *duty = (uint32_t)((int)(*duty) + delta);
    if (*duty > (uint32_t)duty_max) *duty = (uint32_t)duty_max;
    set_func(lt_scale_duty(*duty), (uint8_t)pin);
}

/* ======================================================================
 *  查询接口: 某通道是否已锁存恒流模式 (供光功率校准判断电流到限)
 *  laser_idx: 0=V2(FRONT) 1=H(SIDE) 2=V1(HORIZ)
 * ====================================================================== */
bool laser_is_current_limiting(int laser_idx)
{
    if (laser_idx < 0 || laser_idx >= 3) return false;
    return current_limiting[laser_idx];
}

void laser_main_loop_task(void)
{

    if (gAngleMode == ANGLE_MODE_OVERHEAT) return;  /* 高温模式禁止主循环调光 */
    // ===== 采样阶段：改为检查 g_laser_buf_ready =====
    if (g_laser_buf_ready && !sampled_this_cycle)
    {
        sampled_this_cycle  = true;
        g_laser_buf_ready   = 0;  // 消费掉，允许回调填下一批

        // -------- 激光1 --------
        uint16_t pd_raw, ld1_raw, ld2_raw;
        pd_raw = Read_ADC_SquareWave_WeightedAverage();
        Read_ADC_LD1_LD2_Average(&ld1_raw, &ld2_raw);
        pd_samples[sample_index]  = (uint16_t)((pd_raw  * 3300U) / 4095U);
        ld1_samples[sample_index] = (uint16_t)((ld1_raw * 3300U) / 4095U);
        ld2_samples[sample_index] = (uint16_t)((ld2_raw * 3300U) / 4095U);

        // -------- 激光2 --------
        uint16_t pd_raw1, ld1_raw1, ld2_raw1;
        pd_raw1 = Read_ADC_SquareWave_WeightedAverage1();
        Read_ADC_LD1_LD2_Average1(&ld1_raw1, &ld2_raw1);
        pd_samples1[sample_index]  = (uint16_t)((pd_raw1  * 3300U) / 4095U);
        ld1_samples1[sample_index] = (uint16_t)((ld1_raw1 * 3300U) / 4095U);
        ld2_samples1[sample_index] = (uint16_t)((ld2_raw1 * 3300U) / 4095U);

        // -------- 激光3 --------
        uint16_t pd_raw2, ld1_raw2, ld2_raw2;
        pd_raw2 = Read_ADC_SquareWave_WeightedAverage2();
        Read_ADC_LD1_LD2_Average2(&ld1_raw2, &ld2_raw2);
        pd_samples2[sample_index]  = (uint16_t)((pd_raw2  * 3300U) / 4095U);
        ld1_samples2[sample_index] = (uint16_t)((ld1_raw2 * 3300U) / 4095U);
        ld2_samples2[sample_index] = (uint16_t)((ld2_raw2 * 3300U) / 4095U);

        sample_index++;
        if (sample_index >= SAMPLE_COUNT)
        {
            sample_index    = 0U;
            ready_to_adjust = true;
        }
    }

    // ===== 平均 + 调光阶段：完全不变 =====
    if (ready_to_adjust == true)
    {
        wdt_feed();  /* PID调光前喂狗 */
        ready_to_adjust = false;

        /* 打印分频: 每 PRINT_INTERVAL_CYCLES 个调光周期才打印一次 */
        static uint32_t print_cycle_count = 0;
        bool print_now = false;
        if (++print_cycle_count >= PRINT_INTERVAL_CYCLES)
        {
            print_cycle_count = 0;
            print_now = true;
        }

        char msg[80];

        // -------- 激光1 --------
        {
            uint32_t pd_sum = 0U, ld1_sum = 0U, ld2_sum = 0U;

            for (uint16_t i = 0; i < SAMPLE_COUNT; i++)
            {
                pd_sum  += pd_samples[i];
                ld1_sum += ld1_samples[i];
                ld2_sum += ld2_samples[i];
            }

            uint32_t pd_avg  = pd_sum  / SAMPLE_COUNT;
            uint32_t ld1_avg = ld1_sum / SAMPLE_COUNT;
            uint32_t ld2_avg = ld2_sum / SAMPLE_COUNT;
            /* 更新全局 PD 均值 → V2(FRONT) */
            g_pd_avg[2] = (uint16_t)pd_avg;
            int32_t voltage    = (int32_t)ld1_avg - (int32_t)ld2_avg;
            int32_t current_mA = (abs(voltage) * CURRENT_SENSE_GAIN_X1000) / 1000;
            g_current_mA[2] = current_mA;

            if (gLaserOn[2] != 0U)
                {
                    if (g_print_enabled && print_now)
                    {
                        sprintf(msg, "PD-V2=%lu\r\n", pd_avg);
                        uart9_send_blocking(msg);
                        sprintf(msg, "I-V2=%ld\r\n", current_mA);
                        uart9_send_blocking(msg);
                       sprintf(msg, "duty-V2=%ld\r\n", duty00);
                        uart9_send_blocking(msg);
                        /* sprintf(msg, "effD-V2=%lu\r\n", lt_scale_duty(duty00));
                        uart9_send_blocking(msg);
                        sprintf(msg, "mode-V2=%s\r\n",
                                g_ntc_low_temp ? "LO_TEMP" : "NORMAL");
                        uart9_send_blocking(msg);*/
                    }
                    laser_adjust_duty((int)pd_avg, reference_voltage,
                                      &duty00, 260, 40,
                                      set_laser1_200k_intensity,
                                      TIMER_PIN, "V2",
                                      (int)current_mA, CURRENT_LIMIT_V2_MA);
                }
        }
        wdt_feed();  /* 激光1调光后喂狗 */

        // -------- 激光2 --------
        {
            uint32_t pd_sum1 = 0U, ld1_sum1 = 0U, ld2_sum1 = 0U;

            for (uint16_t i = 0; i < SAMPLE_COUNT; i++)
            {
                pd_sum1  += pd_samples1[i];
                ld1_sum1 += ld1_samples1[i];
                ld2_sum1 += ld2_samples1[i];
            }

            uint32_t pd_avg1  = pd_sum1  / SAMPLE_COUNT;
            uint32_t ld1_avg1 = ld1_sum1 / SAMPLE_COUNT;
            uint32_t ld2_avg1 = ld2_sum1 / SAMPLE_COUNT;
            /* 更新全局 PD 均值 → H(SIDE) */
            g_pd_avg[0] = (uint16_t)pd_avg1;
            int32_t voltage1    = (int32_t)ld1_avg1 - (int32_t)ld2_avg1;
            int32_t current_mA1 = (abs(voltage1) * CURRENT_SENSE_GAIN_X1000) / 1000;
            g_current_mA[0] = current_mA1;

            if (gLaserOn[0] != 0U)
            {
                if (g_print_enabled && print_now)
                {
                    sprintf(msg, "PD-H=%lu\r\n", pd_avg1);
                    uart9_send_blocking(msg);
                    sprintf(msg, "I-H=%ld\r\n", current_mA1);
                    uart9_send_blocking(msg);
                    sprintf(msg, "duty-H=%ld\r\n", duty11);
                    uart9_send_blocking(msg);
                    /* sprintf(msg, "effD-H=%lu\r\n", lt_scale_duty(duty11));
                    uart9_send_blocking(msg);
                    sprintf(msg, "mode-H=%s\r\n",
                            g_ntc_low_temp ? "LO_TEMP" : "NORMAL");
                    uart9_send_blocking(msg);*/
                }
                laser_adjust_duty((int)pd_avg1, reference_voltage1,
                                  &duty11, 350, 40,
                                  set_laser2_200k_intensity,
                                  TIMER_PIN, "H",
                                  (int)current_mA1, CURRENT_LIMIT_H_MA);
                wdt_feed();
            }
        }

        // -------- 激光3 --------
        {
            uint32_t pd_sum2 = 0U, ld1_sum2 = 0U, ld2_sum2 = 0U;

            for (uint16_t i = 0; i < SAMPLE_COUNT; i++)
            {
                pd_sum2  += pd_samples2[i];
                ld1_sum2 += ld1_samples2[i];
                ld2_sum2 += ld2_samples2[i];
            }

            uint32_t pd_avg2  = pd_sum2  / SAMPLE_COUNT;
            uint32_t ld1_avg2 = ld1_sum2 / SAMPLE_COUNT;
            uint32_t ld2_avg2 = ld2_sum2 / SAMPLE_COUNT;
            /* 更新全局 PD 均值 → V1(HORIZ) */
            g_pd_avg[1] = (uint16_t)pd_avg2;
            int32_t voltage2    = (int32_t)ld1_avg2 - (int32_t)ld2_avg2;
            int32_t current_mA2 = (abs(voltage2) * CURRENT_SENSE_GAIN_X1000) / 1000;
            g_current_mA[1] = current_mA2;

            if (gLaserOn[1] != 0U)
            {
                if (g_print_enabled && print_now)
                {
                    sprintf(msg, "PD-V1=%lu\r\n", pd_avg2);
                    uart9_send_blocking(msg);
                    sprintf(msg, "I-V1=%ld\r\n", current_mA2);
                    uart9_send_blocking(msg);
                    sprintf(msg, "duty-V1=%ld\r\n", duty22);
                    uart9_send_blocking(msg);
                    /*sprintf(msg, "effD-V1=%lu\r\n", lt_scale_duty(duty22));
                    uart9_send_blocking(msg);
                    sprintf(msg, "mode-V1=%s\r\n",
                            g_ntc_low_temp ? "LO_TEMP" : "NORMAL");
                    uart9_send_blocking(msg);*/
                }
                laser_adjust_duty((int)pd_avg2, reference_voltage2,
                                  &duty22, 260, 40,
                                  set_laser3_200k_intensity,
                                  GPT_IO_PIN_GTIOCA, "V1",
                                  (int)current_mA2, CURRENT_LIMIT_V1_MA);
                wdt_feed();
            }
        }
    }

    /* 低温保护已由 ISR 预缩放 + laser_adjust_duty 的 lt_scale_duty 覆盖,
     * 此处不再额外调用 set_laser*_200k_intensity, 避免破坏模式函数的相位时序 */
    wdt_feed();
}



/* ==========================================
 * 优化后的 laser_mode_0_4deg()
 * 改进: 只在状态改变时设置PWM,避免重复设置
 * ========================================== */

void laser_mode_0_4deg(void)
{
    pwm_cycle_counter++;

     //延时计数
    if (sample_delay_active == true)
    {
        if (++sample_delay_cnt >= 3)
        {
            sample_delay_active = false;
            if (pwm_in_high_phase && !sampled_this_cycle)
            {
                adc_sample_request = true;
            }
        }
    }

     //===== 关键改进: 只在特定时刻设置PWM =====

    if (pwm_cycle_counter == 1)
    {
         //===== 1ms: 切换到低电平 =====

        // 8.47kHz设置为50%
        set_laser1_8470_intensity(2834, GPT_IO_PIN_GTIOCA);
        set_laser2_8470_intensity(2834, TIMER_PIN);          // ← 改这里，GTIOCA改成TIMER_PIN
        set_laser3_8470_intensity(2834, GPT_IO_PIN_GTIOCA);

        uint32_t low_duty0;
        uint32_t low_duty1;
        uint32_t low_duty2;

        if (gLaserOn[2])
            low_duty0 = (duty00 * LOW_PHASE_DUTY_PERCENT) / 100U;
        else
            low_duty0 = 20;

        if (gLaserOn[0])
            low_duty1 = (duty11 * LOW_PHASE_DUTY_PERCENT) / 100U;
        else
            low_duty1 = 20;

        if (gLaserOn[1])
            low_duty2 = (duty22 * LOW_PHASE_DUTY_PERCENT) / 100U;
        else
            low_duty2 = 20;

        set_laser1_200k_intensity(low_duty0, TIMER_PIN);
        set_laser2_200k_intensity(low_duty1, TIMER_PIN);
        set_laser3_200k_intensity(low_duty2, GPT_IO_PIN_GTIOCA);


        pwm_in_high_phase = 0;
    }
    else if (pwm_cycle_counter == 5)
    {
         //===== 5ms: 切换到高电平 =====

        // 8.47kHz设置为100%
        set_laser1_8470_intensity(6500, GPT_IO_PIN_GTIOCA);
        set_laser2_8470_intensity(6500, TIMER_PIN);          // ← 改这里，GTIOCA改成TIMER_PIN
        set_laser3_8470_intensity(6500, GPT_IO_PIN_GTIOCA);

        // ✅ 200kHz同时开启(只设置一次!)
        if (gLaserOn[2])
            set_laser1_200k_intensity(duty00, TIMER_PIN);
        else
            set_laser1_200k_intensity(LASER_IDLE_DUTY, TIMER_PIN);

        if (gLaserOn[0])
            set_laser2_200k_intensity(duty11, TIMER_PIN);
        else
            set_laser2_200k_intensity(LASER_IDLE_DUTY, TIMER_PIN);

        if (gLaserOn[1])
            set_laser3_200k_intensity(duty22, GPT_IO_PIN_GTIOCA);
        else
            set_laser3_200k_intensity(LASER_IDLE_DUTY, GPT_IO_PIN_GTIOCA);

        pwm_in_high_phase    = true;
        sampled_this_cycle   = false;
        sample_delay_cnt     = 0;
        sample_delay_active  = true;
        adc_sample_request   = false;
    }
    else if (pwm_cycle_counter >= 48)
    {
         //===== 48ms: 周期结束 =====
        pwm_cycle_counter = 0;
        pwm_in_high_phase = 0;
        adc_sample_request = false;
    }

    // ✅ 不再在每1ms都设置200kHz!
    // 只在1ms和5ms这两个时刻设置
}

void laser_mode_4_10deg(void)
{

    if (sample_delay_active == true)
    {
        if (++sample_delay_cnt >= 3)
        {
            sample_delay_active = false;

            // ✅ 延时结束时设置采样请求
            if (pwm_in_high_phase && !sampled_this_cycle)
            {
                adc_sample_request = true;
            }
        }
    }

    /* ===== 1Hz闪烁计时：500ms 翻转 ===== */
    if (++blink_cnt_4_10 >= 500)
    {
        blink_cnt_4_10 = 0;
        blink_on_4_10 ^= 1;
    }

    /* =================================================
     * 8.47kHz：三路一直100% duty
     * ================================================= */

    /*if (gLaserOn[0]) */set_laser1_8470_intensity(6500, GPT_IO_PIN_GTIOCA);

    /*if (gLaserOn[1])*/ set_laser2_8470_intensity(6500, TIMER_PIN);

    /*if (gLaserOn[2])*/ set_laser3_8470_intensity(6500, GPT_IO_PIN_GTIOCA);


    /* =================================================
     * 200kHz：三路按键独立 + 1Hz闪烁
     * ================================================= */
    if (blink_on_4_10)
    {
        if (gLaserOn[2]) set_laser1_200k_intensity(duty00, TIMER_PIN);
        if (gLaserOn[0]) set_laser2_200k_intensity(duty11, TIMER_PIN);
        if (gLaserOn[1]) set_laser3_200k_intensity(duty22, GPT_IO_PIN_GTIOCA);

        /* ======== 采样窗口打开 ======== */
        if (!pwm_in_high_phase)
        {
            pwm_in_high_phase   = true;
            sampled_this_cycle  = false;
            sample_delay_cnt    = 0;
            sample_delay_active = true;
        }
    }
    else
    {
        set_laser1_200k_intensity(LASER_IDLE_DUTY, TIMER_PIN);
        set_laser2_200k_intensity(LASER_IDLE_DUTY, TIMER_PIN);
        set_laser3_200k_intensity(LASER_IDLE_DUTY, GPT_IO_PIN_GTIOCA);

        pwm_in_high_phase = false;
    }
}

void laser_mode_4_10deg_reset(void)
{
    blink_cnt_4_10 = 0;
    blink_on_4_10  = 1;
}



void laser_mode_10_90deg(void)
{

    if (sample_delay_active == true)
    {
        if (++sample_delay_cnt >= 3)
        {
            sample_delay_active = false;

            // ✅ 延时结束时设置采样请求
            if (pwm_in_high_phase && !sampled_this_cycle)
            {
                adc_sample_request = true;
            }
        }
    }


    t10_t_ms++;
    if (t10_t_ms >= 5000)
    {
        t10_t_ms = 0;
        t10_blink200k_on = 1;
        t10_sub_ms = 0;
    }

    /* ========= 前1.5s：200kHz 250ms闪 ========= */
    if (t10_t_ms < 1500)
    {
        if ((t10_t_ms % 250) == 0)
            t10_blink200k_on ^= 1;

        /* 8.47kHz 一直100% */
        /*if (gLaserOn[0])*/ set_laser1_8470_intensity(6500, GPT_IO_PIN_GTIOCA);


       /* if (gLaserOn[1])*/ set_laser2_8470_intensity(6500, TIMER_PIN);


        /*if (gLaserOn[2]) */set_laser3_8470_intensity(6500, GPT_IO_PIN_GTIOCA);


        /* 200kHz 闪 */
        if (t10_blink200k_on)
        {
            if (gLaserOn[2]) set_laser1_200k_intensity(duty00, TIMER_PIN);

            if (gLaserOn[0]) set_laser2_200k_intensity(duty11, TIMER_PIN);

            if (gLaserOn[1]) set_laser3_200k_intensity(duty22, GPT_IO_PIN_GTIOCA);


            /* ======== 采样窗口 ======== */
            if (!pwm_in_high_phase)
            {
                pwm_in_high_phase   = true;
                sampled_this_cycle  = false;
                sample_delay_cnt    = 0;
                sample_delay_active = true;
            }
        }
        else
        {
            set_laser1_200k_intensity(LASER_IDLE_DUTY, TIMER_PIN);
            set_laser2_200k_intensity(LASER_IDLE_DUTY, TIMER_PIN);
            set_laser3_200k_intensity(LASER_IDLE_DUTY, GPT_IO_PIN_GTIOCA);
            pwm_in_high_phase = false;
        }
        return;
    }

    /* ========= 后3.5s：同0–4° ========= */

    /* 200kHz 常开 */
    if (gLaserOn[2]) set_laser1_200k_intensity(duty00, TIMER_PIN);


    if (gLaserOn[0]) set_laser2_200k_intensity(duty11, TIMER_PIN);


    if (gLaserOn[1]) set_laser3_200k_intensity(duty22, GPT_IO_PIN_GTIOCA);


    t10_sub_ms++;
    if (t10_sub_ms >= 48) t10_sub_ms = 0;

    if (t10_sub_ms < 44)
    {
        /* 8.47kHz = 100% → 采样 */
        /*if (gLaserOn[0])*/ set_laser1_8470_intensity(6500, GPT_IO_PIN_GTIOCA);


        /*if (gLaserOn[1])*/ set_laser2_8470_intensity(6500, TIMER_PIN);


        /*if (gLaserOn[2])*/ set_laser3_8470_intensity(6500, GPT_IO_PIN_GTIOCA);


        if (!pwm_in_high_phase)
        {
            pwm_in_high_phase   = true;
            sampled_this_cycle  = false;
            sample_delay_cnt    = 0;
            sample_delay_active = true;
        }
    }
    else
    {
        // -------8.47kHz = 50% → 不采样
        set_laser1_8470_intensity(2834, GPT_IO_PIN_GTIOCA);
        set_laser2_8470_intensity(2834, TIMER_PIN);
        set_laser3_8470_intensity(2834, GPT_IO_PIN_GTIOCA);

        //2---------00kHz 对应减15
        uint32_t low_duty0;
        uint32_t low_duty1;
        uint32_t low_duty2;

        if (gLaserOn[2])
            low_duty0 = (duty00 * LOW_PHASE_DUTY_PERCENT) / 100U;
        else
            low_duty0 = 20;

        if (gLaserOn[0])
            low_duty1 = (duty11 * LOW_PHASE_DUTY_PERCENT) / 100U;
        else
            low_duty1 = 20;

        if (gLaserOn[1])
            low_duty2 = (duty22 * LOW_PHASE_DUTY_PERCENT) / 100U;
        else
            low_duty2 = 20;

        set_laser1_200k_intensity(low_duty0, TIMER_PIN);
        set_laser2_200k_intensity(low_duty1, TIMER_PIN);
        set_laser3_200k_intensity(low_duty2, GPT_IO_PIN_GTIOCA);

        pwm_in_high_phase = false;
    }
}


void laser_mode_10_90deg_reset(void)
{
    t10_t_ms = 0;
    t10_blink200k_on = 1;
    t10_sub_ms = 0;
}
void laser_mode_uncal(void)
{
    pwm_cycle_counter++;

    /* ===== 等待PWM高电平稳定时间 ===== */
    if (sample_delay_active == true)
    {
        if (++sample_delay_cnt >= 3)
        {
            sample_delay_active = false;
        }
    }

    /* ===============================
     * 低电平阶段（50%窗口，仅用于节拍）
     * =============================== */
    if (pwm_cycle_counter == 1)
    {
        /* 8.47kHz 固定 100% */

        /*if (gLaserOn[0])*/ set_laser1_8470_intensity(6500, GPT_IO_PIN_GTIOCA);


        /*if (gLaserOn[1])*/ set_laser2_8470_intensity(6500, TIMER_PIN);

        /*if (gLaserOn[2])*/ set_laser3_8470_intensity(6500, GPT_IO_PIN_GTIOCA);


        /* 200kHz 实时 duty */
        if (gLaserOn[2]) set_laser1_200k_intensity(duty00, TIMER_PIN);


        if (gLaserOn[0]) set_laser2_200k_intensity(duty11, TIMER_PIN);


        if (gLaserOn[1]) set_laser3_200k_intensity(duty22, GPT_IO_PIN_GTIOCA);


        pwm_in_high_phase = 0;
    }

    /* ===============================
     * 高电平阶段（允许采样）
     * =============================== */
    else if (pwm_cycle_counter == 5)
    {
        /* 8.47kHz 仍然 100% */

        /*if (gLaserOn[0])*/ set_laser1_8470_intensity(6500, GPT_IO_PIN_GTIOCA);


        /*if (gLaserOn[1])*/ set_laser2_8470_intensity(6500, TIMER_PIN);

        /*if (gLaserOn[2])*/ set_laser3_8470_intensity(6500, GPT_IO_PIN_GTIOCA);


        /* 200kHz duty 保持 */
        if (gLaserOn[2]) set_laser1_200k_intensity(duty00, TIMER_PIN);


        if (gLaserOn[0]) set_laser2_200k_intensity(duty11, TIMER_PIN);


        if (gLaserOn[1]) set_laser3_200k_intensity(duty22, GPT_IO_PIN_GTIOCA);

        pwm_in_high_phase   = true;
        sampled_this_cycle  = false;
        sample_delay_cnt    = 0;
        sample_delay_active = true;
    }

    /* ===============================
     * 周期结束
     * =============================== */
    else if (pwm_cycle_counter >= 48)
    {
        pwm_cycle_counter = 0;
        pwm_in_high_phase = 0;
    }
}


void laser_pwm_tick_isr(void)
{
    switch (gAngleMode)
    {

        case ANGLE_MODE_0_4:
            laser_mode_0_4deg();
            break;

        case ANGLE_MODE_4_10:
            laser_mode_4_10deg();
            break;

        case ANGLE_MODE_10_90:
            laser_mode_10_90deg();
            break;

        case ANGLE_MODE_UNCAL:
        default:
            laser_mode_0_4deg();
            //laser_mode_uncal();
            break;
    }
}

void clear_flag(void)
{
    pwm_cycle_counter   = 0;
    pwm_in_high_phase   = 0;
    sampled_this_cycle  = 0;
    sample_delay_active = false;
    sample_delay_cnt    = 0;
}

void laser_mode_overheat(void)
{
    overheat_ms++;

    uint16_t pos      = (uint16_t)((overheat_ms - 1U) % OVERHEAT_CYCLE_MS);
    uint16_t cycle_no = (uint16_t)((overheat_ms - 1U) / OVERHEAT_CYCLE_MS);
    bool     should_on = (pos < OVERHEAT_ON_MS);

    /* 完成 5 次后关机 */
    if (cycle_no >= OVERHEAT_BLINK_COUNT)
    {
        set_laser1_8470_intensity(6500, GPT_IO_PIN_GTIOCA);
        set_laser2_8470_intensity(6500, TIMER_PIN);
        set_laser3_8470_intensity(6500, GPT_IO_PIN_GTIOCA);
        set_laser1_200k_intensity(0, TIMER_PIN);
        set_laser2_200k_intensity(0, TIMER_PIN);
        set_laser3_200k_intensity(0, GPT_IO_PIN_GTIOCA);
        system_power_off_request(POWER_OFF_REASON_OVERHEAT);
        return;
    }

    /* 8.47kHz：始终 6500，每次都写 */
    set_laser1_8470_intensity(6500, GPT_IO_PIN_GTIOCA);
    set_laser2_8470_intensity(6500, TIMER_PIN);
    set_laser3_8470_intensity(6500, GPT_IO_PIN_GTIOCA);

    /* 200kHz：相位没变就不重复写寄存器 */
    static bool last_should_on = false;
    if (should_on == last_should_on) return;
    last_should_on = should_on;

    if (should_on)
    {
        set_laser1_200k_intensity(gLaserOn[2] ? duty00 : LASER_IDLE_DUTY, TIMER_PIN);
        set_laser2_200k_intensity(gLaserOn[0] ? duty11 : LASER_IDLE_DUTY, TIMER_PIN);
        set_laser3_200k_intensity(gLaserOn[1] ? duty22 : LASER_IDLE_DUTY, GPT_IO_PIN_GTIOCA);
    }
    else
    {
        set_laser1_200k_intensity(LASER_IDLE_DUTY, TIMER_PIN);
        set_laser2_200k_intensity(LASER_IDLE_DUTY, TIMER_PIN);
        set_laser3_200k_intensity(LASER_IDLE_DUTY, GPT_IO_PIN_GTIOCA);
    }
}
static const char * overheat_source_to_str(overheat_source_t s)
{
    switch (s)
    {
        case OVERHEAT_SOURCE_NTC_BOARD: return "NTC_BOARD";
        case OVERHEAT_SOURCE_NTC_BOOT:  return "NTC_BOOT";
        case OVERHEAT_SOURCE_BATTERY:   return "BATTERY_PACK";
        case OVERHEAT_SOURCE_DEBUG:     return "DEBUG_CMD";
        default:                        return "UNKNOWN";
    }
}

void system_overheat_request(overheat_source_t source)
{
    /* 防止重复进入 */
    if (gAngleMode == ANGLE_MODE_OVERHEAT) return;

    g_overheat_source  = source;
    overheat_ms        = 0U;
    overheat_blink_cnt = 0U;
    overheat_phase_on  = false;
    gAngleMode         = ANGLE_MODE_OVERHEAT;

    uart9_send_blocking("+DBG:OVERHEAT source=");
    uart9_send_blocking(overheat_source_to_str(source));
    uart9_send_blocking("\r\n");
}
