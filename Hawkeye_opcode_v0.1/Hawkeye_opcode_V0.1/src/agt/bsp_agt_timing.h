/*
 * bsp_agt_timing.h
 *
 *  Created on: Sep 23, 2024
 *      Author: AXQ0527A
 */

#ifndef _BSP_AGT_TIMING_H_
#define _BSP_AGT_TIMING_H_
#include "hal_data.h"
#include "stdint.h"
#include "timer_pwm.h"
#include <iic/iic.h>
#include <key/bsp_irq_key.h>
#include <adc/bsp_adc.h>
#include <agt/bsp_agt_timing.h>
#include "hawkeye_config.h"

extern volatile bool pwm_in_high_phase;
extern volatile bool sampled_this_cycle;
extern volatile bool laser_enabled;
extern volatile bool laser_just_enabled;
/* ===== PWM 运行状态 ===== */
extern volatile bool pwm_in_high_phase;
extern volatile bool sampled_this_cycle;
extern volatile uint32_t pwm_cycle_counter;

/* ===== 全局 PD 均值 (mV)，供光功率校准使用 ===== */
/* 索引: [0]=H(SIDE) [1]=V1(HORIZ) [2]=V2(FRONT) */
extern volatile uint16_t g_pd_avg[3];

void GPT_Timing_Init(void);
void laser_pwm_tick_isr(void);

/* ===== 通用 duty 调节接口 ===== */
typedef fsp_err_t (*laser_set_pwm_func_t)(uint32_t, uint8_t);

void laser_adjust_duty(int avg_pd,
                       int reference,
                       uint32_t  *duty,
                       int duty_max,
                       int threshold,
                       laser_set_pwm_func_t set_func,
                       uint32_t pin,
                       const char *tag,
                       int current_mA,
                       int current_limit_mA);

void laser_main_loop_task(void);
void laser_mode_4_10deg_reset(void);
void laser_mode_10_90deg_reset(void);
void clear_flag(void);
void laser_mode_0_4deg(void);
void laser_mode_4_10deg(void);
void laser_mode_10_90deg(void);
void laser_mode_uncal(void);
void laser_mode_overheat(void);
/* 高温保护触发源 */
typedef enum {
    OVERHEAT_SOURCE_NTC_BOARD = 0,  /* 控制板NTC (运行期) */
    OVERHEAT_SOURCE_NTC_BOOT  = 1,  /* 控制板NTC (启动时) */
    OVERHEAT_SOURCE_BATTERY   = 2,  /* 电池包NTC过热 */
    OVERHEAT_SOURCE_DEBUG     = 3,  /* 调试命令 AT+OVERHEAT */
} overheat_source_t;

void system_overheat_request(overheat_source_t source);
#endif /* AGT_BSP_AGT_TIMING_H_ */
