/*
 * bsp_adc.h
 *
 *  Created on: Sep 19, 2024
 *      Author: AXQ0527A
 */
#ifndef _BSP_ADC_H_
#define _BSP_ADC_H_
#include "stdint.h"
#include "hal_data.h"
#include "common_utils.h"
#include <led/bsp_led.h>
#include "hawkeye_config.h"

/* ===== 原有声明 ===== */
extern volatile bool scan_complete_flag;

typedef enum {
    POWER_OFF_REASON_LASER_HOTPLUG  = 0,   /* MCU运行时带电插拔激光管 */
    POWER_OFF_REASON_BATTERY_LOW    = 1,
    POWER_OFF_REASON_LOCK_SW_RUNTIME= 2,
    POWER_OFF_REASON_I2C_FAIL       = 3,
    POWER_OFF_REASON_OVERHEAT       = 4,
    POWER_OFF_REASON_IDLE_TIMEOUT   = 5,
} power_off_reason_t;

/* ===== 新增：激光ADC buffer 及相关标志 ===== */
#define ADC_BUF_SIZE (64U)

extern volatile uint16_t buf_pd1[ADC_BUF_SIZE];
extern volatile uint16_t buf_ld1_1[ADC_BUF_SIZE];
extern volatile uint16_t buf_ld2_1[ADC_BUF_SIZE];

extern volatile uint16_t buf_pd2[ADC_BUF_SIZE];
extern volatile uint16_t buf_ld1_2[ADC_BUF_SIZE];
extern volatile uint16_t buf_ld2_2[ADC_BUF_SIZE];

extern volatile uint16_t buf_pd3[ADC_BUF_SIZE];
extern volatile uint16_t buf_ld1_3[ADC_BUF_SIZE];
extern volatile uint16_t buf_ld2_3[ADC_BUF_SIZE];

extern volatile uint8_t  g_laser_buf_ready;  /* buffer满标志，主循环消费后清0 */
extern volatile uint16_t g_bat_adc_raw;      /* 电池ADC最新值，回调里更新 */
extern volatile uint16_t g_bat_battery_temp_raw; /* 电池NTC温度ADC (AN022/CH22) */

/* ===== 函数声明（原有不变）===== */
void Read_ADC_Voltage(uint16_t *data_array, uint8_t channel);
void ADC_Init(void);
void Start_ADC(void);
void Stop_ADC(void);
uint16_t Read_ADC_Voltage_Value_BAT(void);
uint16_t Read_ADC_SquareWave_WeightedAverage(void);
uint16_t Read_ADC_SquareWave_WeightedAverage1(void);
uint16_t Read_ADC_SquareWave_WeightedAverage2(void);
void Read_ADC_LD1_LD2_Average(uint16_t *ld1_avg, uint16_t *ld2_avg);
void Read_ADC_LD1_LD2_Average1(uint16_t *ld1_avg, uint16_t *ld2_avg);
void Read_ADC_LD1_LD2_Average2(uint16_t *ld1_avg, uint16_t *ld2_avg);
void battery_voltage_task(void);
void battery_led_task(void);
void system_power_off(void);
void system_power_off_with_reason(power_off_reason_t reason);
void system_power_off_request(power_off_reason_t reason);
bool system_power_off_pending(void);
void system_power_off_process_pending(void);
void system_power_on(void);
void battery_state_init(uint16_t bat_adc);
bool battery_is_low_blink(void);
uint16_t Read_ADC_Voltage_Value_Battery_Temp(void);
uint16_t Read_ADC_Voltage_Value_NTC(void);
void battery_temp_check(void);
/* 上电预检测结果 (EN1 拉高前) */
typedef enum {
    PRE_EN1_NOT_DONE = 0,   /* 采样未完成, 继续 */
    PRE_EN1_PASS     = 1,   /* 采样完成, 三路电流均 <= PRE_EN1_CURRENT_LIMIT_MA, 可拉 EN1 */
    PRE_EN1_FAIL     = 2,   /* 采样完成, 至少一路电流 >  PRE_EN1_CURRENT_LIMIT_MA, 不拉 EN1 */
} pre_en1_result_t;

pre_en1_result_t laser_pre_en1_detect_step(void);

/* NTC 低温标记 (NTC ADC > 3150 → <-7°C) */
extern volatile bool g_ntc_low_temp;
#endif /* ADC_BSP_ADC_H_ */
