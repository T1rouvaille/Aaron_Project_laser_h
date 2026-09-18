#include <adc/bsp_adc.h>
#include <stdio.h>
#include <math.h>
#include "common_utils.h"
#include <led/bsp_led.h>
#include "debug_uart/bsp_debug_uart.h"
#include "stats/bsp_stats.h"
#include <string.h>
#include <key/bsp_irq_key.h>
#include "flash/flash.h"

/* ===== 激光ADC buffer ===== */
volatile uint16_t buf_pd1[ADC_BUF_SIZE]   = {0};
volatile uint16_t buf_ld1_1[ADC_BUF_SIZE] = {0};
volatile uint16_t buf_ld2_1[ADC_BUF_SIZE] = {0};

volatile uint16_t buf_pd2[ADC_BUF_SIZE]   = {0};
volatile uint16_t buf_ld1_2[ADC_BUF_SIZE] = {0};
volatile uint16_t buf_ld2_2[ADC_BUF_SIZE] = {0};

volatile uint16_t buf_pd3[ADC_BUF_SIZE]   = {0};
volatile uint16_t buf_ld1_3[ADC_BUF_SIZE] = {0};
volatile uint16_t buf_ld2_3[ADC_BUF_SIZE] = {0};

volatile uint8_t  g_laser_buf_ready = 0;
volatile uint16_t g_bat_adc_raw     = 0;
static   uint8_t  s_buf_index       = 0;
volatile uint16_t g_bat_battery_temp_raw     = 0;
/* ===== 保留原有变量（开机段还在用） ===== */
volatile bool scan_complete_flag = false;

uint16_t PD1;
uint16_t PD2;
uint16_t PD3;

/* ===== 电池状态 ===== */
typedef enum {
    BAT_STATE_FULL = 0,
    BAT_STATE_HIGH,
    BAT_STATE_MID,
    BAT_STATE_CRITICAL,
} bat_state_t;

static bat_state_t  g_bat_state          = BAT_STATE_FULL;
static uint8_t      g_bat_critical_count = 0;
static bool         g_bat_shutdown_pending      = false;
static bool         g_system_power_off_pending  = false;
static power_off_reason_t g_system_power_off_reason = POWER_OFF_REASON_LASER_HOTPLUG;

/* ===== 外部依赖 ===== */
extern volatile bool pwm_in_high_phase;
extern volatile bool sample_delay_active;

/* ===== ADC 回调 ===== */
void adc_callback(adc_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);

    /* 电池通道：任何时候都更新 */
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_9, (uint16_t *)&g_bat_adc_raw);

    /* 电池温度通道：任何时候都更新 */
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_22, (uint16_t *)&g_bat_battery_temp_raw);

    /* 开机段兼容：保留 scan_complete_flag */
    scan_complete_flag = true;

    /* 激光通道：只在采样窗口内存buffer */
    if (!pwm_in_high_phase || sample_delay_active)
        return;

    /* 上一批buffer还没被主循环消费，丢弃本次 */
    if (g_laser_buf_ready)
        return;

    if (s_buf_index < ADC_BUF_SIZE)
    {
        R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_2,  (uint16_t *)&buf_pd1[s_buf_index]);
        R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_0,  (uint16_t *)&buf_ld1_1[s_buf_index]);
        R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_1,  (uint16_t *)&buf_ld2_1[s_buf_index]);

        R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_20, (uint16_t *)&buf_pd2[s_buf_index]);
        R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_5,  (uint16_t *)&buf_ld1_2[s_buf_index]);
        R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_6,  (uint16_t *)&buf_ld2_2[s_buf_index]);

        R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_8,  (uint16_t *)&buf_pd3[s_buf_index]);
        R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_19, (uint16_t *)&buf_ld1_3[s_buf_index]);
        R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_7,  (uint16_t *)&buf_ld2_3[s_buf_index]);

        s_buf_index++;
    }

    if (s_buf_index >= ADC_BUF_SIZE)
    {
        s_buf_index       = 0;
        g_laser_buf_ready = 1;
    }
}

void ADC_Init(void)
{
    fsp_err_t err;
    err = R_ADC_Open(&g_adc0_ctrl, &g_adc0_cfg);
    err = R_ADC_ScanCfg(&g_adc0_ctrl, &g_adc0_channel_cfg);
    assert(FSP_SUCCESS == err);
}

void Start_ADC(void)
{
    R_ADC_ScanStart(&g_adc0_ctrl);
    R_ADC0->ADCSR_b.ADST = 1;
}

void Stop_ADC(void)
{
    R_ADC_ScanStop(&g_adc0_ctrl);
}

/* 快速选择优化版本 - O(n) 平均复杂度 */
/* 使用Bentley-McIlroy 3-way partition快速选择算法 */
static uint16_t trimmed_avg(volatile uint16_t *buf, uint16_t size,
                             uint16_t trim, uint16_t valid)
{
    uint16_t tmp[64];
    uint32_t sum = 0;
    uint16_t i, j, k;
    
    /* 简洁拷贝 */
    __disable_irq();
    for (i = 0; i < size; i++) tmp[i] = buf[i];
    __enable_irq();
    
    /* 快速选择：找到第trim+valid小的元素即可 */
    uint16_t left = 0, right = size - 1;
    uint16_t target_end = trim + valid;
    
    while (left < right) {
        uint16_t pivot = tmp[(left + right) >> 1];
        i = left; j = right; k = left;
        
        /* 3-way partition */
        while (k <= j) {
            if (tmp[k] < pivot) {
                uint16_t t = tmp[i]; tmp[i] = tmp[k]; tmp[k] = t;
                i++; k++;
            } else if (tmp[k] > pivot) {
                uint16_t t = tmp[j]; tmp[j] = tmp[k]; tmp[k] = t;
                j--;
            } else {
                k++;
            }
        }
        
        /* 判断目标区域位置 */
        if (target_end <= i) {
            right = i - 1;
        } else if (trim > j) {
            left = j + 1;
        } else {
            left = right;  // 目标区域已就位，退出循环
        }
    }
    
    /* 求和中间valid个值 */
    for (i = trim; i < target_end; i++) sum += tmp[i];
    return (uint16_t)(sum / valid);
}
/* ===== 对外采样接口：直接读buffer，无阻塞 ===== */
uint16_t Read_ADC_SquareWave_WeightedAverage(void)
{
    return trimmed_avg(buf_pd1, ADC_BUF_SIZE, TRIM_SIDES, VALID_COUNT);
}

uint16_t Read_ADC_SquareWave_WeightedAverage1(void)
{
    return trimmed_avg(buf_pd2, ADC_BUF_SIZE, TRIM_SIDES, VALID_COUNT);
}

uint16_t Read_ADC_SquareWave_WeightedAverage2(void)
{
    return trimmed_avg(buf_pd3, ADC_BUF_SIZE, TRIM_SIDES, VALID_COUNT);
}

void Read_ADC_LD1_LD2_Average(uint16_t *ld1_avg, uint16_t *ld2_avg)
{
    *ld1_avg = trimmed_avg(buf_ld1_1, ADC_BUF_SIZE, LD_TRIM_SIDES, LD_VALID_COUNT);
    *ld2_avg = trimmed_avg(buf_ld2_1, ADC_BUF_SIZE, LD_TRIM_SIDES, LD_VALID_COUNT);
}

void Read_ADC_LD1_LD2_Average1(uint16_t *ld1_avg, uint16_t *ld2_avg)
{
    *ld1_avg = trimmed_avg(buf_ld1_2, ADC_BUF_SIZE, LD_TRIM_SIDES, LD_VALID_COUNT);
    *ld2_avg = trimmed_avg(buf_ld2_2, ADC_BUF_SIZE, LD_TRIM_SIDES, LD_VALID_COUNT);
}

void Read_ADC_LD1_LD2_Average2(uint16_t *ld1_avg, uint16_t *ld2_avg)
{
    *ld1_avg = trimmed_avg(buf_ld1_3, ADC_BUF_SIZE, LD_TRIM_SIDES, LD_VALID_COUNT);
    *ld2_avg = trimmed_avg(buf_ld2_3, ADC_BUF_SIZE, LD_TRIM_SIDES, LD_VALID_COUNT);
}

/* ===== 电池采样：直接读回调更新的全局变量 ===== */
uint16_t Read_ADC_Voltage_Value_BAT(void)
{
    return (uint16_t)g_bat_adc_raw;
}

uint16_t Read_ADC_Voltage_Value_Battery_Temp(void)
{
    return (uint16_t)g_bat_battery_temp_raw;
}

uint16_t Read_ADC_Voltage_Value_NTC(void)
{
    uint16_t ntc_raw = 0;
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_10, &ntc_raw);
    return ntc_raw;
}

/* ======================================================================
 *  上电预检测 — EN1 拉高前采样三路激光管电流/电压 (带电插拔保护)
 *  通道映射与 adc_callback 一致:
 *    V2(FRONT)=Laser1: PD=CH2,  LD1=CH0,  LD2=CH1,  电流系数 CURRENT_SENSE_GAIN_X1000
 *    H(SIDE)  =Laser2: PD=CH20, LD1=CH5,  LD2=CH6,  电流系数 CURRENT_SENSE_GAIN_X1000
 *    V1(HORIZ)=Laser3: PD=CH8,  LD1=CH19, LD2=CH7,  电流系数 CURRENT_SENSE_GAIN_X1000
 *
 *  非阻塞分步采样: 每次 1ms tick 调用一次, 每次只采一批(9通道), 不做软件延时。
 *  采满 PRE_EN1_SAMPLE_COUNT 批后打印一次, 并按三路电流是否超限返回:
 *    PRE_EN1_PASS (三路均 <= 阈值) -> 调用方拉 EN1;
 *    PRE_EN1_FAIL (任一路 > 阈值)   -> 调用方不拉 EN1。
 *
 *  注意: 之前阻塞版在 1ms tick 里 R_BSP_SoftwareDelay 死等 + 栈上累计器,
 *        会拖死主循环导致 WDT 复位, 故改为 static 累计 + 分步非阻塞。
 * ====================================================================== */
#define PRE_EN1_SAMPLE_COUNT  (8U)    /* 采样批次数 (8批 × 1ms = 8ms) */

static uint32_t s_pd1 = 0, s_ld1_1 = 0, s_ld2_1 = 0;
static uint32_t s_pd2 = 0, s_ld1_2 = 0, s_ld2_2 = 0;
static uint32_t s_pd3 = 0, s_ld1_3 = 0, s_ld2_3 = 0;
static uint16_t pre_en1_step = 0;

pre_en1_result_t laser_pre_en1_detect_step(void)
{
    uint16_t v = 0;

    /* 采样一批 (9 通道), 直接读数据寄存器, 非阻塞 */
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_2,  &v); s_pd1   += v;
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_0,  &v); s_ld1_1 += v;
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_1,  &v); s_ld2_1 += v;

    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_20, &v); s_pd2   += v;
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_5,  &v); s_ld1_2 += v;
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_6,  &v); s_ld2_2 += v;

    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_8,  &v); s_pd3   += v;
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_19, &v); s_ld1_3 += v;
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_7,  &v); s_ld2_3 += v;

    pre_en1_step++;
    if (pre_en1_step < PRE_EN1_SAMPLE_COUNT)
    {
        return PRE_EN1_NOT_DONE;   /* 未采满, 下个 tick 继续 */
    }

    pre_en1_step = 0;

    /* 平均 (raw ADC) → mV */
    //uint16_t pd1_mv   = (uint16_t)((s_pd1   / PRE_EN1_SAMPLE_COUNT) * 3300U / 4095U);
    uint16_t ld1_1_mv = (uint16_t)((s_ld1_1 / PRE_EN1_SAMPLE_COUNT) * 3300U / 4095U);
    uint16_t ld2_1_mv = (uint16_t)((s_ld2_1 / PRE_EN1_SAMPLE_COUNT) * 3300U / 4095U);

    //uint16_t pd2_mv   = (uint16_t)((s_pd2   / PRE_EN1_SAMPLE_COUNT) * 3300U / 4095U);
    uint16_t ld1_2_mv = (uint16_t)((s_ld1_2 / PRE_EN1_SAMPLE_COUNT) * 3300U / 4095U);
    uint16_t ld2_2_mv = (uint16_t)((s_ld2_2 / PRE_EN1_SAMPLE_COUNT) * 3300U / 4095U);

    //uint16_t pd3_mv   = (uint16_t)((s_pd3   / PRE_EN1_SAMPLE_COUNT) * 3300U / 4095U);
    uint16_t ld1_3_mv = (uint16_t)((s_ld1_3 / PRE_EN1_SAMPLE_COUNT) * 3300U / 4095U);
    uint16_t ld2_3_mv = (uint16_t)((s_ld2_3 / PRE_EN1_SAMPLE_COUNT) * 3300U / 4095U);

    /* 累加器清零, 供下次检测复用 */
    s_pd1 = 0; s_ld1_1 = 0; s_ld2_1 = 0;
    s_pd2 = 0; s_ld1_2 = 0; s_ld2_2 = 0;
    s_pd3 = 0; s_ld1_3 = 0; s_ld2_3 = 0;

    /* 电流 mA: 统一系数 CURRENT_SENSE_GAIN_X1000 (见 hawkeye_config.h) */
    int32_t d;
    d = (int32_t)ld1_1_mv - (int32_t)ld2_1_mv; if (d < 0) d = -d;
    int32_t cur_v2 = (d * CURRENT_SENSE_GAIN_X1000) / 1000;
    d = (int32_t)ld1_2_mv - (int32_t)ld2_2_mv; if (d < 0) d = -d;
    int32_t cur_h  = (d * CURRENT_SENSE_GAIN_X1000) / 1000;
    d = (int32_t)ld1_3_mv - (int32_t)ld2_3_mv; if (d < 0) d = -d;
    int32_t cur_v1 = (d * CURRENT_SENSE_GAIN_X1000) / 1000;

    /* 任一路电流超限 → 不拉 EN1 */
    bool over_limit = (cur_v2 > (int32_t)PRE_EN1_CURRENT_LIMIT_MA) ||
                      (cur_h  > (int32_t)PRE_EN1_CURRENT_LIMIT_MA) ||
                      (cur_v1 > (int32_t)PRE_EN1_CURRENT_LIMIT_MA);

/*    char msg[160];
    snprintf(msg, sizeof(msg),
             "+DBG:PRE_EN1 V2 pd=%dmV I=%dmA | H pd=%dmV I=%dmA | V1 pd=%dmV I=%dmA -> %s\r\n",
             (int)pd1_mv, (int)cur_v2,
             (int)pd2_mv, (int)cur_h,
             (int)pd3_mv, (int)cur_v1,
             over_limit ? "NO_EN1" : "EN1");
    uart9_send_blocking(msg);*/

    return over_limit ? PRE_EN1_FAIL : PRE_EN1_PASS;
}

/* NTC 低温标记 (NTC ADC > 3150 → <-7°C) */
volatile bool g_ntc_low_temp = false;

/* ===== 电池包 NTC 温度检测 ===== */
#define BAT_NTC_SAMPLE_CNT  (4U)    /* NTC计算累加次数 (4次=4秒) */

static uint32_t g_bat_ntc_vbatt_acc = 0;   /* vbatt 累加器 */
static uint32_t g_bat_ntc_th_acc    = 0;    /* th 累加器 */
static uint8_t  g_bat_ntc_sample_cnt = 0;   /* 采样计数 */

void battery_temp_check(void)
{
    uint16_t vbatt = (uint16_t)g_bat_adc_raw;
    uint16_t th    = (uint16_t)g_bat_battery_temp_raw;

    /* 累加采样 */
    g_bat_ntc_vbatt_acc += vbatt;
    g_bat_ntc_th_acc    += th;
    g_bat_ntc_sample_cnt++;

    if (g_bat_ntc_sample_cnt < BAT_NTC_SAMPLE_CNT) return;

    /* 多次采完, 求平均 */
    vbatt = (uint16_t)(g_bat_ntc_vbatt_acc / BAT_NTC_SAMPLE_CNT);
    th    = (uint16_t)(g_bat_ntc_th_acc    / BAT_NTC_SAMPLE_CNT);

    /* 重置累加器 */
    g_bat_ntc_vbatt_acc  = 0;
    g_bat_ntc_th_acc     = 0;
    g_bat_ntc_sample_cnt = 0;

    if (th == 0) return;  /* 防止除零 */

    /* NTC 电阻计算 (电池 → NTC → TH → R_fixed=8.25k → GND)
     * R_ntc(kΩ) = 15.49 × VBATT / TH - 8.25
     * 定点数 ×100: R_x100 = 1549 × VBATT / TH - 825 (单位 0.01kΩ) */
    int32_t r_x100 = (int32_t)4043 * (int32_t)vbatt / (int32_t)th - 2411;

    /* 电池包过热保护: R_ntc < 0.65kΩ 或 0.75kΩ < R_ntc < 2.30kΩ 触发关机 */
/*    {
        char tmp[48];
        snprintf(tmp, sizeof(tmp), "+DBG:BAT_NTC vbatt=%u th=%u R=%ld\r\n",
                 vbatt, th, (long)r_x100);
        uart9_send_blocking(tmp);
    }*/
    if ((r_x100 > 0 && r_x100 < 196))          //|| (r_x100 < 65))
    {
        //system_overheat_request(OVERHEAT_SOURCE_BATTERY);
    }
}

/* ===== 以下电池管理代码完全不变 ===== */
void battery_voltage_task(void)
{
    uint16_t bat_adc = Read_ADC_Voltage_Value_BAT();

    /* IR压降补偿: 激光开启时电池电压跌落, 补偿后状态机更准确 */
    {
        int32_t ir_comp = 0;
        if (gLaserOn[0]) ir_comp += BAT_IR_DROP_H_ADC;
        if (gLaserOn[1]) ir_comp += BAT_IR_DROP_V1_ADC;
        if (gLaserOn[2]) ir_comp += BAT_IR_DROP_V2_ADC;

        int32_t comp = (int32_t)bat_adc + ir_comp;
        if (comp > 4095) comp = 4095;
        else if (comp < 0) comp = 0;
        bat_adc = (uint16_t)comp;
    }

    bat_state_t new_state = g_bat_state;

    switch (g_bat_state)
    {
        case BAT_STATE_FULL:
            if (bat_adc < (BAT_HIGH_MV - BAT_HYST))
                new_state = BAT_STATE_HIGH;
            break;
        case BAT_STATE_HIGH:
            if      (bat_adc >= (BAT_HIGH_MV + BAT_HYST)) new_state = BAT_STATE_FULL;
            else if (bat_adc <  (BAT_MID_MV  - BAT_HYST)) new_state = BAT_STATE_MID;
            break;
        case BAT_STATE_MID:
            if      (bat_adc >= (BAT_MID_MV  + BAT_HYST)) new_state = BAT_STATE_HIGH;
            else if (bat_adc <   BAT_LOW_MV)               new_state = BAT_STATE_CRITICAL;
            break;
        case BAT_STATE_CRITICAL:
            if (bat_adc >= (BAT_LOW_MV + BAT_HYST))
                new_state = BAT_STATE_MID;
            break;
        default:
            new_state = BAT_STATE_CRITICAL;
            break;
    }

    g_bat_state = new_state;

    if (g_bat_state == BAT_STATE_CRITICAL)
    {
        if (g_bat_critical_count < BAT_CRITICAL_CONFIRM_COUNT)
            g_bat_critical_count++;
    }
    else
    {
        g_bat_critical_count   = 0;
        g_bat_shutdown_pending = false;
    }

    if (g_bat_critical_count >= BAT_CRITICAL_CONFIRM_COUNT)
        g_bat_shutdown_pending = true;
}

void battery_led_task(void)
{
    /* 锁定开关 HIGH 期间强制 LED 全关, 不按电池状态刷新 */
    if (g_lock_led_override)
    {
        led_set_pattern(LED_PATTERN_DEFAULT);
        return;
    }

    /* 重新锁定后的全亮展示期, 强制三颗全亮 */
    if (g_led_show_all)
    {
        led_set_pattern(LED_PATTERN_ALL_ON);
        return;
    }

    switch (g_bat_state)
    {
        case BAT_STATE_FULL:     led_set_pattern(LED_PATTERN_ALL_ON);  break;
        case BAT_STATE_HIGH:     led_set_pattern(LED_PATTERN_12_ON);   break;
        case BAT_STATE_MID:      led_set_pattern(LED_PATTERN_1_ON);    break;
        case BAT_STATE_CRITICAL:
            if (g_bat_shutdown_pending)
                system_power_off_request(POWER_OFF_REASON_BATTERY_LOW);
            break;
        default:
            led_set_pattern(LED_PATTERN_DEFAULT);
            break;
    }
}

static const char *power_off_reason_to_str(power_off_reason_t reason)
{
    switch (reason)
    {
        case POWER_OFF_REASON_BATTERY_LOW:      return "BATTERY_LOW";
        case POWER_OFF_REASON_LOCK_SW_RUNTIME:  return "LOCK_SW_RUNTIME";
        case POWER_OFF_REASON_I2C_FAIL:         return "I2C_FAIL";
        case POWER_OFF_REASON_OVERHEAT:         return "OVERHEAT";
        case POWER_OFF_REASON_IDLE_TIMEOUT:     return "IDLE_TIMEOUT";
        case POWER_OFF_REASON_LASER_HOTPLUG:
        default:                                return "LASER_HOTPLUG";
    }
}


void system_power_off_with_reason(power_off_reason_t reason)
{
    /* 在 SRAM 中写入关机魔数, Bootloader 检测后不会重新上电 */
    *SHUTDOWN_MAGIC_ADDR = SHUTDOWN_MAGIC_VALUE;

    char msg[64];
    snprintf(msg, sizeof(msg), "+RESP:POWEROFF=%s\r\n",
             power_off_reason_to_str(reason));
    uart9_send_blocking(msg);
    wdt_feed();
    flash_save_poweroff_and_stats((uint32_t)reason, stats_get_ptr());

    wdt_feed();
    Batt_OFF;
    Power_En_OFF;
    wdt_feed();
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    wdt_feed();
    set_laser1_200k_intensity(0, TIMER_PIN);
    set_laser2_200k_intensity(0, TIMER_PIN);
    set_laser3_200k_intensity(0, GPT_IO_PIN_GTIOCA);

    set_laser1_8470_intensity(0, GPT_IO_PIN_GTIOCA);
    set_laser2_8470_intensity(0, TIMER_PIN);
    set_laser3_8470_intensity(0, GPT_IO_PIN_GTIOCA);

    LED1_OFF;
    LED2_OFF;
    LED3_OFF;
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    wdt_feed();
    EN1_OFF;

    //NVIC_SystemReset();

    while (1)
    {
        Power_En_OFF;
        Batt_OFF;
        wdt_feed();
        __WFI();
    }
}



void system_power_off_request(power_off_reason_t reason)
{
    if (!g_system_power_off_pending)
    {
        g_system_power_off_pending  = true;
        g_system_power_off_reason   = reason;
    }
}

bool system_power_off_pending(void)
{
    return g_system_power_off_pending;
}

void system_power_off_process_pending(void)
{
    if (g_system_power_off_pending)
        system_power_off_with_reason(g_system_power_off_reason);
}

void system_power_off(void)
{
    system_power_off_request(POWER_OFF_REASON_LASER_HOTPLUG);
}

void system_power_on(void)
{
    Power_En_ON;
    Batt_ON;
}

void battery_state_init(uint16_t bat_adc)
{
    if      (bat_adc >= BAT_HIGH_MV) g_bat_state = BAT_STATE_FULL;
    else if (bat_adc >= BAT_MID_MV)  g_bat_state = BAT_STATE_HIGH;
    else if (bat_adc >= BAT_LOW_MV)  g_bat_state = BAT_STATE_MID;
    else                              g_bat_state = BAT_STATE_CRITICAL;

    if (bat_adc < BAT_LOW_MV)
        g_bat_critical_count = 1;
    else
        g_bat_critical_count = 0;

    g_bat_shutdown_pending = false;
}

bool battery_is_low_blink(void)
{
    return false;  /* LOW 状态已移除 */
}



