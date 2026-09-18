/**
 * @file    power_calibration.h
 * @brief   光功率自动校准模块
 *
 * PC 上位机读取光功率计 → 发 AT+POWCAL=<CH>,<power> → MCU 调整参考电压
 * 三路独立校准，互不干扰，调节完毕后可保存到 Flash。
 *
 * 通道映射:  H(SIDE) / V1(HORIZ) / V2(FRONT)
 * 目标功率:  H=2.00mW, V1=1.00mW, V2=1.00mW
 */

#ifndef POWER_CALIBRATION_H_
#define POWER_CALIBRATION_H_

#include <stdbool.h>
#include <stdint.h>
#include "hawkeye_config.h"

/* ======================================================================
 *  通道定义
 * ====================================================================== */
typedef enum {
    POWCAL_CH_H  = 0,   /* SIDE  — 侧面激光, 目标 2.00mW */
    POWCAL_CH_V1 = 1,   /* HORIZ — 水平激光, 目标 1.00mW */
    POWCAL_CH_V2 = 2,   /* FRONT — 前方激光, 目标 1.00mW */
    POWCAL_CH_NUM = 3
} powcal_channel_t;

/* ======================================================================
 *  校准状态
 * ====================================================================== */
typedef enum {
    POWCAL_STAT_ADJ = 0,        /* 已调整，等待下次测量 */
    POWCAL_STAT_DONE,           /* 到位, power>=target 且误差在+0.05mW内 */
    POWCAL_STAT_SAT,            /* 饱和，已达电压上限 */
    POWCAL_STAT_IDLE,           /* 空闲，未启动校准 */
    POWCAL_STAT_LASER_OFF,      /* 保护: 激光未打开 */
    POWCAL_STAT_PD_LOW,         /* 保护: PD 初始值过低 (<50mV) */
    POWCAL_STAT_NO_RESP,        /* 保护: 电压升高但光功率未响应 */
} powcal_status_t;

/* ======================================================================
 *  通道名称字符串
 * ====================================================================== */
extern const char * const powcal_ch_names[POWCAL_CH_NUM];

/* 目标光功率表 (x100, 单位 0.01mW)，索引对应 powcal_channel_t */
extern const int powcal_targets[POWCAL_CH_NUM];

/* ======================================================================
 *  公开 API
 * ====================================================================== */

/**
 * @brief 处理 PC 发来的光功率读数，自动调整参考电压
 *
 * @param ch    通道 (0=H, 1=V1, 2=V2)
 * @param power 光功率值 (x100, 单位 0.01mW)，如 200 = 2.00mW
 * @return      校准状态 (ADJ=继续 / DONE=到位 / SAT=饱和)
 */
powcal_status_t powcal_process(powcal_channel_t ch, int power);

/**
 * @brief 保存全部三路校准结果到 Flash
 * @return true=成功, false=失败
 */
bool powcal_save(void);

/**
 * @brief 查询某通道当前参考电压 (mV)
 * @param ch 通道
 * @return   参考电压值 (mV)
 */
int powcal_get_ref_mv(powcal_channel_t ch);

/**
 * @brief 查询某通道当前状态
 * @param ch 通道
 * @return   校准状态
 */
powcal_status_t powcal_get_status(powcal_channel_t ch);

/**
 * @brief 重置某通道校准状态 (终止校准)
 * @param ch 通道
 */
void powcal_reset(powcal_channel_t ch);

#endif /* POWER_CALIBRATION_H_ */
