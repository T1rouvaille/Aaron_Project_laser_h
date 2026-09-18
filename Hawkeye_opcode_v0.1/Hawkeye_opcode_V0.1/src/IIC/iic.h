/*
 * iic.h
 *
 *  Created on: 2026年1月22日
 *      Author: AT403
 */

#ifndef IIC_IIC_H_
#define IIC_IIC_H_
#include <key/bsp_irq_key.h>
#include <led/bsp_led.h>
#include <adc/bsp_adc.h>
#include <agt/bsp_agt_timing.h>
#include <SysTick/bsp_SysTick.h>
#include "hal_data.h"
#include "debug_uart/bsp_debug_uart.h"
#include "common_utils.h"
#include "timer_pwm.h"
#include "wdt/wdt.h"
#include "hawkeye_config.h"

typedef enum {
    ANGLE_MODE_0_4   = 0,
    ANGLE_MODE_4_10  = 1,
    ANGLE_MODE_10_90 = 2,
    ANGLE_MODE_UNCAL = 3,
    ANGLE_MODE_OVERHEAT = 4,  /* ← 新增：高温保护，最高优先级 */
} angle_mode_t;

/* IMU 传感器类型 */
typedef enum {
    IMU_SENSOR_NONE        = 0,
    IMU_SENSOR_IIM42351    = 1,
    IMU_SENSOR_ICM40608    = 2,
    IMU_SENSOR_LSM6DSOTR   = 3,
} imu_sensor_t;

extern volatile angle_mode_t gAngleMode;

extern volatile bool g_calib_request;
extern uint16_t g_calib_cnt;
extern int32_t g_calib_sum[3];

/* 校准期间激光闪烁控制 */
extern volatile bool g_calib_laser_flash;   /* true=校准进行中, 激光闪烁 */
extern volatile bool g_calib_laser_on;      /* 当前闪烁相位: true=亮, false=暗 */

//---------calibration---------

extern int16_t imu_offset[3];
void IIM42351_calibration(void);


/* 你的原枚举可直接用 */
typedef enum
{
    IIM423XX_ACCEL_CONFIG0_ODR_500_HZ    = 0xF,
    IIM423XX_ACCEL_CONFIG0_ODR_1_5625_HZ = 0xE,
    IIM423XX_ACCEL_CONFIG0_ODR_3_125_HZ  = 0xD,
    IIM423XX_ACCEL_CONFIG0_ODR_6_25_HZ   = 0xC,
    IIM423XX_ACCEL_CONFIG0_ODR_12_5_HZ   = 0xB,
    IIM423XX_ACCEL_CONFIG0_ODR_25_HZ     = 0xA,
    IIM423XX_ACCEL_CONFIG0_ODR_50_HZ     = 0x9,
    IIM423XX_ACCEL_CONFIG0_ODR_100_HZ    = 0x8,
    IIM423XX_ACCEL_CONFIG0_ODR_200_HZ    = 0x7,
    IIM423XX_ACCEL_CONFIG0_ODR_1_KHZ     = 0x6,
    IIM423XX_ACCEL_CONFIG0_ODR_2_KHZ     = 0x5,
    IIM423XX_ACCEL_CONFIG0_ODR_4_KHZ     = 0x4,
    IIM423XX_ACCEL_CONFIG0_ODR_8_KHZ     = 0x3,
    IIM423XX_ACCEL_CONFIG0_ODR_16_KHZ    = 0x2,
    IIM423XX_ACCEL_CONFIG0_ODR_32_KHZ    = 0x1,
} iim42351_odr_t;

/* ===== 角度阈值 ===== */


//-----
void posture_update(int16_t *acc);

fsp_err_t i2c_master_init(void);
fsp_err_t iim42351_read_reg(uint8_t reg, uint8_t *data);
fsp_err_t iim42351_write_reg(uint8_t reg, uint8_t value);
fsp_err_t iim42351_read_bytes(uint8_t reg, uint8_t* buf, uint8_t len);
int8_t IIM42351_getAcc(int16_t* acc);

/* 统一 IMU 接口: 自动检测 IIM42351/ICM40608/LSM6DSOTR */
int8_t imu_init(uint8_t highSampleRate);
int8_t imu_get_acc(int16_t* acc);

/* IMU 角度调试打印开关: AT+IMUDBG=1 开启, AT+IMUDBG=0 关闭 */
extern volatile bool g_imu_debug_enabled;

#endif /* IIC_IIC_H_ */
