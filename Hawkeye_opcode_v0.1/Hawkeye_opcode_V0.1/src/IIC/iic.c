/*
 * iic.c
 *
 *  Created on: 2026�?�?2�?
 *      Author: AT403
 */

#ifndef IIC_IIC_C_
#define IIC_IIC_C_
#include <key/bsp_irq_key.h>
#include <led/bsp_led.h>
#include <adc/bsp_adc.h>
#include <agt/bsp_agt_timing.h>
#include <SysTick/bsp_SysTick.h>
#include <iic/iic.h>
#include "hal_data.h"
#include "debug_uart/bsp_debug_uart.h"
#include "common_utils.h"
#include "timer_pwm.h"
#include "flash/flash.h"
#include "wdt/wdt.h"
#include "hawkeye_config.h"
volatile bool g_calib_request = false;
uint16_t g_calib_cnt = 0;
int32_t g_calib_sum[3] = {0, 0, 0};

volatile bool g_imu_debug_enabled = false;   /* AT+IMUDBG=1 开启实时角度打印 */

/* 校准期间激光闪烁 */
volatile bool g_calib_laser_flash = false;  /* true=校准进行中, 激光闪烁 */
volatile bool g_calib_laser_on = false;     /* 当前闪烁相位 */

volatile angle_mode_t gAngleMode = ANGLE_MODE_0_4;

/* IIM42351 寄存器地址 */
#define MPUREG_ACCEL_DATA_X1_H   0x1F
#define MPUREG_ACCEL_CONFIG0     0x50
#define MPUREG_PWR_MGMT_0        0x4E
#define MPUREG_WHO_AM_I          0x75
#define IIM_WHO_AM_I_DEFAULT     0x6C
#define ICM_WHO_AM_I_DEFAULT     0x39

/* LSM6DSOTR 寄存器地址 */
#define LSM6DSO_WHO_AM_I         0x0F
#define LSM6DSO_WHO_AM_I_VAL     0x6C
#define LSM6DSO_CTRL1_XL         0x10   /* 加速度计控制: ODR + FS + LPF2 */
#define LSM6DSO_CTRL2_G          0x11   /* 陀螺仪控制 (本应用关闭) */
#define LSM6DSO_CTRL3_C          0x12   /* 主控制: BDU, IF_INC, SW_RESET 等 */
#define LSM6DSO_CTRL6_C          0x15   /* 加速度计功耗模式: XL_HM_MODE */
#define LSM6DSO_OUTX_L_A         0x28   /* 加速度 X 低字节 (Little Endian) */

/* LSM6DSOTR CTRL1_XL 配置位 */
#define LSM6DSO_ODR_12_5HZ       0x10   /* 12.5 Hz (与 IIM42351 低频一致) */
#define LSM6DSO_ODR_104HZ        0x40   /* 104 Hz  (最接近 IIM42351 100Hz) */
#define LSM6DSO_FS_XL_2G         0x00   /* ±2g */

/* LSM6DSOTR CTRL3_C 配置: 使能 BDU (Block Data Update) */
#define LSM6DSO_BDU              0x40
#define LSM6DSO_IF_INC           0x04   /* 寄存器地址自动递增 */

/* ODR + 量程 */
#define IIM423XX_ACCEL_CONFIG0_FS_SEL_2g    (0x3 << 5)
#define IIM423XX_ACCEL_CONFIG0_ODR_12_5_HZ  0xB
#define IIM423XX_ACCEL_CONFIG0_ODR_50_HZ    0x9
#define IIM423XX_ACCEL_CONFIG0_ODR_100_HZ   0x8  /*!< 100 Hz (10 ms)*/
/* 低噪模式 */
#define BIT_PWR_MGMT_0_ACCEL_MODE_MASK   0x03
#define IIM423XX_PWR_MGMT_0_ACCEL_MODE_LN  0x03

/* ========== 全局变量 ========== */
static volatile bool g_i2c_done = false;
#define I2C_WAIT_TIMEOUT_LOOP   (30000UL)

int16_t imu_offset[3] = {0, 0, 0};
static imu_sensor_t g_imu_sensor = IMU_SENSOR_NONE;  /* 当前检测到的传感器类型 */


static angle_mode_t gPendingMode = ANGLE_MODE_4_10;  // 待切换的模式
static uint16_t mode_change_counter = 0;              // 模式切换计数�?


/* ========== I2C 回调 ========== */
void i2c_callback(i2c_master_callback_args_t *p_args)
{
    if ((p_args->event == I2C_MASTER_EVENT_TX_COMPLETE) ||
        (p_args->event == I2C_MASTER_EVENT_RX_COMPLETE) ||
        (p_args->event == I2C_MASTER_EVENT_ABORTED))
    {
        g_i2c_done = true;
    }
}

/* ========== I2C 初始�?========== */
fsp_err_t i2c_master_init(void)
{
    return R_IIC_MASTER_Open(&g_i2c_master0_ctrl, &g_i2c_master0_cfg);
}


/* ========== IIM42351 读单寄存�?========== */
fsp_err_t iim42351_read_reg(uint8_t reg, uint8_t *data)
{
    fsp_err_t err;
    uint32_t timeout;

    g_i2c_done = false;
    err = R_IIC_MASTER_Write(&g_i2c_master0_ctrl, &reg, 1, true);
    if (err != FSP_SUCCESS) return err;
    timeout = I2C_WAIT_TIMEOUT_LOOP;
    while (!g_i2c_done && (timeout > 0U)) { timeout--; wdt_feed(); }
    if (0U == timeout) return FSP_ERR_TIMEOUT;

    g_i2c_done = false;
    err = R_IIC_MASTER_Read(&g_i2c_master0_ctrl, data, 1, false);
    if (err != FSP_SUCCESS) return err;
    timeout = I2C_WAIT_TIMEOUT_LOOP;
    while (!g_i2c_done && (timeout > 0U)) { timeout--; wdt_feed(); }
    if (0U == timeout) return FSP_ERR_TIMEOUT;

    return FSP_SUCCESS;
}

/* ========== IIM42351 写寄存器 ========== */
fsp_err_t iim42351_write_reg(uint8_t reg, uint8_t value)
{
    fsp_err_t err;
    uint8_t buf[2];
    uint32_t timeout;

    buf[0] = reg;
    buf[1] = value;

    g_i2c_done = false;
    err = R_IIC_MASTER_Write(&g_i2c_master0_ctrl, buf, 2, false);
    if (err != FSP_SUCCESS) return err;
    timeout = I2C_WAIT_TIMEOUT_LOOP;
    while (!g_i2c_done && (timeout > 0U)) { timeout--; wdt_feed(); }
    if (0U == timeout) return FSP_ERR_TIMEOUT;

    return FSP_SUCCESS;
}

/* ========== IIM42351 连续  n 字节 ========== */
fsp_err_t iim42351_read_bytes(uint8_t reg, uint8_t* buf, uint8_t len)
{
    fsp_err_t err;
    uint32_t timeout;

    g_i2c_done = false;
    err = R_IIC_MASTER_Write(&g_i2c_master0_ctrl, &reg, 1, true);
    if (err != FSP_SUCCESS) return err;
    timeout = I2C_WAIT_TIMEOUT_LOOP;
    while (!g_i2c_done && (timeout > 0U)) { timeout--; wdt_feed(); }
    if (0U == timeout) return FSP_ERR_TIMEOUT;

    g_i2c_done = false;
    err = R_IIC_MASTER_Read(&g_i2c_master0_ctrl, buf, len, false);
    if (err != FSP_SUCCESS) return err;
    timeout = I2C_WAIT_TIMEOUT_LOOP;
    while (!g_i2c_done && (timeout > 0U)) { timeout--; wdt_feed(); }
    if (0U == timeout) return FSP_ERR_TIMEOUT;

    return FSP_SUCCESS;
}

/* ========== 通用 IMU 初始化 (IIM42351/ICM40608 共用) ========== */
static int8_t imu_chip_init(uint8_t highSampleRate, uint8_t who_am_i)
{
    fsp_err_t err;
    uint8_t id = 0;

    /* WHO_AM_I 检测 (静默探测, 不在探测阶段打印 NOT FOUND) */
    err = iim42351_read_reg(MPUREG_WHO_AM_I, &id);
    if (err != FSP_SUCCESS || id != who_am_i)
    {
        return -1;
    }

    /* 配置加速度 ODR + 2g FS */
    uint8_t data = IIM423XX_ACCEL_CONFIG0_FS_SEL_2g;
    if (highSampleRate)
        data |= IIM423XX_ACCEL_CONFIG0_ODR_100_HZ;
    else
        data |= IIM423XX_ACCEL_CONFIG0_ODR_12_5_HZ;

    err = iim42351_write_reg(MPUREG_ACCEL_CONFIG0, data);
    if (err != FSP_SUCCESS)
    {
        uart9_send_blocking("+RESP:SET ACCEL_CONFIG0 FAIL\r\n");
        return -1;
    }

    /* 配置低噪模式 */
    err = iim42351_write_reg(MPUREG_PWR_MGMT_0, IIM423XX_PWR_MGMT_0_ACCEL_MODE_LN);
    if (err != FSP_SUCCESS)
    {
        uart9_send_blocking("+RESP:SET PWR_MGMT_0 FAIL\r\n");
        return -1;
    }

    return 0;
}

/* ========== 切换 I2C 从机地址 (用于 LSM6DSOTR 探测) ========== */
static void i2c_set_slave_addr(uint8_t addr)
{
    /* FSP HAL 每次 Read/Write 时从 ctrl.slave 写入硬件寄存器,
     * 直接修改即可生效, 无需 Close/Open (避免重新初始化破坏总线状态) */
    g_i2c_master0_ctrl.slave = addr;
}

/* ========== LSM6DSOTR 芯片初始化 (仅加速度计) ========== */
static int8_t lsm6dsotr_chip_init(uint8_t highSampleRate)
{
    fsp_err_t err;
    uint8_t id = 0;
    uint8_t addr_idx;
    /* 尝试 SA0=GND(0x6A) 和 SA0=VCC(0x6B) 两个 I2C 地址 */
    static const uint8_t lsm_addrs[] = {0x6A, 0x6B};

    for (addr_idx = 0; addr_idx < 2; addr_idx++)
    {
        /* 切换 I2C 地址 (同时 Reset 总线清除错误状态) */
        i2c_set_slave_addr(lsm_addrs[addr_idx]);

        /* WHO_AM_I 检测 (LSM6DSOTR 的 WHO_AM_I 在 0x0F) */
        err = iim42351_read_reg(LSM6DSO_WHO_AM_I, &id);
        if (err == FSP_SUCCESS && id == LSM6DSO_WHO_AM_I_VAL)
        {
            break;  /* 找到了 */
        }
    }

    if (addr_idx >= 2)
    {
        /* 两个地址都没找到, 恢复默认 I2C 地址 */
        i2c_set_slave_addr(0x69);
        return -1;
    }

    /* ===== 配置 CTRL1_XL: ODR + ±2g FS ===== */
    uint8_t data = LSM6DSO_FS_XL_2G;
    if (highSampleRate)
        data |= LSM6DSO_ODR_104HZ;    /* 104 Hz ≈ IIM42351 的 100 Hz */
    else
        data |= LSM6DSO_ODR_12_5HZ;   /* 12.5 Hz (与 IIM42351 一致) */

    err = iim42351_write_reg(LSM6DSO_CTRL1_XL, data);
    if (err != FSP_SUCCESS)
    {
        uart9_send_blocking("+RESP:SET CTRL1_XL FAIL\r\n");
        return -1;
    }

    /* ===== 配置 CTRL2_G: 陀螺仪断电 (仅用加速度计) ===== */
    err = iim42351_write_reg(LSM6DSO_CTRL2_G, 0x00);
    if (err != FSP_SUCCESS)
    {
        uart9_send_blocking("+RESP:SET CTRL2_G FAIL\r\n");
        return -1;
    }

    /* ===== 配置 CTRL3_C: 使能 BDU (Block Data Update) =====
     * BDU=1 时, 先读低字节会锁定高字节, 直到高字节被读取,
     * 保证高低字节来自同一采样时刻, 避免数据撕裂 */
    err = iim42351_write_reg(LSM6DSO_CTRL3_C, LSM6DSO_BDU | LSM6DSO_IF_INC);
    if (err != FSP_SUCCESS)
    {
        uart9_send_blocking("+RESP:SET CTRL3_C FAIL\r\n");
        return -1;
    }

    /* ===== 配置 CTRL6_C: High Performance 模式 =====
     * XL_HM_MODE=0 → High Performance (与 IIM42351 Low Noise 对应) */
    err = iim42351_write_reg(LSM6DSO_CTRL6_C, 0x00);
    if (err != FSP_SUCCESS)
    {
        uart9_send_blocking("+RESP:SET CTRL6_C FAIL\r\n");
        return -1;
    }

    return 0;
}

/* ========== LSM6DSOTR 读取加速度 (Little Endian: 低字节在前) ========== */
static int8_t lsm6dsotr_get_acc(int16_t* acc)
{
    uint8_t buf[6];
    fsp_err_t err;
    uint8_t i;

    err = iim42351_read_bytes(LSM6DSO_OUTX_L_A, buf, 6);
    if (err != FSP_SUCCESS)
    {
        acc[0] = 0;
        acc[1] = 0;
        acc[2] = 0;
        return -1;
    }

    /* LSM6DSOTR 数据格式: Little Endian (低字节在前)
     * buf[0]=X_L, buf[1]=X_H → (X_H << 8) | X_L */
    for (i = 0; i < 3; i++)
    {
        acc[i] = (int16_t)((buf[2*i + 1] << 8) | buf[2*i]);
    }

    return 0;
}

/* ========== 读取加速度 X/Y/Z (IIM42351/ICM40608 共用) ========== */
int8_t IIM42351_getAcc(int16_t* acc)
{
    uint8_t buf[6];
    fsp_err_t err;
    uint8_t i;

    err = iim42351_read_bytes(MPUREG_ACCEL_DATA_X1_H, buf, 6);
    if (err != FSP_SUCCESS)
    {
        acc[0] = 0;
        acc[1] = 0;
        acc[2] = 0;
        return -1;
    }

    for (i = 0; i < 3; i++)
    {
        acc[i] = (int16_t)((buf[2*i] << 8) | buf[2*i + 1]);
    }

    return 0;
}

/* ========== 统一 IMU 初始化: 自动检测传感器类型 ========== */
int8_t imu_init(uint8_t highSampleRate)
{
    /* 1. 尝试 IIM42351 (I2C 地址 0x69) */
    if (imu_chip_init(highSampleRate, IIM_WHO_AM_I_DEFAULT) == 0)
    {
        g_imu_sensor = IMU_SENSOR_IIM42351;
        uart9_send_blocking("+RESP:IMU IIM42351\r\n");
        return 0;
    }
    wdt_feed();  /* 传感器探测失败后喂狗, 防止连续I2C超时触发WDT */

    /* 2. 尝试 ICM40608 (I2C 地址 0x69, 寄存器兼容) */
    if (imu_chip_init(highSampleRate, ICM_WHO_AM_I_DEFAULT) == 0)
    {
        g_imu_sensor = IMU_SENSOR_ICM40608;
        uart9_send_blocking("+RESP:IMU ICM40608\r\n");
        return 0;
    }
    wdt_feed();  /* 传感器探测失败后喂狗, 防止连续I2C超时触发WDT */

    /* 3. 尝试 LSM6DSOTR (I2C 地址 0x6A, 独立寄存器映射) */
    if (lsm6dsotr_chip_init(highSampleRate) == 0)
    {
        g_imu_sensor = IMU_SENSOR_LSM6DSOTR;
        uart9_send_blocking("+RESP:IMU LSM6DSOTR\r\n");
        return 0;
    }

    /* 所有传感器均未检测到, 恢复默认 I2C 地址 */
    i2c_set_slave_addr(0x69);
    g_imu_sensor = IMU_SENSOR_NONE;
    uart9_send_blocking("+RESP:IMU NOT FOUND\r\n");
    return -1;
}

/* ========== 统一读取加速度 (根据传感器类型分发) ========== */
int8_t imu_get_acc(int16_t* acc)
{
    if (g_imu_sensor == IMU_SENSOR_NONE)
    {
        acc[0] = 0;
        acc[1] = 0;
        acc[2] = 0;
        return -1;
    }

    /* LSM6DSOTR 数据格式不同 (Little Endian), 使用独立读取函数 */
    if (g_imu_sensor == IMU_SENSOR_LSM6DSOTR)
    {
        return lsm6dsotr_get_acc(acc);
    }

    return IIM42351_getAcc(acc);
}

void IIM42351_calibration(void)
{
    int16_t acc[3];
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint16_t i;
    uint8_t valid_cnt = 0;

    uart9_send_blocking("+RESP:CALIBRATION START\r\n");
    wdt_feed();

    for (i = 0; i < 8; i++)
    {
        wdt_feed();

        if (imu_get_acc(acc) != 0)
        {
            wdt_feed();
            continue;  // 读取失败跳过，不中断整个校准
        }

        sum_x += acc[0];
        sum_y += acc[1];
        sum_z += acc[2];
        valid_cnt++;
        wdt_feed();
    }

    if (valid_cnt == 0)
    {
        uart9_send_blocking("+RESP:CALIBRATION FAIL\r\n");
        return;
    }

    imu_offset[0] = (int16_t)(sum_x / valid_cnt);
    imu_offset[1] = (int16_t)(sum_y / valid_cnt);
    imu_offset[2] = (int16_t)(sum_z / valid_cnt);

    wdt_feed();

    char msg[64];
    sprintf(msg, "+RESP:CALIB DONE X=%d Y=%d Z=%d\r\n",
            imu_offset[0], imu_offset[1], imu_offset[2]);
    uart9_send_blocking(msg);
    wdt_feed();

    flash_save_calibration(imu_offset[0], imu_offset[1], imu_offset[2]);
    wdt_feed();
}

void posture_update(int16_t *acc)
{

    if (gAngleMode == ANGLE_MODE_OVERHEAT) return;  /* 高温期间禁止切换 */

    if ((imu_offset[0] == 0) && (imu_offset[1] == 0) && (imu_offset[2] == 0))
    {
        gAngleMode = ANGLE_MODE_UNCAL;
        return;
    }

    int32_t x_val    = acc[0];
    int32_t y_val = abs((int32_t)acc[1]);

    int32_t z_raw = (int32_t)acc[2];
//    z_raw = z_raw + Y_AXIS_OFFSET_COMP;  /* 直接减，正负都适用 */
    int32_t z_val = abs(z_raw);

    unsigned int isUpright = (x_val > 0) ? 1U : 0U;

    int32_t YZ_angle = (y_val > z_val) ? y_val : z_val;

    angle_mode_t targetMode = gAngleMode;  // 目标模式

    /* ===== 根据当前状态判断目标模�?===== */
    switch (gAngleMode)
    {
        case ANGLE_MODE_0_4:
        {
            if (!isUpright)
            {
                targetMode = ANGLE_MODE_10_90;
            }
            else if (YZ_angle < LIMIT_4_DEG)
            {
                targetMode = ANGLE_MODE_0_4;  // 保持
            }
            else if (YZ_angle < LIMIT_10_DEG)
            {
                targetMode = ANGLE_MODE_4_10;
            }
            else
            {
                targetMode = ANGLE_MODE_10_90;
            }
        }
        break;

        case ANGLE_MODE_4_10:
        {
            if (!isUpright)
            {
                targetMode = ANGLE_MODE_10_90;
            }
            else if (YZ_angle < LIMIT_3_8_DEG)
            {
                targetMode = ANGLE_MODE_0_4;
            }
            else if (YZ_angle < LIMIT_10_DEG)
            {
                targetMode = ANGLE_MODE_4_10;  // 保持
            }
            else
            {
                targetMode = ANGLE_MODE_10_90;
            }
        }
        break;

        case ANGLE_MODE_10_90:
        {
            if (!isUpright)
            {
                targetMode = ANGLE_MODE_10_90;  // 保持倒置
            }
            else if (YZ_angle < LIMIT_3_8_DEG)
            {
                targetMode = ANGLE_MODE_0_4;
            }
            else if (YZ_angle < LIMIT_7_5_DEG)
            {
                targetMode = ANGLE_MODE_4_10;
            }
            else
            {
                targetMode = ANGLE_MODE_10_90;  // 保持
            }
        }
        break;

        case ANGLE_MODE_UNCAL:
        {
            targetMode = ANGLE_MODE_4_10;
        }
        break;

        default:
        {
            targetMode = ANGLE_MODE_4_10;
        }
        break;
    }

    /* ===== 防抖逻辑：需要连续N次相同才切换 ===== */
    if (targetMode != gAngleMode)
    {
        // 目标模式改变�?
        if (targetMode == gPendingMode)
        {
            // 和上次的待切换模式一�?计数�?1
            mode_change_counter++;

            if (mode_change_counter >= MODE_CHANGE_THRESHOLD)
            {
                // 达到阈�?真正切换模式
                gAngleMode = targetMode;
                mode_change_counter = 0;

            }
        }
        else
        {
            // 目标模式又变�?重新开始计�?
            gPendingMode = targetMode;
            mode_change_counter = 1;
        }
    }
    else
    {
        // 目标模式就是当前模式,清除待切换状�?
        mode_change_counter = 0;
        gPendingMode = gAngleMode;
    }
}
#endif /* IIC_IIC_C_ */



