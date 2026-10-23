#ifndef FLASH_FLASH_H_
#define FLASH_FLASH_H_

#include "hal_data.h"
#include "stdint.h"
#include "stdbool.h"
#include "hawkeye_config.h"

/* ======================================================================
 *  RA2E1 Data Flash 参数（与 hawkeye_config.h 保持一致）
 * ====================================================================== */
#define FLASH_DF_BASE_ADDR       (0x40100000U)
#define FLASH_DF_BLOCK_SIZE      (64U)
#define FLASH_DF_WRITE_SIZE      (4U)

#define FLASH_BLOCK_CALIBRATION  (FLASH_DF_BASE_ADDR)

/* ======================================================================
 *  平台化 Datalog 布局（对齐 Stepper_Motor_Control_V0.1.xlsx 标准）
 *
 *  Data Flash 共 6 块（384 字节）：
 *    块 0~4（offset 0~319）  = 标准 datalog（320 字节），boot 'F' 读取
 *    块 5（offset 320~383）  = flash_devinfo_t（版本/设备信息，仅 APP 内部使用）
 *
 *  320 字节标准 datalog 布局（基地址 0x40100000）：
 *  -------------------------------------------------------------------
 *   offset   大小   字段（Stepper 标准）          Hawkeye 映射
 *  -------------------------------------------------------------------
 *   0        4      Header magic（格式标识）
 *   4        8      保留（原校准 X/Y/Z 已移至 244）
 *   12       4      校准请求 flag（UBOX 写入, 固定不可移动）
 *   16       8      保留
 *   24..51   28     Protection[14] (uint16×14)    关机原因 → 对应字段
 *   52..203  152    Time[38] (uint32×38)          激光/角度运行时长
 *   204..231 28     Cycle[7] (uint32×7)           激光按键次数
 *   232..243 12     Optical[3] (int32×3)          光功率
 *   244..261 18     Offset: MEMS X/Y/Z (int16) +   校准偏移
 *                   Reset (uint32) + Model (uint8[8])
 *   262..269 8      Reference[4] (uint16×4)       参考电压
 *   270..271 2      MEMS drop record (uint16)
 *   272..277 6      temp[3] (uint16×3)
 *   278..281 4      Bluetooth[2] (uint16×2)
 *   282..301 20     Run Time[5] (uint32×5)
 *   302..319 18     保留
 *  -------------------------------------------------------------------
 * ====================================================================== */
#define FLASH_DATALOG_BLOCKS     (5U)                                    /* datalog 5 块 */
#define FLASH_DATALOG_SIZE       (FLASH_DF_BLOCK_SIZE * FLASH_DATALOG_BLOCKS)  /* 320 字节 */
#define FLASH_TOTAL_BLOCKS       (6U)                                    /* + 1 块 devinfo */
#define FLASH_TOTAL_SIZE         (FLASH_DF_BLOCK_SIZE * FLASH_TOTAL_BLOCKS)    /* 384 字节 */
#define FLASH_OFFSET_DEVINFO     (FLASH_DATALOG_SIZE)                    /* 320, 块5 */

/* ======================================================================
 *  魔数
 * ====================================================================== */
#define FLASH_DATALOG_MAGIC      (0xA5A5A5A5U)   /* 平台 datalog 头部格式标识 (offset 0) */
#define FLASH_STATS_MAGIC        (0xB6B6B6B6U)   /* RAM 累加器有效标识 */
#define FLASH_POWEROFF_MAGIC     (0xC3C3C3C3U)   /* 关机日志有效标识 (RAM) */
#define FLASH_DEVINFO_MAGIC      (0xD4D4D4D4U)   /* 设备信息有效标识 */

/* ======================================================================
 *  校准请求 flag (offset 12, 4 bytes) — 固定不可移动
 *  UBOX 收到 Honcho 校准命令后写入, APP 启动时检测
 * ====================================================================== */
#define FLASH_OFFSET_CALIB_REQ   (12U)
#define FLASH_CALIB_REQ_ADDR     (FLASH_BLOCK_CALIBRATION + FLASH_OFFSET_CALIB_REQ)
#define FLASH_CALIB_REQ_MAGIC    (0xCA11B000U)    /* UBOX 写入的校准请求魔数 */

#define FLASH_OFFSET_HDR_MAGIC   (0U)             /* 头部格式标识 */
#define FLASH_OFFSET_HDR_TOTAL_RUN (4U)           /* 设备总运行时长 total_run_s (uint32) */

/* ======================================================================
 *  Protection 段 (offset 24..51, uint16 × 14) — Stepper 标准保护/关机计数
 * ====================================================================== */
#define FLASH_OFFSET_PROT_BASE        (24U)
#define FLASH_PROT_NTC_HOT_LASER      (24U)   /* NTC 内腔(控制板) >80° */
#define FLASH_PROT_NTC_COLD_LASER     (26U)   /* NTC 内腔 < -7° */
#define FLASH_PROT_NTC_FROZEN_PACK    (28U)   /* 电池 < 0° */
#define FLASH_PROT_NTC_HOT_PACK       (30U)   /* 电池 > 70° shutdown */
#define FLASH_PROT_BATTERY_UV         (32U)   /* 电池欠压 */
#define FLASH_PROT_MAIN_SW_OFF        (34U)   /* 主开关关闭 */
#define FLASH_PROT_MEMS_ERROR         (36U)   /* MEMS 未实装/通信不良 (I2C 芯片丢失) */
#define FLASH_PROT_IDLE_TIMEOUT       (38U)   /* 空闲超时 */
#define FLASH_PROT_H1_OVERCUR         (40U)
#define FLASH_PROT_H2_OVERCUR         (42U)
#define FLASH_PROT_V1_OVERCUR         (44U)
#define FLASH_PROT_V2_OVERCUR         (46U)
#define FLASH_PROT_BUCK_UPPER         (48U)
#define FLASH_PROT_BUCK_LOWER         (50U)

/* ======================================================================
 *  Time 段 (offset 52..203, uint32 × 38) — Stepper 标准运行时长
 *  Hawkeye 仅写入单/组合激光 + 角度(tilt) 字段，其余保持 0
 * ====================================================================== */
#define FLASH_OFFSET_TIME_BASE        (52U)
#define FLASH_TIME_H1                 (52U)   /* Laser H1 runtime  ← H 单独 */
#define FLASH_TIME_H2                 (56U)
#define FLASH_TIME_V1                 (60U)   /* Laser V1 runtime  ← V1 单独 */
#define FLASH_TIME_V2                 (64U)   /* Laser V2 runtime  ← V2 单独 */
#define FLASH_TIME_H1H2               (68U)
#define FLASH_TIME_H1V1               (72U)   /* H1&V1 ← v1_h_only_s */
#define FLASH_TIME_H1V2               (76U)   /* H1&V2 ← v2_h_only_s */
#define FLASH_TIME_H2V1               (80U)
#define FLASH_TIME_H2V2               (84U)
#define FLASH_TIME_V1V2               (88U)   /* V1&V2 ← v1_v2_only_s */
#define FLASH_TIME_H1H2V1             (92U)
#define FLASH_TIME_H1H2V2             (96U)
#define FLASH_TIME_H1V1V2             (100U)  /* H1&V1&V2 ← all3_laser_s */
#define FLASH_TIME_H2V1V2             (104U)
#define FLASH_TIME_H1H2V1V2           (108U)
/* Slope 段 (112..171) — 激光组合 × (倾角 > 4°) 交叉时长
 * Hawkeye 仅一个 H 水平激光 → 7 个非 H2 组合有效, 其余(H2 相关)保持 0 */
#define FLASH_SLOPE_H1                (112U)  /* Slope H1       ← slope_h_s */
#define FLASH_SLOPE_H2                (116U)  /* H2 未使用, 保持 0 */
#define FLASH_SLOPE_V1                (120U)  /* Slope V1       ← slope_v1_s */
#define FLASH_SLOPE_V2                (124U)  /* Slope V2       ← slope_v2_s */
#define FLASH_SLOPE_H1H2              (128U)  /* 未使用, 保持 0 */
#define FLASH_SLOPE_H1V1              (132U)  /* Slope H1&V1    ← slope_v1_h_s */
#define FLASH_SLOPE_H1V2              (136U)  /* Slope H1&V2    ← slope_v2_h_s */
#define FLASH_SLOPE_H2V1              (140U)  /* 未使用, 保持 0 */
#define FLASH_SLOPE_H2V2              (144U)  /* 未使用, 保持 0 */
#define FLASH_SLOPE_V1V2              (148U)  /* Slope V1&V2    ← slope_v1_v2_s */
#define FLASH_SLOPE_H1H2V1            (152U)  /* 未使用, 保持 0 */
#define FLASH_SLOPE_H1H2V2            (156U)  /* 未使用, 保持 0 */
#define FLASH_SLOPE_H1V1V2            (160U)  /* Slope H1&V1&V2 ← slope_all3_s */
#define FLASH_SLOPE_H2V1V2            (164U)  /* 未使用, 保持 0 */
#define FLASH_SLOPE_H1H2V1V2          (168U)  /* 未使用, 保持 0 */
#define FLASH_TIME_TILT_0_4           (172U)  /* tilt < 4°  ← angle_0_4_s */
#define FLASH_TIME_TILT_4_10          (176U)  /* 4~10°     ← angle_4_10_s */
#define FLASH_TIME_TILT_10_90         (180U)  /* > 10°     ← angle_10_90_s */
/* 马达/亮度段 (184..203) — Hawkeye 无马达/亮度，保持 0 */

/* ======================================================================
 *  Cycle 段 (offset 204..231, uint32 × 7) — Stepper 标准开关次数
 * ====================================================================== */
#define FLASH_OFFSET_CYCLE_BASE       (204U)
#define FLASH_CYCLE_MAIN_SW           (204U)  /* 主开关周期 ← LOCK_SW 关机计数 (与 @34 同源) */
#define FLASH_CYCLE_HOR_LASER         (208U)  /* 水平激光按键 ← laser_on_cnt[0]=H */
#define FLASH_CYCLE_V1_LASER          (212U)  /* 垂直1激光按键 ← laser_on_cnt[1]=V1 */
#define FLASH_CYCLE_V2_LASER          (216U)  /* 垂直2激光按键 ← laser_on_cnt[2]=V2 */
#define FLASH_CYCLE_LOCK_SW           (220U)  /* 锁定开关周期 ← LOCK_SW 关机计数 (与 @34 同源) */
#define FLASH_CYCLE_BRIGHTNESS        (224U)
#define FLASH_CYCLE_BRIGHTNESS_REMOTE (228U)

/* ======================================================================
 *  Optical 段 (offset 232..243, int32 × 3) — 光功率
 * ====================================================================== */
#define FLASH_OFFSET_OPTICAL_BASE     (232U)
#define FLASH_OPT_H                   (232U)
#define FLASH_OPT_V1                  (236U)
#define FLASH_OPT_V2                  (240U)

/* ======================================================================
 *  Offset 段 (offset 244..261) — MEMS 校准
 * ====================================================================== */
#define FLASH_OFFSET_MEMS_X           (244U)  /* int16 LSB */
#define FLASH_OFFSET_MEMS_Y           (246U)  /* int16 LSB */
#define FLASH_OFFSET_MEMS_Z           (248U)  /* int16 LSB */
#define FLASH_OFFSET_MEMS_RESET       (250U)  /* uint32 校准次数 */
#define FLASH_OFFSET_MEMS_MODEL       (254U)  /* uint8[8] ASCII */

/* ======================================================================
 *  Reference 段 (offset 262..269, uint16 × 4) — PD 参考电压 (mV)
 * ====================================================================== */
#define FLASH_OFFSET_REF_BASE         (262U)
#define FLASH_REF_H1                  (262U)  /* H  ← ref_side */
#define FLASH_REF_H2                  (264U)
#define FLASH_REF_V1                  (266U)  /* V1 ← ref_horiz */
#define FLASH_REF_V2                  (268U)  /* V2 ← ref_front */

/* ======================================================================
 *  其余段（Hawkeye 保持 0）
 * ====================================================================== */
#define FLASH_OFFSET_DROP             (270U)  /* uint16 */
#define FLASH_OFFSET_TEMP_BASE        (272U)  /* uint16 × 3 */
#define FLASH_TEMP_HIGH               (272U)  /* Laser Works in high Temp   ← temp_high_cnt */
#define FLASH_TEMP_NORMAL             (274U)  /* Laser Works in normal Temp ← temp_normal_cnt */
#define FLASH_TEMP_LOW                (276U)  /* Laser Works in low Temp    ← temp_low_cnt */
#define FLASH_OFFSET_BLE_BASE         (278U)  /* uint16 × 2 */
#define FLASH_OFFSET_RUNTIME_BASE     (282U)  /* uint32 × 5 */

/* ======================================================================
 *  关机原因（与 bsp_adc.h 中 power_off_reason_t 对应）
 *
 *  仅统计 5 种关机原因；LASER_HOTPLUG(=0) 由 laser_safety.c 检测触发
 *  关机，但不再作为 datalog 计数（已删除）。
 * ====================================================================== */
#define POWEROFF_REASON_COUNT    (5U)   /* 计数的关机原因数 */

/*
 *  对应关系（索引 = count[] 下标 = power_off_reason_t 枚举值 - 1）：
 *   0 : POWER_OFF_REASON_BATTERY_LOW(1)     → Battery Under Voltage
 *   1 : POWER_OFF_REASON_LOCK_SW_RUNTIME(2) → Main Power Switch Off
 *   2 : POWER_OFF_REASON_I2C_FAIL(3)        → MEMS error (I2C 芯片丢失)
 *   3 : POWER_OFF_REASON_OVERHEAT(4)        → NTC Hot Pack
 *   4 : POWER_OFF_REASON_IDLE_TIMEOUT(5)    → IDLE Timeout
 */

/* ======================================================================
 *  RAM 累加器结构体 — 运行统计（bsp_stats.c 实时累计）
 *
 *  注意：此结构体是 RAM 累加器，字段名保持与 bsp_stats.c / bsp_debug_uart.c
 *  一致；Flash 中的物理布局见 flash.c 中的 save/load 映射。
 *  已删除 uncal_run_s（Hawkeye 无未校准概念，未校准模式按 0-4° 计）。
 * ====================================================================== */
typedef struct
{
    uint32_t  magic;            /* FLASH_STATS_MAGIC */
    uint32_t  total_run_s;      /* 设备总运行时长 (秒) */
    uint32_t  laser_on_cnt[3];  /* 三路激光各自被开启的次数 [H, V1, V2] */
    uint32_t  v1_h_only_s;      /* V1+H 同时开启时长 (秒) */
    uint32_t  v2_h_only_s;      /* V2+H 同时开启时长 (秒) */
    uint32_t  v1_v2_only_s;     /* V1+V2 同时开启时长 (秒) */
    uint32_t  v1_only_s;        /* V1 单独开启时长 (秒) */
    uint32_t  v2_only_s;        /* V2 单独开启时长 (秒) */
    uint32_t  h_only_s;         /* H 单独开启时长 (秒) */
    uint32_t  all3_laser_s;     /* 三路激光同时开启的时长 (秒) */
    uint32_t  angle_0_4_s;      /* 0-4度模式运行时长 (秒) */
    uint32_t  angle_4_10_s;     /* 4-10度模式运行时长 (秒) */
    uint32_t  angle_10_90_s;    /* 10-90度模式运行时长 (秒) */

    /* ---- Slope 段: 激光组合 × (倾角 > 4°) 交叉时长 (秒) ---- */
    uint32_t  slope_h_s;        /* Slope H1       @112 */
    uint32_t  slope_v1_s;       /* Slope V1       @120 */
    uint32_t  slope_v2_s;       /* Slope V2       @124 */
    uint32_t  slope_v1_h_s;     /* Slope H1&V1    @132 */
    uint32_t  slope_v2_h_s;     /* Slope H1&V2    @136 */
    uint32_t  slope_v1_v2_s;    /* Slope V1&V2    @148 */
    uint32_t  slope_all3_s;     /* Slope H1&V1&V2 @160 */

    /* ---- temp 段: 按控制板 NTC 温度分区的检测次数 (开机/运行/关机) ---- */
    uint32_t  temp_high_cnt;    /* Laser Works in high Temp   @272 (>40°C) */
    uint32_t  temp_normal_cnt;  /* Laser Works in normal Temp @274 (0~40°C) */
    uint32_t  temp_low_cnt;     /* Laser Works in low Temp    @276 (<0°C) */

    /* ---- Protection 段新增检测计数 (运行时累积, uint16 落盘) ---- */
    uint32_t  ntc_hot_laser;    /* NTC Hot Laser    @24 (控制板 NTC 高温) */
    uint32_t  ntc_cold_laser;   /* NTC Cold Laser   @26 (控制板 NTC 低温) */
    uint32_t  ntc_frozen_pack;  /* NTC Frozen Pack  @28 (电池包 NTC 冻结) */
    uint32_t  h1_over_cur;      /* H1 Over Current  @40 */
    uint32_t  v1_over_cur;      /* V1 Over Current  @44 */
    uint32_t  v2_over_cur;      /* V2 Over Current  @46 */
} flash_stats_t;

/* ======================================================================
 *  关机日志结构体（RAM 表示，Flash 中映射到 Protection 段）
 * ====================================================================== */
typedef struct
{
    uint32_t  magic;                          /* FLASH_POWEROFF_MAGIC */
    uint32_t  total;                          /* 总关机次数 */
    uint32_t  count[POWEROFF_REASON_COUNT];   /* 各原因关机次数 */
    uint32_t  reserved;                       /* 保留 */
} flash_poweroff_log_t;

/* ======================================================================
 *  参考电压结构体（RAM 表示，Flash 中映射到 Reference 段 uint16）
 * ====================================================================== */
typedef struct
{
    uint32_t  magic;       /* 保留，与 RAM 校验兼容 */
    int32_t   ref_front;   /* V2 参考电压 (mV) */
    int32_t   ref_side;    /* H  参考电压 (mV) */
    int32_t   ref_horiz;   /* V1 参考电压 (mV) */
} flash_ref_data_t;

typedef struct
{
    uint32_t  magic;               /* FLASH_DEVINFO_MAGIC                  */

    /* 硬件版本（各1字节，格式 major.minor.patch.build） */
    uint8_t   hw_major;
    uint8_t   hw_minor;
    uint8_t   hw_patch;
    uint8_t   hw_build;

    /* 固件版本（各1字节） */
    uint8_t   fw_major;
    uint8_t   fw_minor;
    uint8_t   fw_patch;
    uint8_t   fw_build;

    /* ===== 新增字段 ===== */
    uint8_t   laser_on_mask;       /* 激光开启状态掩码（bit0=激光1, bit1=激光2, bit2=激光3） */
    uint8_t   reserved_1[3];       /* 保留，对齐到4字节边界 */

    uint32_t  reserved_ver;        /* 保留，版本区对齐到16字节             */

    char      project_name[32];    /* 项目名称，最多31字节+'\0'            */
    char      reserved_str[12];    /* 预留备注字段，最多11字节+'\0'        */
} flash_devinfo_t;   /* 4+4+4+4+4+4+32+12 = 64字节 */


/* ======================================================================
 *  对外接口
 * ====================================================================== */

fsp_err_t flash_init(void);

fsp_err_t flash_save_calibration(int16_t ox, int16_t oy, int16_t oz);
fsp_err_t flash_load_calibration(int16_t *ox, int16_t *oy, int16_t *oz);

/**
 * @brief 写入 MEMS 型号字符串到 Offset 段 @254 (uint8[8])
 * @param model  型号字符串 (取前 8 字节, 不足补 0)
 * @note  初始化时检测到传感器后调用一次即可
 */
fsp_err_t flash_save_mems_model(const char *model);

/**
 * @brief 读取 MEMS 校准次数 @250 (uint32)
 */
fsp_err_t flash_load_mems_reset_count(uint32_t *count);

/**
 * @brief 读取 MEMS 型号字符串 @254 (uint8[8], 不含终止符)
 */
fsp_err_t flash_load_mems_model(uint8_t model[8]);

fsp_err_t flash_save_reference(int32_t front, int32_t side, int32_t horiz);
fsp_err_t flash_load_reference(int32_t *front, int32_t *side, int32_t *horiz);

/**
 * @brief 加载关机日志（若 Flash 中无有效数据则全部清零返回）
 */
fsp_err_t flash_load_poweroff_log(flash_poweroff_log_t *log);

/**
 * @brief 记录一次关机事件（读-改-写整块）
 * @param reason  关机原因，对应 power_off_reason_t 枚举值（1~5；0=hotplug 不计数）
 */
fsp_err_t flash_record_poweroff(uint32_t reason);

/**
 * @brief 清空关机日志
 */
fsp_err_t flash_clear_poweroff_log(void);

/* 统计数据接口 */
fsp_err_t flash_load_stats(flash_stats_t *stats);
fsp_err_t flash_save_stats(const flash_stats_t *stats);
fsp_err_t flash_clear_stats(void);
/* 新增函数声明 */
fsp_err_t flash_save_poweroff_and_stats(uint32_t reason,
                                        const flash_stats_t *stats);

/* ======================================================================
 *  新增函数声明
 * ====================================================================== */
fsp_err_t flash_load_devinfo(flash_devinfo_t *info);
fsp_err_t flash_save_devinfo(const flash_devinfo_t *info);
fsp_err_t flash_clear_devinfo(void);

/* 用当前 hawkeye_config.h 的宏自动填充版本信息，写入Flash */
fsp_err_t flash_save_devinfo_from_config(void);
void flash_close(void);

/* ======================================================================
 *  激光状态接口
 * ====================================================================== */
fsp_err_t flash_save_laser_state(uint8_t mask);
uint8_t flash_load_laser_state(void);
fsp_err_t flash_clear_all(void);

/* ======================================================================
 *  校准请求 flag 接口 (UBOX → APP 跨固件通信)
 * ====================================================================== */
bool     flash_check_calib_request(void);
fsp_err_t flash_clear_calib_request(void);

#endif /* FLASH_FLASH_H_ */
