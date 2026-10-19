#ifndef FLASH_FLASH_H_
#define FLASH_FLASH_H_

#include "hal_data.h"
#include "stdint.h"
#include "hawkeye_config.h"

/* ======================================================================
 *  RA2E1 Data Flash 参数（与 hawkeye_config.h 保持一致）
 * ====================================================================== */
#define FLASH_DF_BASE_ADDR       (0x40100000U)
#define FLASH_DF_BLOCK_SIZE      (64U)
#define FLASH_DF_WRITE_SIZE      (4U)

#define FLASH_BLOCK_CALIBRATION  (FLASH_DF_BASE_ADDR)

/* ======================================================================
 *  魔数
 * ====================================================================== */
#define FLASH_CALIB_MAGIC        (0xA5A5A5A5U)
#define FLASH_REF_MAGIC          (0x5A5A5A5AU)
#define FLASH_POWEROFF_MAGIC     (0xC3C3C3C3U)   /* 关机日志有效标识 */

#define FLASH_BLOCK_STATS        (FLASH_DF_BASE_ADDR + FLASH_DF_BLOCK_SIZE)  /* 第2块，偏移64 */
#define FLASH_STATS_MAGIC        (0xB6B6B6B6U)   /* 统计数据有效标识 */

#define FLASH_TOTAL_BLOCKS       (3U)                                        /* 扩展为3块 */
#define FLASH_TOTAL_SIZE         (FLASH_DF_BLOCK_SIZE * FLASH_TOTAL_BLOCKS)  /* 192字节  */

#define FLASH_OFFSET_STATS       (64U)    /* 第2块：运行统计        */
#define FLASH_OFFSET_DEVINFO     (128U)   /* 第3块：版本/设备信息   */

#define FLASH_DEVINFO_MAGIC      (0xD4D4D4D4U)   /* 设备信息有效标识 */
/* ======================================================================
 *  存储布局（64字节块内）
 *
 *   偏移  0 ~ 11  : flash_calib_data_t    (12字节)
 *   偏移 12 ~ 15  : 保留对齐
 *   偏移 16 ~ 31  : flash_ref_data_t      (16字节)
 *   偏移 32 ~ 63  : flash_poweroff_log_t  (32字节) ← 新增
 * ====================================================================== */
#define FLASH_OFFSET_CALIB       (0U)
#define FLASH_OFFSET_REF         (16U)
#define FLASH_OFFSET_POWEROFF    (32U)    /* 关机日志块内偏移 */

/* ======================================================================
 *  UBOX 校准请求 flag (offset 12, 4 bytes)
 *  UBOX 收到 Honcho 校准命令后写入, APP 启动时检测
 * ====================================================================== */
#define FLASH_OFFSET_CALIB_REQ   (12U)            /* 第1块 offset 12 */
#define FLASH_CALIB_REQ_ADDR     (FLASH_BLOCK_CALIBRATION + FLASH_OFFSET_CALIB_REQ)
#define FLASH_CALIB_REQ_MAGIC    (0xCA11B000U)    /* UBOX 写入的校准请求魔数 */

/* ======================================================================
 *  校准数据结构体 (12字节)
 * ====================================================================== */
typedef struct
{
    uint32_t  magic;       /* FLASH_CALIB_MAGIC */
    int16_t   offset_x;
    int16_t   offset_y;
    int16_t   offset_z;
    uint16_t  reserved;
} flash_calib_data_t;

/* ======================================================================
 *  参考电压结构体 (16字节)
 * ====================================================================== */
typedef struct
{
    uint32_t  magic;       /* FLASH_REF_MAGIC */
    int32_t   ref_front;
    int32_t   ref_side;
    int32_t   ref_horiz;
} flash_ref_data_t;

/* ======================================================================
 *  关机原因枚举（与 bsp_adc.c 中的 power_off_reason_t 对应）
 *  此处单独定义数量宏，方便数组索引
 * ====================================================================== */
#define POWEROFF_REASON_COUNT    (6U)   /* 关机原因总数，与 power_off_reason_t 枚举项数一致 */

/*
 * 对应关系（索引 = power_off_reason_t 枚举值）：
 *   0 : POWER_OFF_REASON_LASER_HOTPLUG
 *   1 : POWER_OFF_REASON_BATTERY_LOW
 *   2 : POWER_OFF_REASON_LOCK_SW_RUNTIME
 *   3 : POWER_OFF_REASON_I2C_FAIL
 *   4 : POWER_OFF_REASON_OVERHEAT
 *   5 : POWER_OFF_REASON_IDLE_TIMEOUT
 */

/* ======================================================================
 *  关机日志结构体 (36字节)
 *
 *   magic        4字节
 *   total        4字节  总关机次数
 *   count[6]     24字节 各原因次数（uint32_t × 6）
 *   reserved     4字节  保留
 *   共           36字节
 * ====================================================================== */
typedef struct
{
    uint32_t  magic;                          /* FLASH_POWEROFF_MAGIC */
    uint32_t  total;                          /* 总关机次数 */
    uint32_t  count[POWEROFF_REASON_COUNT];   /* 各原因关机次数 */
    uint32_t  reserved;                       /* 保留 */
} flash_poweroff_log_t;   /* 4+4+24+4 = 36字节 */



/* ======================================================================
 *  运行统计数据 — 存储在第2块 Data Flash (基地址 + 64)
 *
 *  存储布局（64字节块）：
 *    偏移  0 ~ 3  : magic          (4字节)
 *    偏移  4 ~ 7  : total_run_s    总运行时长(秒)         (4字节)
 *    偏移  8 ~19  : laser_on_cnt   三路激光开启次数        (3×4=12字节)
 *    偏移 20 ~31  : laser_on_s     三路激光开启总时长(秒)  (3×4=12字节)
 *    偏移 32 ~35  : any_laser_s    任意激光开启时长(秒)    (4字节)
 *    偏移 36 ~39  : all3_laser_s   三路同时开启时长(秒)    (4字节)
 *    偏移 40 ~43  : uncal_run_s    未校准模式运行时长(秒)  (4字节)
 *    偏移 44 ~47  : angle_0_4_s    0-4度模式时长(秒)      (4字节)
 *    偏移 48 ~51  : angle_4_10_s   4-10度模式时长(秒)     (4字节)
 *    偏移 52 ~55  : angle_10_90_s  10-90度模式时长(秒)    (4字节)
 *    偏移 56 ~59  : led_blink_s    LED闪烁(低电)时长(秒)  (4字节)
 *    偏移 60 ~63  : reserved       保留                   (4字节)
 *  共 64字节
 * ====================================================================== */

typedef struct
{
    uint32_t  magic;            /* FLASH_STATS_MAGIC */
    uint32_t  total_run_s;      /* 设备总运行时长 (秒) */
    uint32_t  laser_on_cnt[3];  /* 三路激光各自被开启的次数 [V1, V2, H] */
    uint32_t  v1_h_only_s;      /* V1+H 同时开启时长 (秒) */
    uint32_t  v2_h_only_s;      /* V2+H 同时开启时长 (秒) */
    uint32_t  v1_v2_only_s;     /* V1+V2 同时开启时长 (秒) */
    uint32_t  v1_only_s;        /* V1 单独开启时长 (秒) */
    uint32_t  v2_only_s;        /* V2 单独开启时长 (秒) */
    uint32_t  h_only_s;         /* H 单独开启时长 (秒) */
    uint32_t  all3_laser_s;     /* 三路激光同时开启的时长 (秒) */
    uint32_t  uncal_run_s;      /* 未校准(UNCAL)模式运行时长 (秒) */
    uint32_t  angle_0_4_s;      /* 0-4度模式运行时长 (秒) */
    uint32_t  angle_4_10_s;     /* 4-10度模式运行时长 (秒) */
    uint32_t  angle_10_90_s;    /* 10-90度模式运行时长 (秒) */
} flash_stats_t;   /* 4+4+12+4+4+4+4+4+4+4+4+4+4+4 = 64字节 */

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

fsp_err_t flash_save_reference(int32_t front, int32_t side, int32_t horiz);
fsp_err_t flash_load_reference(int32_t *front, int32_t *side, int32_t *horiz);

/**
 * @brief 加载关机日志（若 Flash 中无有效数据则全部清零返回）
 */
fsp_err_t flash_load_poweroff_log(flash_poweroff_log_t *log);

/**
 * @brief 记录一次关机事件（读-改-写整块）
 * @param reason  关机原因，对应 power_off_reason_t 枚举值（范围 0~4）
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
