#include "flash/flash.h"
#include "debug_uart/bsp_debug_uart.h"
#include "wdt/wdt.h"
#include "r_flash_lp.h"
#include "stats/bsp_stats.h"
#include "string.h"

/* ======================================================================
 *  统一128字节缓冲（覆盖第1块+第2块）
 *  buf[0~63]  = 第1块：校准 + 参考电压 + 关机日志
 *  buf[64~127] = 第2块：运行统计
 * ====================================================================== */
static uint8_t g_flash_buf[FLASH_TOTAL_SIZE];

/* ======================================================================
 *  内部辅助：从Flash把两块全部读入缓冲
 * ====================================================================== */
static void flash_buf_load(void)
{
    memcpy(g_flash_buf,
           (const void *)FLASH_BLOCK_CALIBRATION,
           FLASH_TOTAL_SIZE);   /* 自动跟随 FLASH_TOTAL_SIZE = 192 */
}


/* ======================================================================
 *  内部辅助：把缓冲里的两块一次性擦除并写回Flash
 *  RA2E1 R_FLASH_LP_Erase 支持连续多块：传块数=2，一次擦除128字节
 * ====================================================================== */
static fsp_err_t flash_buf_flush(void)
{
    fsp_err_t err;

    wdt_feed();  /* Flash擦除前喂狗, 防止WDT超时 */
    err = R_FLASH_LP_Erase(&g_flash0_ctrl,
                           FLASH_BLOCK_CALIBRATION,
                           FLASH_TOTAL_BLOCKS);   /* 3块一次擦除 */
    if (err != FSP_SUCCESS)
    {
        uart9_send_blocking("+RESP:FLASH ERASE FAIL\r\n");
        return err;
    }

    wdt_feed();  /* Flash写入前喂狗, 防止WDT超时 */
    err = R_FLASH_LP_Write(&g_flash0_ctrl,
                           (uint32_t)g_flash_buf,
                           FLASH_BLOCK_CALIBRATION,
                           FLASH_TOTAL_SIZE);     /* 192字节一次写入 */
    if (err != FSP_SUCCESS)
    {
        uart9_send_blocking("+RESP:FLASH WRITE FAIL\r\n");
        return err;
    }

    return FSP_SUCCESS;
}

/* ======================================================================
 *  flash_init
 * ====================================================================== */
fsp_err_t flash_init(void)
{
    return R_FLASH_LP_Open(&g_flash0_ctrl, &g_flash0_cfg);
}

/* ======================================================================
 *  校准数据
 * ====================================================================== */
fsp_err_t flash_save_calibration(int16_t ox, int16_t oy, int16_t oz)
{
    flash_buf_load();

    flash_calib_data_t calib;
    calib.magic    = FLASH_CALIB_MAGIC;
    calib.offset_x = ox;
    calib.offset_y = oy;
    calib.offset_z = oz;
    calib.reserved = 0;
    memcpy(g_flash_buf + FLASH_OFFSET_CALIB, &calib, sizeof(flash_calib_data_t));

    /* 校准数据写入时同步清除 UBOX 的校准请求 flag (offset 12-15) */
    memset(g_flash_buf + FLASH_OFFSET_CALIB_REQ, 0xFF, 4U);

    return flash_buf_flush();
}

fsp_err_t flash_load_calibration(int16_t *ox, int16_t *oy, int16_t *oz)
{
    const flash_calib_data_t *p =
        (const flash_calib_data_t *)(FLASH_BLOCK_CALIBRATION + FLASH_OFFSET_CALIB);

    if (p->magic != FLASH_CALIB_MAGIC)
        return FSP_ERR_NOT_INITIALIZED;

    *ox = p->offset_x;
    *oy = p->offset_y;
    *oz = p->offset_z;
    return FSP_SUCCESS;
}

/* ======================================================================
 *  UBOX 校准请求 flag (跨固件通信: UBOX 写入 → APP 检测并执行校准)
 *
 *  UBOX 收到 Honcho 校准命令后写 0xCA11B000 到 Data Flash offset 12.
 *  APP 启动时检测此值, 若存在则先执行校准再清除 flag.
 *  flash_save_calibration() 会自动擦除此 flag (offset 12-15 写 0xFF).
 * ====================================================================== */

/**
 * @brief 检测 UBOX 是否请求校准 (直接读 Data Flash, 无需打开驱动)
 * @return true  有待处理的校准请求
 */
bool flash_check_calib_request(void)
{
    return (*(uint32_t *)FLASH_CALIB_REQ_ADDR) == FLASH_CALIB_REQ_MAGIC;
}

/**
 * @brief 清除校准请求 flag (读-改-写整块)
 * @note  调用 flash_save_calibration() 时会自动清除, 通常无需单独调用此函数
 */
fsp_err_t flash_clear_calib_request(void)
{
    flash_buf_load();
    memset(g_flash_buf + FLASH_OFFSET_CALIB_REQ, 0xFF, 4U);
    return flash_buf_flush();
}

/* ======================================================================
 *  参考电压
 * ====================================================================== */
fsp_err_t flash_save_reference(int32_t front, int32_t side, int32_t horiz)
{
    flash_buf_load();

    flash_ref_data_t ref;
    ref.magic     = FLASH_REF_MAGIC;
    ref.ref_front = front;
    ref.ref_side  = side;
    ref.ref_horiz = horiz;
    memcpy(g_flash_buf + FLASH_OFFSET_REF, &ref, sizeof(flash_ref_data_t));

    return flash_buf_flush();
}

fsp_err_t flash_load_reference(int32_t *front, int32_t *side, int32_t *horiz)
{
    const flash_ref_data_t *p =
        (const flash_ref_data_t *)(FLASH_BLOCK_CALIBRATION + FLASH_OFFSET_REF);

    if (p->magic != FLASH_REF_MAGIC)
        return FSP_ERR_NOT_INITIALIZED;

    *front = p->ref_front;
    *side  = p->ref_side;
    *horiz = p->ref_horiz;
    return FSP_SUCCESS;
}

/* ======================================================================
 *  关机日志
 * ====================================================================== */
fsp_err_t flash_load_poweroff_log(flash_poweroff_log_t *log)
{
    if (log == NULL) return FSP_ERR_INVALID_ARGUMENT;

    const flash_poweroff_log_t *p =
        (const flash_poweroff_log_t *)(FLASH_BLOCK_CALIBRATION + FLASH_OFFSET_POWEROFF);

    if (p->magic != FLASH_POWEROFF_MAGIC)
    {
        memset(log, 0, sizeof(flash_poweroff_log_t));
        log->magic = FLASH_POWEROFF_MAGIC;
        return FSP_ERR_NOT_INITIALIZED;
    }

    memcpy(log, p, sizeof(flash_poweroff_log_t));
    return FSP_SUCCESS;
}

fsp_err_t flash_record_poweroff(uint32_t reason)
{
    flash_buf_load();

    flash_poweroff_log_t log;
    memcpy(&log, g_flash_buf + FLASH_OFFSET_POWEROFF, sizeof(flash_poweroff_log_t));

    if (log.magic != FLASH_POWEROFF_MAGIC)
    {
        memset(&log, 0, sizeof(flash_poweroff_log_t));
        log.magic = FLASH_POWEROFF_MAGIC;
    }

    log.total++;
    if (reason < POWEROFF_REASON_COUNT)
        log.count[reason]++;

    memcpy(g_flash_buf + FLASH_OFFSET_POWEROFF, &log, sizeof(flash_poweroff_log_t));

    return flash_buf_flush();
}

fsp_err_t flash_clear_poweroff_log(void)
{
    flash_buf_load();

    flash_poweroff_log_t log;
    memset(&log, 0, sizeof(flash_poweroff_log_t));
    log.magic = FLASH_POWEROFF_MAGIC;
    memcpy(g_flash_buf + FLASH_OFFSET_POWEROFF, &log, sizeof(flash_poweroff_log_t));

    return flash_buf_flush();
}

/* ======================================================================
 *  运行统计（偏移64，在第2块）
 * ====================================================================== */
fsp_err_t flash_load_stats(flash_stats_t *stats)
{
    if (stats == NULL) return FSP_ERR_INVALID_ARGUMENT;

    const flash_stats_t *p =
        (const flash_stats_t *)(FLASH_BLOCK_CALIBRATION + FLASH_OFFSET_STATS);

    if (p->magic != FLASH_STATS_MAGIC)
    {
        memset(stats, 0, sizeof(flash_stats_t));
        stats->magic = FLASH_STATS_MAGIC;
        return FSP_ERR_NOT_INITIALIZED;
    }

    memcpy(stats, p, sizeof(flash_stats_t));
    return FSP_SUCCESS;
}

fsp_err_t flash_save_stats(const flash_stats_t *stats)
{
    if (stats == NULL) return FSP_ERR_INVALID_ARGUMENT;

    flash_buf_load();
    memcpy(g_flash_buf + FLASH_OFFSET_STATS, stats, sizeof(flash_stats_t));

    return flash_buf_flush();
}

fsp_err_t flash_clear_stats(void)
{
    flash_buf_load();

    flash_stats_t stats;
    memset(&stats, 0, sizeof(flash_stats_t));
    stats.magic = FLASH_STATS_MAGIC;
    memcpy(g_flash_buf + FLASH_OFFSET_STATS, &stats, sizeof(flash_stats_t));

    return flash_buf_flush();
}

/* ======================================================================
 *  关机时一次性写入：关机日志 + 统计数据 同一次 flush 完成
 * ====================================================================== */
fsp_err_t flash_save_poweroff_and_stats(uint32_t reason,
                                        const flash_stats_t *stats)
{
    /* 读入全部128字节 */
    flash_buf_load();

    /* ---- 更新关机日志 ---- */
    flash_poweroff_log_t log;
    memcpy(&log, g_flash_buf + FLASH_OFFSET_POWEROFF, sizeof(flash_poweroff_log_t));

    if (log.magic != FLASH_POWEROFF_MAGIC)
    {
        memset(&log, 0, sizeof(flash_poweroff_log_t));
        log.magic = FLASH_POWEROFF_MAGIC;
    }
    log.total++;
    if (reason < POWEROFF_REASON_COUNT)
        log.count[reason]++;

    memcpy(g_flash_buf + FLASH_OFFSET_POWEROFF, &log, sizeof(flash_poweroff_log_t));

    /* ---- 更新统计数据 ---- */
    if (stats != NULL)
        memcpy(g_flash_buf + FLASH_OFFSET_STATS, stats, sizeof(flash_stats_t));

    /* ---- 一次 flush，两块一起写 ---- */
    return flash_buf_flush();
}

/* ======================================================================
 *  版本/设备信息
 * ====================================================================== */
fsp_err_t flash_load_devinfo(flash_devinfo_t *info)
{
    if (info == NULL) return FSP_ERR_INVALID_ARGUMENT;

    const flash_devinfo_t *p =
        (const flash_devinfo_t *)(FLASH_BLOCK_CALIBRATION + FLASH_OFFSET_DEVINFO);

    if (p->magic != FLASH_DEVINFO_MAGIC)
    {
        memset(info, 0, sizeof(flash_devinfo_t));
        info->magic = FLASH_DEVINFO_MAGIC;
        return FSP_ERR_NOT_INITIALIZED;
    }

    memcpy(info, p, sizeof(flash_devinfo_t));
    return FSP_SUCCESS;
}

fsp_err_t flash_save_devinfo(const flash_devinfo_t *info)
{
    if (info == NULL) return FSP_ERR_INVALID_ARGUMENT;

    flash_buf_load();
    memcpy(g_flash_buf + FLASH_OFFSET_DEVINFO, info, sizeof(flash_devinfo_t));

    return flash_buf_flush();
}

fsp_err_t flash_clear_devinfo(void)
{
    flash_buf_load();

    flash_devinfo_t info;
    memset(&info, 0, sizeof(flash_devinfo_t));
    info.magic = FLASH_DEVINFO_MAGIC;
    memcpy(g_flash_buf + FLASH_OFFSET_DEVINFO, &info, sizeof(flash_devinfo_t));

    return flash_buf_flush();
}

/* 用 hawkeye_config.h 的宏自动写入当前版本信息 */
fsp_err_t flash_save_devinfo_from_config(void)
{
    flash_devinfo_t info;
    memset(&info, 0, sizeof(flash_devinfo_t));

    info.magic    = FLASH_DEVINFO_MAGIC;

    /* 硬件版本：从 hawkeye_config.h 读取 */
    info.hw_major = HAWKEYE_VERSION_MAJOR;
    info.hw_minor = HAWKEYE_VERSION_MINOR;
    info.hw_patch = HAWKEYE_VERSION_PATCH;
    info.hw_build = HAWKEYE_VERSION_BUILD;

    /* 固件版本：与硬件版本共用同一套宏，如有独立固件版本宏可替换 */
    info.fw_major = HAWKEYE_VERSION_MAJOR;
    info.fw_minor = HAWKEYE_VERSION_MINOR;
    info.fw_patch = HAWKEYE_VERSION_PATCH;
    info.fw_build = HAWKEYE_VERSION_BUILD;

    /* 项目名称 */
    strncpy(info.project_name, HAWKEYE_PROJECT_NAME, sizeof(info.project_name) - 1U);
    info.project_name[sizeof(info.project_name) - 1U] = '\0';

    /* reserved_str 预留空，可后续通过 AT 指令写入SN等 */
    memset(info.reserved_str, 0, sizeof(info.reserved_str));

    return flash_save_devinfo(&info);
}

/* ======================================================================
 *  flash_close
 * ====================================================================== */
void flash_close(void)
{
    R_FLASH_LP_Close(&g_flash0_ctrl);
}

/* ======================================================================
 *  激光状态接口
 * ====================================================================== */

/**
 * @brief 保存激光状态到Flash
 * @param mask 激光状态掩码（bit0=激光1, bit1=激光2, bit2=激光3）
 */
fsp_err_t flash_save_laser_state(uint8_t mask)
{
    flash_devinfo_t info;
    fsp_err_t err = flash_load_devinfo(&info);
    
    if (err != FSP_SUCCESS && err != FSP_ERR_NOT_INITIALIZED) {
        return err;
    }
    
    // 更新激光状态
    info.laser_on_mask = mask;
    
    // 保存到Flash
    return flash_save_devinfo(&info);
}

/**
 * @brief 从Flash加载激光状态
 * @return 激光状态掩码
 */
uint8_t flash_load_laser_state(void)
{
    flash_devinfo_t info;
    fsp_err_t err = flash_load_devinfo(&info);
    
    if (err != FSP_SUCCESS) {
        return 0;  // 默认全关
    }
    
    return info.laser_on_mask;
}
fsp_err_t flash_clear_all(void)
{
    memset(g_flash_buf, 0xFF, FLASH_TOTAL_SIZE);
    fsp_err_t err = flash_buf_flush();

    /* Flash 擦除后同步清零 RAM 统计，防止关机时旧数据写回 */
    stats_clear();

    return err;
}

