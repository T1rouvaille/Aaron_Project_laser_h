#include "flash/flash.h"
#include "debug_uart/bsp_debug_uart.h"
#include "wdt/wdt.h"
#include "r_flash_lp.h"
#include "stats/bsp_stats.h"
#include "string.h"

/* ======================================================================
 *  平台化 Data Flash 缓冲区（384 字节 = 6 块）
 *    buf[0..319]   = 标准 datalog（320 字节，5 块），boot 'F' 读取
 *    buf[320..383] = flash_devinfo_t（版本/设备信息，1 块，仅 APP 内部）
 * ====================================================================== */
static uint8_t g_flash_buf[FLASH_TOTAL_SIZE];

/* ======================================================================
 *  内部辅助：小端字节读写（RA2E1 Cortex-M23 不支持非对齐访问，逐字节安全）
 * ====================================================================== */
static void buf_w16(uint8_t *buf, uint32_t off, uint16_t v)
{
    buf[off]     = (uint8_t)(v & 0xFFU);
    buf[off + 1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void buf_w32(uint8_t *buf, uint32_t off, uint32_t v)
{
    buf[off]     = (uint8_t)(v & 0xFFU);
    buf[off + 1] = (uint8_t)((v >> 8)  & 0xFFU);
    buf[off + 2] = (uint8_t)((v >> 16) & 0xFFU);
    buf[off + 3] = (uint8_t)((v >> 24) & 0xFFU);
}

static uint16_t buf_r16(const uint8_t *buf, uint32_t off)
{
    return (uint16_t)((uint16_t)buf[off] | ((uint16_t)buf[off + 1] << 8));
}

static uint32_t buf_r32(const uint8_t *buf, uint32_t off)
{
    return (uint32_t)buf[off]
         | ((uint32_t)buf[off + 1] << 8)
         | ((uint32_t)buf[off + 2] << 16)
         | ((uint32_t)buf[off + 3] << 24);
}

/* 直接从 Data Flash 读取（load 路径，无需先载入缓冲） */
static uint16_t flash_rd16(uint32_t off)
{
    const uint8_t *p = (const uint8_t *)(FLASH_BLOCK_CALIBRATION + off);
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t flash_rd32(uint32_t off)
{
    const uint8_t *p = (const uint8_t *)(FLASH_BLOCK_CALIBRATION + off);
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/* RAM uint32 计数 → Flash uint16 字段（饱和到 0xFFFF，避免回绕） */
static uint16_t u32_to_u16(uint32_t v)
{
    return (v > 0xFFFFU) ? 0xFFFFU : (uint16_t)v;
}

/* ======================================================================
 *  内部辅助：缓冲 <-> Flash
 * ====================================================================== */
static void flash_buf_load(void)
{
    memcpy(g_flash_buf,
           (const void *)FLASH_BLOCK_CALIBRATION,
           FLASH_TOTAL_SIZE);   /* 384 字节（6 块） */
}

static fsp_err_t flash_buf_flush(void)
{
    fsp_err_t err;
    bool      write_phase   = false;
    uint32_t  primask       = __get_PRIMASK();
    uint32_t  systick_ctrl  = SysTick->CTRL;

    /* Data Flash 擦写临界区: 禁中断 + 停 SysTick,
     * 防止 200kHz PWM ISR 打断擦写时序 → HardFault (与 AT+FWUPDATE 一致) */
    __disable_irq();
    SysTick->CTRL = systick_ctrl & ~SysTick_CTRL_ENABLE_Msk;

    wdt_feed();  /* Flash擦除前喂狗, 防止WDT超时 */
    err = R_FLASH_LP_Erase(&g_flash0_ctrl,
                           FLASH_BLOCK_CALIBRATION,
                           FLASH_TOTAL_BLOCKS);   /* 6 块一次擦除 */

    if (err == FSP_SUCCESS)
    {
        write_phase = true;
        wdt_feed();  /* Flash写入前喂狗, 防止WDT超时 */
        err = R_FLASH_LP_Write(&g_flash0_ctrl,
                               (uint32_t)g_flash_buf,
                               FLASH_BLOCK_CALIBRATION,
                               FLASH_TOTAL_SIZE);     /* 384 字节一次写入 */
    }

    /* 先恢复中断, 再打印失败信息 (避免关中断下阻塞串口) */
    SysTick->CTRL = systick_ctrl;
    __set_PRIMASK(primask);

    if (err != FSP_SUCCESS)
    {
        uart9_send_blocking(write_phase ? "+RESP:FLASH WRITE FAIL\r\n"
                                        : "+RESP:FLASH ERASE FAIL\r\n");
    }

    return err;
}

/* ======================================================================
 *  内部辅助：datalog 头部校验 / 魔数 / 首次初始化
 * ====================================================================== */

/** @brief datalog 是否有效（@0 魔数匹配） */
static bool flash_datalog_valid(void)
{
    return flash_rd32(FLASH_OFFSET_HDR_MAGIC) == FLASH_DATALOG_MAGIC;
}

/** @brief 在缓冲中写入 datalog 头部魔数（所有 save 函数调用） */
static void flash_buf_set_magic(void)
{
    buf_w32(g_flash_buf, FLASH_OFFSET_HDR_MAGIC, FLASH_DATALOG_MAGIC);
}

/**
 * @brief 首次写 datalog 前的初始化：若魔数无效（新/已擦除），
 *        先清零整个 datalog 区域（0..319），使标准中 Hawkeye 未使用的
 *        字段保持 0（而非 0xFF）。同时保留校准请求 flag（@12..15），
 *        避免覆盖 UBOX 待处理的校准请求。
 */
static void flash_buf_prepare_datalog(void)
{
    if (buf_r32(g_flash_buf, FLASH_OFFSET_HDR_MAGIC) == FLASH_DATALOG_MAGIC)
        return;   /* 已初始化，无需清零 */

    uint32_t calib_req = buf_r32(g_flash_buf, FLASH_OFFSET_CALIB_REQ);
    memset(g_flash_buf, 0, FLASH_DATALOG_SIZE);
    buf_w32(g_flash_buf, FLASH_OFFSET_CALIB_REQ, calib_req);

    /* Optical 段默认值: 首次初始化写入默认目标功率,
     * 之后由 AT+POWTARGET 经 flash_save_optical_targets() 覆盖 */
    buf_w32(g_flash_buf, FLASH_OPT_H1, (uint32_t)POWCAL_TARGET_H);
    buf_w32(g_flash_buf, FLASH_OPT_H2, 0U);               /* H2 未使用, 恒 0 */
    buf_w32(g_flash_buf, FLASH_OPT_V1, (uint32_t)POWCAL_TARGET_V1);
    buf_w32(g_flash_buf, FLASH_OPT_V2, (uint32_t)POWCAL_TARGET_V2);
}

/* ======================================================================
 *  关机原因 → Protection 段偏移（uint16）
 *  count[] 下标 = power_off_reason_t 枚举值 - 1（跳过 0=LASER_HOTPLUG）
 * ====================================================================== */
static const uint16_t poweroff_prot_offsets[POWEROFF_REASON_COUNT] = {
    FLASH_PROT_BATTERY_UV,     /* count[0] = BATTERY_LOW(1)      → @32 */
    FLASH_PROT_MAIN_SW_OFF,    /* count[1] = LOCK_SW_RUNTIME(2)  → @34 */
    FLASH_PROT_MEMS_ERROR,     /* count[2] = I2C_FAIL(3)         → @36 */
    FLASH_PROT_NTC_HOT_PACK,   /* count[3] = OVERHEAT(4)         → @30 */
    FLASH_PROT_IDLE_TIMEOUT,   /* count[4] = IDLE_TIMEOUT(5)     → @38 */
};

/* ======================================================================
 *  内部辅助：stats 的 scatter（RAM 结构体 → 标准偏移）/ gather（反向）
 * ====================================================================== */
static void flash_scatter_stats(uint8_t *buf, const flash_stats_t *stats)
{
    /* Header */
    buf_w32(buf, FLASH_OFFSET_HDR_TOTAL_RUN, stats->total_run_s);

    /* Time 段 */
    buf_w32(buf, FLASH_TIME_H1,      stats->h_only_s);
    buf_w32(buf, FLASH_TIME_V1,      stats->v1_only_s);
    buf_w32(buf, FLASH_TIME_V2,      stats->v2_only_s);
    buf_w32(buf, FLASH_TIME_H1V1,    stats->v1_h_only_s);
    buf_w32(buf, FLASH_TIME_H1V2,    stats->v2_h_only_s);
    buf_w32(buf, FLASH_TIME_V1V2,    stats->v1_v2_only_s);
    buf_w32(buf, FLASH_TIME_H1V1V2,  stats->all3_laser_s);
    buf_w32(buf, FLASH_TIME_TILT_UNCAL, stats->uncal_run_s);
    buf_w32(buf, FLASH_TIME_TILT_0_4,   stats->angle_0_4_s);
    buf_w32(buf, FLASH_TIME_TILT_4_10,  stats->angle_4_10_s);
    buf_w32(buf, FLASH_TIME_TILT_10_90, stats->angle_10_90_s);

    /* Slope 段（激光组合 × 倾角>4° 交叉时长，仅 7 个非 H2 组合有效） */
    buf_w32(buf, FLASH_SLOPE_H1,     stats->slope_h_s);
    buf_w32(buf, FLASH_SLOPE_V1,     stats->slope_v1_s);
    buf_w32(buf, FLASH_SLOPE_V2,     stats->slope_v2_s);
    buf_w32(buf, FLASH_SLOPE_H1V1,   stats->slope_v1_h_s);
    buf_w32(buf, FLASH_SLOPE_H1V2,   stats->slope_v2_h_s);
    buf_w32(buf, FLASH_SLOPE_V1V2,   stats->slope_v1_v2_s);
    buf_w32(buf, FLASH_SLOPE_H1V1V2, stats->slope_all3_s);

    /* Cycle 段（laser_on_cnt: [0]=H, [1]=V1, [2]=V2; 按键开+关各计一次; H2 无激光保持 0） */
    buf_w32(buf, FLASH_CYCLE_H1_LASER, stats->laser_on_cnt[0]);
    buf_w32(buf, FLASH_CYCLE_H2_LASER, 0);
    buf_w32(buf, FLASH_CYCLE_V1_LASER, stats->laser_on_cnt[1]);
    buf_w32(buf, FLASH_CYCLE_V2_LASER, stats->laser_on_cnt[2]);

    /* Optical 段（光功率目标）不在此处写: 由 flash_buf_prepare_datalog()
     * 首次初始化写入默认宏, 之后经 AT+POWTARGET 由 flash_save_optical_targets()
     * 覆盖; 统计保存不应覆写用户自定义目标值。 */

    /* temp 段（次数, uint16, 饱和） */
    buf_w16(buf, FLASH_TEMP_HIGH,   u32_to_u16(stats->temp_high_cnt));
    buf_w16(buf, FLASH_TEMP_NORMAL, u32_to_u16(stats->temp_normal_cnt));
    buf_w16(buf, FLASH_TEMP_LOW,    u32_to_u16(stats->temp_low_cnt));

    /* Protection 段新增检测计数（uint16, 饱和） */
    buf_w16(buf, FLASH_PROT_NTC_HOT_LASER,   u32_to_u16(stats->ntc_hot_laser));
    buf_w16(buf, FLASH_PROT_NTC_COLD_LASER,  u32_to_u16(stats->ntc_cold_laser));
    buf_w16(buf, FLASH_PROT_NTC_FROZEN_PACK, u32_to_u16(stats->ntc_frozen_pack));
    buf_w16(buf, FLASH_PROT_H1_OVERCUR,      u32_to_u16(stats->h1_over_cur));
    buf_w16(buf, FLASH_PROT_V1_OVERCUR,      u32_to_u16(stats->v1_over_cur));
    buf_w16(buf, FLASH_PROT_V2_OVERCUR,      u32_to_u16(stats->v2_over_cur));
}

static void flash_gather_stats(flash_stats_t *stats)
{
    memset(stats, 0, sizeof(*stats));
    stats->magic = FLASH_STATS_MAGIC;

    stats->total_run_s   = flash_rd32(FLASH_OFFSET_HDR_TOTAL_RUN);
    stats->h_only_s      = flash_rd32(FLASH_TIME_H1);
    stats->v1_only_s     = flash_rd32(FLASH_TIME_V1);
    stats->v2_only_s     = flash_rd32(FLASH_TIME_V2);
    stats->v1_h_only_s   = flash_rd32(FLASH_TIME_H1V1);
    stats->v2_h_only_s   = flash_rd32(FLASH_TIME_H1V2);
    stats->v1_v2_only_s  = flash_rd32(FLASH_TIME_V1V2);
    stats->all3_laser_s  = flash_rd32(FLASH_TIME_H1V1V2);
    stats->uncal_run_s   = flash_rd32(FLASH_TIME_TILT_UNCAL);
    stats->angle_0_4_s   = flash_rd32(FLASH_TIME_TILT_0_4);
    stats->angle_4_10_s  = flash_rd32(FLASH_TIME_TILT_4_10);
    stats->angle_10_90_s = flash_rd32(FLASH_TIME_TILT_10_90);

    /* Slope 段 */
    stats->slope_h_s     = flash_rd32(FLASH_SLOPE_H1);
    stats->slope_v1_s    = flash_rd32(FLASH_SLOPE_V1);
    stats->slope_v2_s    = flash_rd32(FLASH_SLOPE_V2);
    stats->slope_v1_h_s  = flash_rd32(FLASH_SLOPE_H1V1);
    stats->slope_v2_h_s  = flash_rd32(FLASH_SLOPE_H1V2);
    stats->slope_v1_v2_s = flash_rd32(FLASH_SLOPE_V1V2);
    stats->slope_all3_s  = flash_rd32(FLASH_SLOPE_H1V1V2);

    stats->laser_on_cnt[0] = flash_rd32(FLASH_CYCLE_H1_LASER);
    stats->laser_on_cnt[1] = flash_rd32(FLASH_CYCLE_V1_LASER);
    stats->laser_on_cnt[2] = flash_rd32(FLASH_CYCLE_V2_LASER);

    /* temp 段（次数, uint16 → uint32） */
    stats->temp_high_cnt   = (uint32_t)flash_rd16(FLASH_TEMP_HIGH);
    stats->temp_normal_cnt = (uint32_t)flash_rd16(FLASH_TEMP_NORMAL);
    stats->temp_low_cnt    = (uint32_t)flash_rd16(FLASH_TEMP_LOW);

    /* Protection 段新增检测计数（uint16 → uint32） */
    stats->ntc_hot_laser   = (uint32_t)flash_rd16(FLASH_PROT_NTC_HOT_LASER);
    stats->ntc_cold_laser  = (uint32_t)flash_rd16(FLASH_PROT_NTC_COLD_LASER);
    stats->ntc_frozen_pack = (uint32_t)flash_rd16(FLASH_PROT_NTC_FROZEN_PACK);
    stats->h1_over_cur = (uint32_t)flash_rd16(FLASH_PROT_H1_OVERCUR);
    stats->v1_over_cur = (uint32_t)flash_rd16(FLASH_PROT_V1_OVERCUR);
    stats->v2_over_cur = (uint32_t)flash_rd16(FLASH_PROT_V2_OVERCUR);
}

/* ======================================================================
 *  flash_init
 * ====================================================================== */
fsp_err_t flash_init(void)
{
    return R_FLASH_LP_Open(&g_flash0_ctrl, &g_flash0_cfg);
}

/* ======================================================================
 *  校准数据（MEMS 偏移 → Offset 段 @256/258/260, int16）
 * ====================================================================== */
fsp_err_t flash_save_calibration(int16_t ox, int16_t oy, int16_t oz)
{
    flash_buf_load();
    flash_buf_prepare_datalog();

    buf_w16(g_flash_buf, FLASH_OFFSET_MEMS_X, (uint16_t)ox);
    buf_w16(g_flash_buf, FLASH_OFFSET_MEMS_Y, (uint16_t)oy);
    buf_w16(g_flash_buf, FLASH_OFFSET_MEMS_Z, (uint16_t)oz);

    /* MEMS Reset Count +1（读-改-写 @262，每完成一次校准递增） */
    uint32_t reset_cnt = buf_r32(g_flash_buf, FLASH_OFFSET_MEMS_RESET) + 1U;
    buf_w32(g_flash_buf, FLASH_OFFSET_MEMS_RESET, reset_cnt);

    flash_buf_set_magic();

    /* 校准数据写入时同步清除 UBOX 的校准请求 flag (offset 12-15) */
    memset(g_flash_buf + FLASH_OFFSET_CALIB_REQ, 0xFF, 4U);

    return flash_buf_flush();
}

fsp_err_t flash_save_mems_model(const char *model)
{
    if (model == NULL) return FSP_ERR_INVALID_ARGUMENT;

    flash_buf_load();
    flash_buf_prepare_datalog();

    /* MEMS Model：取前 8 字节写入 @266（不足部分补 0） */
    memset(g_flash_buf + FLASH_OFFSET_MEMS_MODEL, 0, 8U);
    for (uint8_t i = 0; i < 8U; i++)
    {
        if (model[i] == '\0') break;
        g_flash_buf[FLASH_OFFSET_MEMS_MODEL + i] = (uint8_t)model[i];
    }

    flash_buf_set_magic();
    return flash_buf_flush();
}

fsp_err_t flash_load_calibration(int16_t *ox, int16_t *oy, int16_t *oz)
{
    if (!flash_datalog_valid())
        return FSP_ERR_NOT_INITIALIZED;

    *ox = (int16_t)flash_rd16(FLASH_OFFSET_MEMS_X);
    *oy = (int16_t)flash_rd16(FLASH_OFFSET_MEMS_Y);
    *oz = (int16_t)flash_rd16(FLASH_OFFSET_MEMS_Z);
    return FSP_SUCCESS;
}

fsp_err_t flash_load_mems_reset_count(uint32_t *count)
{
    if (!flash_datalog_valid())
        return FSP_ERR_NOT_INITIALIZED;

    *count = flash_rd32(FLASH_OFFSET_MEMS_RESET);
    return FSP_SUCCESS;
}

fsp_err_t flash_load_mems_model(uint8_t model[8])
{
    if (!flash_datalog_valid())
        return FSP_ERR_NOT_INITIALIZED;

    const uint8_t *p = (const uint8_t *)(FLASH_BLOCK_CALIBRATION + FLASH_OFFSET_MEMS_MODEL);
    for (uint8_t i = 0; i < 8U; i++)
    {
        model[i] = p[i];
    }
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
 *  参考电压（int32 RAM → Reference 段 @274/276/278/280, uint16）
 *    H1(270) ← ref_side(H),  H2(272) 未使用,  V1(274) ← ref_horiz(V1),
 *    V2(276) ← ref_front(V2)
 * ====================================================================== */
fsp_err_t flash_save_reference(int32_t front, int32_t side, int32_t horiz)
{
    flash_buf_load();
    flash_buf_prepare_datalog();

    buf_w16(g_flash_buf, FLASH_REF_H1, (uint16_t)side);   /* H  ← ref_side  */
    buf_w16(g_flash_buf, FLASH_REF_H2, 0);                /* H2 未使用，保持 0 */
    buf_w16(g_flash_buf, FLASH_REF_V1, (uint16_t)horiz);  /* V1 ← ref_horiz */
    buf_w16(g_flash_buf, FLASH_REF_V2, (uint16_t)front);  /* V2 ← ref_front */
    flash_buf_set_magic();

    return flash_buf_flush();
}

fsp_err_t flash_load_reference(int32_t *front, int32_t *side, int32_t *horiz)
{
    if (!flash_datalog_valid())
        return FSP_ERR_NOT_INITIALIZED;

    *front = (int32_t)flash_rd16(FLASH_REF_V2);  /* V2 ← ref_front */
    *side  = (int32_t)flash_rd16(FLASH_REF_H1);  /* H  ← ref_side  */
    *horiz = (int32_t)flash_rd16(FLASH_REF_V1);  /* V1 ← ref_horiz */
    return FSP_SUCCESS;
}

/* ======================================================================
 *  Optical 段（光功率目标 int32 × 4, @240/244/248/252）
 *   调试阶段经 AT+POWTARGET 写入, 替换默认 POWCAL_TARGET_* 宏;
 *   若从未设置, 首次初始化时由 flash_buf_prepare_datalog() 写入默认宏。
 *   H2 为 Stepper 标准的水平2激光, Hawkeye 无此路, 恒写 0。
 * ====================================================================== */
fsp_err_t flash_save_optical_targets(int h1, int h2, int v1, int v2)
{
    flash_buf_load();
    flash_buf_prepare_datalog();

    buf_w32(g_flash_buf, FLASH_OPT_H1, (uint32_t)h1);
    buf_w32(g_flash_buf, FLASH_OPT_H2, (uint32_t)h2);
    buf_w32(g_flash_buf, FLASH_OPT_V1, (uint32_t)v1);
    buf_w32(g_flash_buf, FLASH_OPT_V2, (uint32_t)v2);

    flash_buf_set_magic();
    return flash_buf_flush();
}

void flash_load_optical_targets(int *h1, int *h2, int *v1, int *v2)
{
    if (h1) *h1 = (int)flash_rd32(FLASH_OPT_H1);
    if (h2) *h2 = (int)flash_rd32(FLASH_OPT_H2);
    if (v1) *v1 = (int)flash_rd32(FLASH_OPT_V1);
    if (v2) *v2 = (int)flash_rd32(FLASH_OPT_V2);
}

/* ======================================================================
 *  关机日志（Protection 段, uint16；total = 各原因计数之和）
 * ====================================================================== */
fsp_err_t flash_load_poweroff_log(flash_poweroff_log_t *log)
{
    if (log == NULL) return FSP_ERR_INVALID_ARGUMENT;

    if (!flash_datalog_valid())
    {
        memset(log, 0, sizeof(*log));
        log->magic = FLASH_POWEROFF_MAGIC;
        return FSP_ERR_NOT_INITIALIZED;
    }

    memset(log, 0, sizeof(*log));
    log->magic = FLASH_POWEROFF_MAGIC;
    log->total = 0;
    for (uint8_t i = 0; i < POWEROFF_REASON_COUNT; i++)
    {
        log->count[i] = (uint32_t)flash_rd16(poweroff_prot_offsets[i]);
        log->total += log->count[i];
    }
    return FSP_SUCCESS;
}

fsp_err_t flash_record_poweroff(uint32_t reason)
{
    flash_buf_load();
    flash_buf_prepare_datalog();

    flash_poweroff_log_t log;
    memset(&log, 0, sizeof(log));
    log.magic = FLASH_POWEROFF_MAGIC;
    log.total = 0;
    for (uint8_t i = 0; i < POWEROFF_REASON_COUNT; i++)
    {
        log.count[i] = (uint32_t)buf_r16(g_flash_buf, poweroff_prot_offsets[i]);
        log.total += log.count[i];
    }

    /* 仅统计 1..5 的关机原因；0=LASER_HOTPLUG 不再计数 */
    if (reason >= 1U && reason <= POWEROFF_REASON_COUNT)
    {
        log.count[reason - 1U]++;
        log.total++;
    }

    for (uint8_t i = 0; i < POWEROFF_REASON_COUNT; i++)
        buf_w16(g_flash_buf, poweroff_prot_offsets[i], (uint16_t)log.count[i]);

    /* Cycle 段: 主开关/锁定开关周期次数 = LOCK_SW 关机计数 (与 @34 同源) */
    buf_w32(g_flash_buf, FLASH_CYCLE_MAIN_SW, log.count[1]);
    buf_w32(g_flash_buf, FLASH_CYCLE_LOCK_SW,  log.count[1]);

    flash_buf_set_magic();
    return flash_buf_flush();
}

fsp_err_t flash_clear_poweroff_log(void)
{
    flash_buf_load();
    for (uint8_t i = 0; i < POWEROFF_REASON_COUNT; i++)
        buf_w16(g_flash_buf, poweroff_prot_offsets[i], 0);

    /* Cycle 段主开关/锁定开关周期次数同源, 一并清零 */
    buf_w32(g_flash_buf, FLASH_CYCLE_MAIN_SW, 0);
    buf_w32(g_flash_buf, FLASH_CYCLE_LOCK_SW,  0);

    return flash_buf_flush();
}

/* ======================================================================
 *  运行统计（Time/Cycle 段，scatter/gather 映射）
 * ====================================================================== */
fsp_err_t flash_load_stats(flash_stats_t *stats)
{
    if (stats == NULL) return FSP_ERR_INVALID_ARGUMENT;

    if (!flash_datalog_valid())
    {
        memset(stats, 0, sizeof(*stats));
        stats->magic = FLASH_STATS_MAGIC;
        return FSP_ERR_NOT_INITIALIZED;
    }

    flash_gather_stats(stats);
    return FSP_SUCCESS;
}

fsp_err_t flash_save_stats(const flash_stats_t *stats)
{
    if (stats == NULL) return FSP_ERR_INVALID_ARGUMENT;

    flash_buf_load();
    flash_buf_prepare_datalog();
    flash_scatter_stats(g_flash_buf, stats);
    flash_buf_set_magic();

    return flash_buf_flush();
}

fsp_err_t flash_clear_stats(void)
{
    flash_buf_load();

    flash_stats_t stats;
    memset(&stats, 0, sizeof(stats));
    flash_scatter_stats(g_flash_buf, &stats);

    return flash_buf_flush();
}

/* ======================================================================
 *  关机时一次性写入：关机日志 + 统计数据 同一次 flush 完成
 * ====================================================================== */
fsp_err_t flash_save_poweroff_and_stats(uint32_t reason,
                                        const flash_stats_t *stats)
{
    flash_buf_load();
    flash_buf_prepare_datalog();

    /* ---- 更新关机日志 (Protection 段) ---- */
    flash_poweroff_log_t log;
    memset(&log, 0, sizeof(log));
    log.magic = FLASH_POWEROFF_MAGIC;
    log.total = 0;
    for (uint8_t i = 0; i < POWEROFF_REASON_COUNT; i++)
    {
        log.count[i] = (uint32_t)buf_r16(g_flash_buf, poweroff_prot_offsets[i]);
        log.total += log.count[i];
    }

    if (reason >= 1U && reason <= POWEROFF_REASON_COUNT)
    {
        log.count[reason - 1U]++;
        log.total++;
    }

    for (uint8_t i = 0; i < POWEROFF_REASON_COUNT; i++)
        buf_w16(g_flash_buf, poweroff_prot_offsets[i], (uint16_t)log.count[i]);

    /* Cycle 段: 主开关/锁定开关周期次数 = LOCK_SW 关机计数 (与 @34 同源) */
    buf_w32(g_flash_buf, FLASH_CYCLE_MAIN_SW, log.count[1]);
    buf_w32(g_flash_buf, FLASH_CYCLE_LOCK_SW,  log.count[1]);

    /* ---- 更新统计数据 (Time/Cycle 段) ---- */
    if (stats != NULL)
        flash_scatter_stats(g_flash_buf, stats);

    flash_buf_set_magic();
    return flash_buf_flush();
}

/* ======================================================================
 *  版本/设备信息（第 6 块, offset 320, 独立于 datalog）
 * ====================================================================== */
fsp_err_t flash_load_devinfo(flash_devinfo_t *info)
{
    if (info == NULL) return FSP_ERR_INVALID_ARGUMENT;

    const flash_devinfo_t *p =
        (const flash_devinfo_t *)(FLASH_BLOCK_CALIBRATION + FLASH_OFFSET_DEVINFO);

    if (p->magic != FLASH_DEVINFO_MAGIC)
    {
        memset(info, 0, sizeof(*info));
        info->magic = FLASH_DEVINFO_MAGIC;
        return FSP_ERR_NOT_INITIALIZED;
    }

    memcpy(info, p, sizeof(*info));
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
    memset(&info, 0, sizeof(info));
    info.magic = FLASH_DEVINFO_MAGIC;
    memcpy(g_flash_buf + FLASH_OFFSET_DEVINFO, &info, sizeof(info));

    return flash_buf_flush();
}

/* 用 hawkeye_config.h 的宏自动写入当前版本信息 */
fsp_err_t flash_save_devinfo_from_config(void)
{
    flash_devinfo_t info;
    memset(&info, 0, sizeof(info));

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

    /* 硬件/固件版本字符串（Stepper 标准 Module ID 属性） */
    strncpy(info.hardware_revision, HAWKEYE_HW_REVISION_STR,
            sizeof(info.hardware_revision) - 1U);
    info.hardware_revision[sizeof(info.hardware_revision) - 1U] = '\0';
    strncpy(info.firmware_revision, HAWKEYE_FW_REVISION_STR,
            sizeof(info.firmware_revision) - 1U);
    info.firmware_revision[sizeof(info.firmware_revision) - 1U] = '\0';

    return flash_save_devinfo(&info);
}

/* ======================================================================
 *  底层字段读取接口 (调试/完整 dump 用)
 * ====================================================================== */
uint16_t flash_read_u16(uint32_t offset)
{
    return flash_rd16(offset);
}

uint32_t flash_read_u32(uint32_t offset)
{
    return flash_rd32(offset);
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
