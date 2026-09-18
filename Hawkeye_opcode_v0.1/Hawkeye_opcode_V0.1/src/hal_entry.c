#include <key/bsp_irq_key.h>
#include <led/bsp_led.h>
#include <adc/bsp_adc.h>
#include <agt/bsp_agt_timing.h>
#include <SysTick/bsp_SysTick.h>
#include "hal_data.h"
#include "debug_uart/bsp_debug_uart.h"
#include "common_utils.h"
#include "timer_pwm.h"
#include <iic/iic.h>
#include "flash/flash.h"
#include "wdt/wdt.h"
#include "hawkeye_config.h"
#include "stats/bsp_stats.h"

/* ======================================================================
 *  HardFault Handler: 喂狗后软复位, 避免 WDT 粗暴复位导致 Data Flash 二次损坏
 * ====================================================================== */
void HardFault_Handler(void);
void HardFault_Handler(void)
{
    /* 喂狗: 防止 WDT 在 Flash 操作期间复位 → ECC 损坏 */
    R_WDT_Refresh(&g_wdt0_ctrl);

    /* 简单标记: 写HardFault标志到SRAM, 下次启动可检测 */
    *(volatile uint32_t *)0x20007FE0U = 0x48464155U;  /* "HFAU" */

    /* 软复位: 比 WDT 复位更安全, 不打断 Flash 操作 */
    NVIC_SystemReset();

    while (1) { __WFI(); }
}

static uint32_t no_laser_idle_s = 0U;
static uint16_t g_calib_fail_cnt = 0;       /* 校准期间连续 I2C 读取失败计数 */

static int16_t acc[3];
extern volatile uint8_t gSystemTickFlag;   /* 1ms节拍标志 */
extern volatile int reference_voltage;
extern volatile int reference_voltage1;
extern volatile int reference_voltage2;
extern volatile uint16_t g_bat_adc_raw;

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

/* ======================================================================
 *  静态辅助函数
 * ====================================================================== */

/** Flash 初始化并加载所有持久化数据 */
static void flash_load_all(void)
{
    if (flash_init() == FSP_SUCCESS)
    {
        /* 1. IMU 校准数据 */
        int16_t ox, oy, oz;
        if (flash_load_calibration(&ox, &oy, &oz) == FSP_SUCCESS)
        {
            imu_offset[0] = ox;
            imu_offset[1] = oy;
            imu_offset[2] = oz;
        }

        /* 2. 参考电压 */
        int32_t front, side, horiz;
        if (flash_load_reference(&front, &side, &horiz) == FSP_SUCCESS)
        {
            reference_voltage  = (int)front;
            reference_voltage1 = (int)side;
            reference_voltage2 = (int)horiz;
        }

        /* 3. 版本信息：无数据则首次写入，版本变化则刷新 */
        flash_devinfo_t info;
        fsp_err_t dev_err = flash_load_devinfo(&info);

        if (dev_err == FSP_ERR_NOT_INITIALIZED)
        {
            flash_save_devinfo_from_config();
        }
        else if (dev_err == FSP_SUCCESS)
        {
            if (info.fw_major != HAWKEYE_VERSION_MAJOR ||
                info.fw_minor != HAWKEYE_VERSION_MINOR ||
                info.fw_patch != HAWKEYE_VERSION_PATCH ||
                info.fw_build != HAWKEYE_VERSION_BUILD)
            {
                flash_save_devinfo_from_config();
            }
        }

        /* 4. 运行统计 */
        stats_init();
    }
    else
    {
        /* Flash 初始化失败，统计模块仍初始化（全零数据） */
        stats_init();
    }
}

/** NTC 控制板温度保护：高温关机 + 低温标记 */
static void ntc_temp_protection_task(uint16_t ntc_adc)
{
    /* 高温模式已触发，等待闪烁完成后自动关机，不再检测温度 */
    if (gAngleMode == ANGLE_MODE_OVERHEAT) return;

    static uint16_t ntc_overheat_cnt = 0;
    static uint16_t ntc_low_temp_cnt  = 0;

    if (ntc_adc < NTC_OVERHEAT_THRESHOLD)
    {
        /* 高温保护: NTC ADC < 450 → >100°C */
        ntc_low_temp_cnt = 0;
        ntc_overheat_cnt++;
        if (ntc_overheat_cnt >= NTC_TEMP_CONFIRM_COUNT)
        {
            ntc_overheat_cnt = 0;
            //system_overheat_request(OVERHEAT_SOURCE_NTC_BOARD);
        }
    }
    else if (ntc_adc > NTC_LOW_TEMP_THRESHOLD)
    {
        /* 低温检测: NTC ADC > 3150 → <-7°C */
        ntc_overheat_cnt = 0;
        ntc_low_temp_cnt++;
        if (ntc_low_temp_cnt >= NTC_TEMP_CONFIRM_COUNT)
        {
            ntc_low_temp_cnt = 0;
            g_ntc_low_temp = true;
        }
    }
    else
    {
        /* 正常温度区间 */
        ntc_overheat_cnt = 0;
        ntc_low_temp_cnt  = 0;

        /* 低温恢复: 带滞回, 需低于 (阈值 - 回差) 连续 N 次才退出 */
        static uint16_t ntc_recover_cnt = 0;
        if (g_ntc_low_temp)
        {
            if (ntc_adc < (NTC_LOW_TEMP_THRESHOLD - NTC_LOW_TEMP_HYST))
            {
                ntc_recover_cnt++;
                if (ntc_recover_cnt >= NTC_TEMP_CONFIRM_COUNT)
                {
                    ntc_recover_cnt = 0;
                    g_ntc_low_temp = false;
                }
            }
            else
            {
                ntc_recover_cnt = 0;
            }
        }
    }
}

/** IMU 校准 + 姿态更新 */
static void imu_calib_posture_task(void)
{
    if (g_calib_request)
    {
        /* 首次进入校准 → 打开所有激光开始闪烁 */
        if (!g_calib_laser_flash)
        {
            if (laser_serial_ctrl_all(true))
            {
                g_calib_laser_flash = true;
                g_calib_laser_on   = true;
                g_calib_fail_cnt    = 0;
            }
            /* 若 MOS 忙则等下个周期重试 */
        }
        else if (g_calib_cnt == 0)
        {
            /* 重新校准 (AT+CALIBRATION 再次发送): 同步失败计数 */
            g_calib_fail_cnt = 0;
        }

        if (imu_get_acc(acc) == 0)
        {
            wdt_feed();
            g_calib_fail_cnt = 0;
            g_calib_sum[0] += acc[0];
            g_calib_sum[1] += acc[1];
            g_calib_sum[2] += acc[2];
            g_calib_cnt++;

            if (g_calib_cnt >= CALIBRATION_TRIALS)
            {
                wdt_feed();
                g_calib_request      = false;
                g_calib_laser_flash   = false;
                /* 校准完成：停止闪烁，设 H 亮 / V1,V2 载波 */
                gLaserOn[0] = 1;
                gLaserOn[1] = 0;
                gLaserOn[2] = 0;
                wdt_feed();
                set_laser1_200k_intensity(LASER_IDLE_DUTY, TIMER_PIN);
                set_laser2_200k_intensity(LASER_DEFAULT_DUTY, TIMER_PIN);
                set_laser3_200k_intensity(LASER_IDLE_DUTY, GPT_IO_PIN_GTIOCA);

                imu_offset[0] = (int16_t)(g_calib_sum[0] / CALIBRATION_TRIALS);
                imu_offset[1] = (int16_t)(g_calib_sum[1] / CALIBRATION_TRIALS);
                imu_offset[2] = (int16_t)(g_calib_sum[2] / CALIBRATION_TRIALS);

                char msg[64];
                sprintf(msg, "+RESP:CALIB DONE X=%d Y=%d Z=%d\r\n",
                        imu_offset[0], imu_offset[1], imu_offset[2]);
                uart9_send_blocking(msg);
                wdt_feed();
                flash_save_calibration(imu_offset[0], imu_offset[1], imu_offset[2]);
            }
        }
        else
        {
            /* I2C 读取失败: 累计连续失败次数 */
            g_calib_fail_cnt++;
            if (g_calib_fail_cnt >= 10)
            {
                g_calib_fail_cnt = 0;
                g_calib_cnt      = 0;
                g_calib_sum[0]   = 0;
                g_calib_sum[1]   = 0;
                g_calib_sum[2]   = 0;
                uart9_send_blocking("+RESP:CALIBRATION RETRY\r\n");
            }
        }
    }
    else
    {
        /* 正常模式: 读取加速度并更新姿态 */
        if (imu_get_acc(acc) == 0)
        {
            wdt_feed();  /* I2C读取后喂狗 */

            /* 保存原始值用于调试打印 */
            int16_t raw_x = acc[0];
            int16_t raw_y = acc[1];
            int16_t raw_z = acc[2];

            acc[1] -= imu_offset[1];
            acc[2] -= imu_offset[2];
            posture_update(acc);

            if (g_imu_debug_enabled)
            {
                /* AT+IMUDBG=1: 每 100ms 打印原始/校准后加速度 + 角度模式 */
                char msg[96];
                sprintf(msg, "+IMU:RAW=%d,%d,%d CAL=%d,%d,%d MODE=%d\r\n",
                        raw_x, raw_y, raw_z,
                        acc[0], acc[1], acc[2],
                        (int)gAngleMode);
                uart9_send_blocking(msg);
            }
        }
    }
}

/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    fsp_err_t err = FSP_SUCCESS;
    R_BSP_PinAccessEnable();

    /* TODO: add your own code here */
    LED_Init();
    //system_power_on();

    /* 开机锁定开关检测：关闭状态(P407=HIGH)则复位 */
    {
        bsp_io_level_t lock_level;
        R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_07, &lock_level);
        if (lock_level == BSP_IO_LEVEL_HIGH)
        {
            /* 锁定开关处于关闭位置，切断电源并软件复位 */
            LED1_OFF; LED2_OFF; LED3_OFF;
            Batt_OFF;
            Power_En_OFF;
            NVIC_SystemReset();
        }
    }
    //system_power_on();
    ADC_Init();
    Start_ADC();

    /* 开机电压采样：4 次取平均 */
    uint32_t bat_sum = 0;
    for (int i = 0; i < 4; i++)
    {
        scan_complete_flag = false;
        while (!scan_complete_flag)
        {
            wdt_feed();  /* ADC忙等待中喂狗, 防止扫描停转导致WDT复位 */
        }
        scan_complete_flag = false;
        uint16_t tmp = 0;
        R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_9, &tmp);
        bat_sum += tmp;
    }
    uint16_t bat_adc_boot = (uint16_t)(bat_sum / 4);

    Debug_UART9_Init();

    if (bat_adc_boot < BAT_BOOT_MIN_ADC)
    {
        /* 电压不够，不维持供电，喂狗等电容放完电自然断电 */
        LED1_OFF; LED2_OFF; LED3_OFF;
        Batt_OFF;
        Power_En_OFF;
        while (1) { wdt_feed(); }
    }
    system_power_on();
    battery_state_init(bat_adc_boot);

    /* P100 同时启用 ADC AN022 和 UART RX (电池NTC温度采样) */
    R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_00,
        ((uint32_t)IOPORT_CFG_ANALOG_ENABLE | (uint32_t)IOPORT_CFG_PERIPHERAL_PIN
         | (uint32_t)IOPORT_PERIPHERAL_SCI0_2_4_6_8));

    GPT_Timing_Init();
    key_init();
    wdt_init();

    /*
     *  idx  period  offset  用途
     *   0     10      0     按键扫描
     *   1     20      5     电源监控
     *   2     50     15     串口 + 电池
     *   3    100     30     MEMS + 姿态
     *   4    500     30     低频任务
     *   5   1000     50     运行统计
     */
    static const uint16_t PERIOD[] = { 10,  20,  50,  80,  500, 1000 };
    static       uint16_t CNT[]    = {  0,   5,  15,  30,   40,   50 };

    /* I2C + IMU 初始化 (自动检测 IIM42351/ICM40608) */
    i2c_master_init();
    if (imu_init(1) != 0)
    {
        system_power_off_request(POWER_OFF_REASON_I2C_FAIL);
    }

    /* 启动 GPT PWM */
    err = gpt_initialize();
    if (FSP_SUCCESS != err) { APP_ERR_TRAP(err); }
    err = gpt_start();
    if (FSP_SUCCESS != err) { APP_ERR_TRAP(err); }

    R_GPT_Start(&g_timer9_ctrl);    /* 1ms 中断 */

    flash_load_all();

    /* 启动时 控制板NTC 温度检测: 低温/高温即时生效, 不等运行期确认计数 */
    {
        uint16_t ntc_boot = Read_ADC_Voltage_Value_NTC();
        if (ntc_boot < NTC_OVERHEAT_THRESHOLD)
        {
            system_overheat_request(OVERHEAT_SOURCE_NTC_BOOT);
        }
        else if (ntc_boot > NTC_LOW_TEMP_THRESHOLD)
        {
            g_ntc_low_temp = true;
        }
    }

    /* 启动时 电池包NTC 过热检测 (单次, 无需多次平均) */
    {
        uint16_t vbatt = (uint16_t)g_bat_adc_raw;
        uint16_t th    = (uint16_t)g_bat_battery_temp_raw;
        if (th > 0)
        {
            int32_t r_x100 = (int32_t)4043 * (int32_t)vbatt / (int32_t)th - 2411;
            if (r_x100 > 0 && r_x100 < 196)
            {
                system_overheat_request(OVERHEAT_SOURCE_BATTERY);
            }
        }
    }

    /* ==================================================================
     *  检测 UBOX 校准请求 flag
     *
     *  若 UBOX 通过 Honcho 协议写入了校准请求 flag,
     *  则在进入主循环前同步完成校准, 确保 APP 启动即处于已校准状态.
     * ================================================================== */

    if (flash_check_calib_request())
    {
        /* 清除 flag, 然后交给主循环的 imu_calib_posture_task() 执行校准 */
        flash_clear_calib_request();
        g_calib_request = true;
        //uart9_send_blocking("+RESP:BOOT CALIB REQ\r\n");
    }
    else
    {
        uart9_send_blocking("+RESP:BOOT CALIB NOT REQ\r\n");
    }

    LED1_ON; LED2_ON; LED3_ON;
    system_power_on();
    /* ==================================================================
     *  主循环：6 槽协作调度器，由 1ms 节拍驱动
     * ================================================================== */
    while (1)
    {
        if (gSystemTickFlag)
        {
            gSystemTickFlag = 0;
            wdt_feed();

            /* ---- 1ms 任务 ---- */
            key_en_delay_task_1ms();
            laser_main_loop_task();

            /* ---- 10ms 任务 ---- */
            if (++CNT[0] >= PERIOD[0])
            {
                wdt_feed();
                CNT[0] = 0;
                system_power_off_process_pending();
                key_scan_task();
                key_handle_events();
                lock_sw_read();

                /* 上电后延迟 100ms 恢复激光状态 (10ms槽位, 计数10次=100ms) */
                static bool     laser_state_restored = false;
                static uint16_t restore_delay_10ms   = 0;
                if (!laser_state_restored)
                {
                    restore_delay_10ms++;
                    if (restore_delay_10ms >= 10)
                    {
                        key_restore_on_boot();
                        laser_state_restored = true;
                    }
                }
            }

            /* ---- 20ms 任务: NTC 温度保护 ---- */
            if (++CNT[1] >= PERIOD[1])
            {
                wdt_feed();
                CNT[1] = 0;
                ntc_temp_protection_task(Read_ADC_Voltage_Value_NTC());
            }

            /* ---- 50ms 任务: 串口 + 电池 + IMU姿态 ---- */
            if (++CNT[2] >= PERIOD[2])
            {
                wdt_feed();
                CNT[2] = 0;
                Debug_UART9_ProcessReceivedData();
                battery_voltage_task();

            }

            /* ---- 100ms 任务: (空) ---- */
            if (++CNT[3] >= PERIOD[3])
            {
                wdt_feed();
                CNT[3] = 0;
                imu_calib_posture_task();
            }

            /* ---- 500ms 任务: 电池 LED ---- */
            if (++CNT[4] >= PERIOD[4])
            {
                wdt_feed();
                CNT[4] = 0;
                battery_led_task();
            }

            /* ---- 1000ms 任务 ---- */
            if (++CNT[5] >= PERIOD[5])
            {
                wdt_feed();
                CNT[5] = 0;

                battery_temp_check();

                /* 电池ADC原始值, 每秒输出一次 */
/*                {
                    char bat_str[32];
                    snprintf(bat_str, sizeof(bat_str), "+BAT_ADC:%u\r\n",
                             Read_ADC_Voltage_Value_BAT());
                    uart9_send_blocking(bat_str);
                }*/
            }

            /* ---- 按秒统计 + 无激光超时关机 ---- */
            if (stats_tick_pending())
            {
                wdt_feed();
                stats_tick_1s((const uint8_t *)gLaserOn,
                              (uint8_t)gAngleMode,
                              battery_is_low_blink());

                if (gAngleMode != ANGLE_MODE_OVERHEAT)
                {
                    if (!gLaserOn[0] && !gLaserOn[1] && !gLaserOn[2])
                    {
                        no_laser_idle_s++;
                        if (no_laser_idle_s >= NO_LASER_POWEROFF_S)
                            system_power_off_request(POWER_OFF_REASON_IDLE_TIMEOUT);
                    }
                    else
                    {
                        no_laser_idle_s = 0U;
                    }
                }
            }
        }
        else
        {
            wdt_feed();  /* 后备喂狗: 定时器停转时防止WDT复位 */
        }
    }

#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}


/*******************************************************************************************************************//**
 * This function is called at various points during the startup process.  This implementation uses the event that is
 * called right before main() to set up the pins.
 *
 * @param[in]  event    Where at in the start up process the code is currently at
 **********************************************************************************************************************/
void R_BSP_WarmStart(bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
#if BSP_FEATURE_FLASH_LP_VERSION != 0
        /* Enable reading from data flash. */
        R_FACI_LP->DFLCTL = 1U;
        /* Would normally have to wait tDSTOP(6us) for data flash recovery. Placing the enable here, before clock and
         * C runtime initialization, should negate the need for a delay since the initialization will typically take more than 6us. */
#endif
    }

    if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment and system clocks are setup. */
        /* Configure pins. */
        R_IOPORT_Open (&IOPORT_CFG_CTRL, &IOPORT_CFG_NAME);

#if BSP_CFG_SDRAM_ENABLED
        /* Setup SDRAM and initialize it. Must configure pins first. */
        R_BSP_SdramInit(true);
#endif
    }
}

#if BSP_TZ_SECURE_BUILD

FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

/* Trustzone Secure Projects require at least one nonsecure callable function in order to build (Remove this if it is not required to build). */
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
FSP_CPP_FOOTER

#endif
