/**
 * @file    hawkeye_config.h
 * @brief   Hawkeye 激光控制系统 — 统一配置文件
 *
 * 所有可调参数集中管理，方便维护和审查。
 * 硬件引脚级宏（LED1_ON 等）仍保留在 bsp_led.h，此处只放数值型配置。
 *
 * @version 0.0.0.1
 * @date    2026-04-15
 */

#ifndef HAWKEYE_CONFIG_H_
#define HAWKEYE_CONFIG_H_

/* ======================================================================
 *  项目信息
 * ====================================================================== */
#define HAWKEYE_PROJECT_NAME    "Hawkeye"           /* 项目名称 */
#define HAWKEYE_VERSION_MAJOR   0                   /* 主版本号 */
#define HAWKEYE_VERSION_MINOR   0                   /* 次版本号 */
#define HAWKEYE_VERSION_PATCH   0                   /* 修订号 */
#define HAWKEYE_VERSION_BUILD   1                   /* 构建号 */
#define HAWKEYE_VERSION_STR     "0.0.0.1"           /* 版本字符串 */

/* ======================================================================
 *  MCU / 硬件基础参数
 * ====================================================================== */

/* ======================================================================
 *  安全保护 — 超时参数（CRITICAL 级保护）
 * ====================================================================== */


/* ======================================================================
 *  电池管理参数
 * ====================================================================== */

/* 电池电压阈值 (ADC 原始值，非 mV)
 * 换算公式: mV = adc * 3300 * 48 / 5 / 4095 */
#define BAT_HIGH_MV             (2464U)     /* 19.2V: FULL→HIGH 边界 */
#define BAT_MID_MV              (2299U)     /* 17.9V: HIGH→MID  边界 */
#define BAT_LOW_MV              (1819U)     /* 14.5V: MID →关断  边界 */

/** 开机最低电压阈值 (ADC 原始值) */
#define BAT_BOOT_MIN_ADC        (1825U)


/** 电池状态迟滞量 (ADC 原始值) */
#define BAT_HYST                (50U)

/** 激光IR压降补偿 (ADC 原始值)
 *  激光开启时电池电压会跌落, 补偿后状态机判断更准确
 *  H 电流最大, V1/V2 电流相近
 *  测量方法: 对比激光全关和全开时的电池ADC差值 */
#define BAT_IR_DROP_H_ADC       18       /* H 激光压降补偿 (TODO: 实测后确定) */
#define BAT_IR_DROP_V1_ADC      16       /* V1激光压降补偿 (TODO: 实测后确定) */
#define BAT_IR_DROP_V2_ADC     16       /* V2激光压降补偿 (TODO: 实测后确定) */

/* 低压关机确认次数 — battery_voltage_task() 每 50ms 调用一次
 *  5 次 × 50ms = 250ms 连续确认，防止瞬时掉压误关机 */
#define BAT_CRITICAL_CONFIRM_COUNT  (5U)

/* ======================================================================
 *  按键参数
 * ====================================================================== */
#define KEY_SCAN_PERIOD_MS      (10U)       /* 按键扫描周期 (ms) */
#define KEY_DEBOUNCE_MS         (50U)       /* 消抖时间 (ms) */
#define KEY_LONG_PRESS_MS       (1000U)     /* 长按判定阈值 (ms) */
#define KEY_ACTIVE_LEVEL        (0U)        /* 按下时电平: 0=低电平有效 */
#define MOS_DELAY_MS            (10U)       /* MOS 驱动延迟 (ms)，PWM 稳定后再开 EN */
#define KEY_STARTUP_IGNORE_MS   (300U)      /* 上电后按键屏蔽时间 (ms) */

/* 锁定开关运行时关机确认次数，lock_sw_read() 每 10ms 调用一次
 *  5 次 × 10ms = 50ms，避免接触抖动或干扰误关机 */
#define LOCK_SW_RUNTIME_CONFIRM_COUNT  (5U)

/* 锁定开关松开 (P407=HIGH) 后的延时关机: 5s 内重新锁定可取消, 超时关机
 *  lock_sw_read() 每 10ms 调用一次, 故 tick 数 = 秒数 × 100 */
#define LOCK_SUSPEND_TIMEOUT_S        (5U)
#define LOCK_SUSPEND_TIMEOUT_TICKS    (LOCK_SUSPEND_TIMEOUT_S * 100U)

/* 重新锁定后 LED 全亮展示时长 (ms), 随后按电量刷新 (复现开机效果) */
#define LOCK_LED_ALL_ON_MS            (500U)

/* ======================================================================
 *  激光 PWM 参数
 * ====================================================================== */
#define TIMER_PIN               GPT_IO_PIN_GTIOCB  /* 默认 PWM 输出引脚 */
#define GPT_PIN                 BSP_IO_PORT_02_PIN_12
#define MAX_INTENSITY           (100U)      /* 最大亮度百分比 */
#define MAX_DUTY_CYCLE          (1000U)     /* PWM 最大占空比 raw count */
#define LASER_DEFAULT_DUTY      (60U)       /* 激光默认 duty (开启时) */
#define LASER_IDLE_DUTY         (20U)       /* 激光空闲 duty (关闭时载波维持) */
#define LASER_CHANNEL_COUNT     (3U)        /* 激光通道数 */

/* 低电平阶段 duty 比例 (%)，用于 0-4° 和 10-90° 模式 */
#define LOW_PHASE_DUTY_PERCENT  (66U)

/* ======================================================================
 *  ADC 采样参数
 * ====================================================================== */
#define TOTAL_SAMPLES           (64U)       /* 单次采样总数 */
#define TRIM_SIDES              (16U)       /* 去掉两端各 16 个异常值 */
#define VALID_COUNT             (32U)       /* 取中间 32 个求均值 */

#define LD_TOTAL_SAMPLES        (64U)       /* LD 采样总数 */
#define LD_TRIM_SIDES           (16U)       /* LD 去掉两端各 16 */
#define LD_VALID_COUNT          (32U)       /* LD 有效采样数 */

#define HIGH_THRESHOLD          (1000U)     /* 方波高电平判定阈值 */

/* PWM 周期内 ADC 采样组数 */
#define SAMPLE_COUNT            (5U)

/* ======================================================================
 *  PID 及激光调光参数
 * ====================================================================== */
#define PD_FAULT_THRESHOLD      (40)       /* PD 低于此值判定激光管故障 */
#define PD_ABNORMAL_THRESHOLD   (3200)      /* PD 误差超过此值跳过本次调节 */

/* 电流采样换算系数 (x1000): 由采样电阻决定
 *   current_mA = |ld1_avg - ld2_avg|(mV) × 系数 / 1000 */
#define CURRENT_SENSE_GAIN_X1000    (3000)

/* 带电插拔激光管检测 — 激光运行中 PD≈0 且电流低于阈值 → 判被拔掉, 异常关机
 * 区分: 故障(PD低但电流仍存在) vs 插拔(PD低且电流≈0断路) */
#define LASER_HOTPLUG_CURRENT_MA    (40)    /* 电流低于此值(mA) 判插拔 */
#define LASER_HOTPLUG_CONFIRM_COUNT (2U)   /* 连续确认次数, 防上电建立期误判 */

/* 合法开启"电流建立期"覆盖的调光周期数 — 开启命令后前 N 个调光周期内的
 * 电流建立视为合法, 用于区分"合法开启"与"带电插入"。
 * laser_safety_hotplug_check() 每个调光周期递减 1, 减到 0 后若才检测到
 * 电流建立(>= LASER_HOTPLUG_CURRENT_MA) 则判带电插入。
 * 值越大对合法开启越宽容、对带电插入检测越迟钝; 建议略大于 H 通道从
 * 开启到电流稳定超过 LASER_HOTPLUG_CURRENT_MA 的实际调光周期数。 */
#define LASER_OPEN_ESTABLISH_CYCLES  (5U)

/* 电流超限消抖确认次数 — laser_adjust_duty() 每轮调光调用一次
 *  连续 5 次超过限流才锁存恒流模式，防止上电暂态尖峰误触发 */
#define CURRENT_LIMIT_CONFIRM_COUNT  (5U)

/* 各通道限流阈值 (mA)，超过后触发恒流保护 */
#define CURRENT_LIMIT_V2_MA    (350U)   /* V2(FRONT) 限流 280mA */
#define CURRENT_LIMIT_H_MA     (420U)   /* H(SIDE)   限流 400mA */
#define CURRENT_LIMIT_V1_MA    (350U)   /* V1(HORIZ) 限流 280mA */

/* 上电预检测 (EN1 拉高前) 电流阈值 (mA): 任一激光管电流超过此值则不拉 EN1 */
#define PRE_EN1_CURRENT_LIMIT_MA   (60U)

/*  电流采样合理性校验的低占空比阈值
 *  duty 低于此值时电流不可能超限；若此时仍读到超限电流，
 *  判定为电流采样异常（读数失真），跳过本次调光 */
#define CURRENT_SENSE_FAULT_DUTY_THRESHOLD  (80U)

/* ======================================================================
 *  UART 串口参数
 * ====================================================================== */
#define RX_BUFFER_SIZE          (32U)       /* UART 接收缓冲区大小 (字节) */
/* [FIX #3] WARNING: 明文密码，量产前务必更换或改为从 Flash 加载 */
#define CFG_PASSWORD            "123456"    /* AT 指令配置解锁密码 */
#define UART_MSG_BUF_SIZE       (64U)       /* UART 消息格式化缓冲区 */

/* ======================================================================
 *  IMU (IIM42351) 参数
 * ====================================================================== */
#define CALIBRATION_TRIALS      (32)       /* 校准采样次数 */

/** 角度阈值 (加速度计原始值) */
#define LIMIT_3_8_DEG           (1085)      /* 3.8° 阈值 */
#define LIMIT_4_DEG             (1143)      /* 4.0° 阈值 */
#define LIMIT_7_5_DEG           (2139)      /* 7.5° 阈值 */
#define LIMIT_10_DEG            (2846)      /* 10.0° 阈值 */

/** 姿态模式切换防抖阈值 — 需连续 N 次一致才切换 */
#define MODE_CHANGE_THRESHOLD   (4U)

/* ======================================================================
 *  Flash 存储参数
 * ====================================================================== */
#define FLASH_DF_BASE_ADDR      (0x40100000U)   /* RA2E1 Data Flash 基地址 */
#define FLASH_DF_BLOCK_SIZE     (64U)           /* Data Flash 块大小 (字节) */
#define FLASH_DF_WRITE_SIZE     (4U)            /* Data Flash 最小写入粒度 */
#define FLASH_BLOCK_CALIBRATION (FLASH_DF_BASE_ADDR) /* 校准数据存储块 */
#define FLASH_CALIB_MAGIC       (0xA5A5A5A5U)   /* 校准数据有效标识 */
#define FLASH_REF_MAGIC         (0x5A5A5A5AU)   /* 参考电压有效标识 */
#define FLASH_OFFSET_CALIB      (0U)            /* 校准数据块内偏移 */
#define FLASH_OFFSET_REF        (16U)           /* 参考电压块内偏移 (4 字节对齐) */


/* ======================================================================
 *  WDT 看门狗参数
 * ====================================================================== */

/* ======================================================================
 *  高温保护参数
 * ====================================================================== */
#define OVERHEAT_BLINK_COUNT        (6U)    /* 闪烁次数 */
#define OVERHEAT_ON_MS              (125U)  /* ON 半周期 ms */
#define OVERHEAT_OFF_MS             (125U)  /* OFF 半周期 ms */
#define OVERHEAT_CYCLE_MS           (OVERHEAT_ON_MS + OVERHEAT_OFF_MS)  /* 4Hz */

/* ======================================================================
 *  NTC 控制板温度保护 (ADC CH10)
 * ====================================================================== */
#define NTC_OVERHEAT_THRESHOLD      450     /* NTC ADC < 此值 → >100°C, 触发高温保护 */
#define NTC_LOW_TEMP_THRESHOLD      3150    /* NTC ADC > 此值 → <-7°C, 标记低温 */
#define NTC_LOW_TEMP_HYST           100     /* 低温恢复滞回 (ADC值), 需低于(阈值-回差)才退出 */
#define NTC_TEMP_CONFIRM_COUNT      5       /* 连续确认次数 (5×20ms=100ms消抖) */
#define NTC_LOW_TEMP_DUTY_PERCENT   85      /* 低温时激光占空比百分比 (85%) */
#define NTC_LOW_TEMP_REF_PERCENT    75      /* 低温时参考电压百分比 (75%), 独立于占空比 */

/* ======================================================================
 *  光功率自动校准参数
 *  协议: PC上位机读取光功率计 → 发AT+POW=当前值 → APP小步调电压 → 循环至目标
 *  通道: H(SIDE) / V1(HORIZ) / V2(FRONT)
 * ====================================================================== */
/* 目标光功率 (x100, 单位 0.01mW) */
#define POWCAL_TARGET_H         250     /* H  目标 2.50mW */
#define POWCAL_TARGET_V1        120     /* V1 目标 1.20mW */
#define POWCAL_TARGET_V2        120     /* V2 目标 1.20mW */

/* 步进阈值 (x100, 单位 0.01mW) */
#define POWCAL_THRESH_LARGE     50      /* |差值| > 0.50mW → 大步进 */
#define POWCAL_THRESH_MED       20      /* |差值| > 0.20mW → 中步进 */
#define POWCAL_THRESH_SMALL     10      /* 正误差阈值: 0.10mW, power>=target 且 power-target<=此值 → 到位 */

/* 步进电压值 (mV) */
#define POWCAL_STEP_LARGE_MV    200     /* 大步进电压 */
#define POWCAL_STEP_MED_MV      100     /* 中步进电压 */
#define POWCAL_STEP_SMALL_MV    50      /* 小步进电压 */

/* 参考电压限幅 */
#define POWCAL_VOLT_MAX_MV      3300    /* 参考电压上限 */
#define POWCAL_VOLT_MIN_MV      0       /* 参考电压下限 */

/* 保护阈值 */
#define POWCAL_PD_MIN_MV        30      /* PD 初始值低于此值禁止校准 */

/* ======================================================================
 *  sleep mode time
 * ====================================================================== */
#define NO_LASER_POWEROFF_S         (15U * 60U)   /* 900秒 15min */

#define SHUTDOWN_MAGIC_ADDR  ((volatile uint32_t *)0x20007FF0U)
#define SHUTDOWN_MAGIC_VALUE (0xDEADBEEFU)

#endif /* HAWKEYE_CONFIG_H_ */
