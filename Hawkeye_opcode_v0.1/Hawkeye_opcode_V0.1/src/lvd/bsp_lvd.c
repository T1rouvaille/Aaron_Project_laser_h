/*
 * bsp_lvd.c
 *
 *  异常断电（拔电池）掉电抢救模块 — 实现
 *
 *  Created on: 2026年11月3日
 *      Author: AT403
 */

/**
 * @file    bsp_lvd.c
 * @brief   掉电抢救实现。
 *
 *  原理：RA2E1 的 LVD1（电压监测1）监测 VCC(3.3V 轨)。拔电池后 BUCK 停止，
 *        3.3V 轨靠电容放电，跌到阈值(~2.90V)时 LVD1 触发 NMI（非屏蔽中断）。
 *        NMI 回调里先砍掉激光/LED 负载（把 ~100mA 降到 ~15mA，大幅延长
 *        放电时间），再调用 flash_save_stats() 写回统计，最后电压继续掉到
 *        POR 复位。RA2E1 Data Flash 最低擦写电压 1.8V，抢救窗口充足。
 *
 *  LVD1 走 NMI 路径（非普通外设 IRQ），FSP 已实现 NMI_Handler 分发：
 *    R_BSP_GroupIrqWrite(BSP_GRP_IRQ_LVD1, cb) 注册回调
 *    R_ICU->NMIER  bit2(LVD1EN) 使能 NMI 源
 *    无需修改 vector_data.c，无需重新生成 FSP 配置。
 */

#include "hal_data.h"
#include "led/bsp_led.h"
#include "timer_pwm.h"
#include "flash/flash.h"
#include "stats/bsp_stats.h"
#include "lvd/bsp_lvd.h"

/* ======================================================================
 *  常量
 * ====================================================================== */

/* LVD1 掉电检测阈值（已核对 RA2E1 datasheet R01DS0386EJ0150 Table 2.47）。
 *   Note 3：Vdet1_# 的 # = LVDLVLR.LVD1LVL[4:0] 的值（十六进制）。
 *   下降沿（fall，掉电方向）检测电压档位（typ）：
 *     0x4=3.10V  0x5=3.00V  0x6=2.90V  0x7=2.79V  0x8=2.68V
 *     0x9=2.58V  0xA=2.48V  0xB=2.20V  0xC~0xF <2.1V
 *   选 0x6(Vdet1_6=2.90V)：触发早 → 砍负载后到 flash 最低 1.8V 的窗口
 *   更大（~34ms），且离 3.3V 有 0.4V 余量，正常纹波不误触发。
 *   若 3.3V 轨激光全开瞬态跌落明显，可降为 0x7(2.79V) 或 0x8(2.68V)。 */
#define LVD1_THRESHOLD_2V9   (0x06u)

/* LVD1CR0 bit3 为保留位（头文件未命名）。RA2E1 UM §7.2.3 注明
 * 「读值未定义，写值应为 1」。写入 0 虽通常不影响当前硅片功能，
 * 但为符合手册要求、避免未定义行为，初始化时显式置 1。 */
#define LVD1CR0_RSV_BIT3     (0x08u)

/* ======================================================================
 *  NMI 回调：砍负载 → 写统计
 * ====================================================================== */

/**
 * @brief LVD1 掉电 NMI 回调（由 FSP NMI_Handler 分发，退出时自动清 NMICLR）。
 *        注意：NMI 优先级最高，__disable_irq() 屏蔽不了它——这正是掉电时
 *        唯一能在 flash_buf_flush() 关中断窗口内打断 CPU 的入口。
 */
static void lvd1_powerloss_cb(bsp_grp_irq_t irq)
{
    (void)irq;

    /* 1. 先砍负载：关激光总使能 + 三路 duty 清零 + 关 LED。
     *    电流从 ~100mA 降到 ~15mA，30ms 内 3.3V 只掉不到 1V，仍 > 1.8V。 */
    EN1_OFF;
    set_laser1_200k_intensity(0, TIMER_PIN);
    set_laser2_200k_intensity(0, TIMER_PIN);
    set_laser3_200k_intensity(0, GPT_IO_PIN_GTIOCA);
    set_laser1_8470_intensity(0, GPT_IO_PIN_GTIOCA);
    set_laser2_8470_intensity(0, TIMER_PIN);
    set_laser3_8470_intensity(0, GPT_IO_PIN_GTIOCA);
    LED1_OFF;
    LED2_OFF;
    LED3_OFF;

    /* 2. 只存统计，不记关机原因（按需求决定，避免改动关机日志语义）。 */
    flash_save_stats(stats_get_ptr());
}

/* ======================================================================
 *  初始化
 * ====================================================================== */

void lvd_powerloss_init(void)
{
    /* 注册 NMI 回调（FSP NMI_Handler 分发，回调后自动清 NMICLR）。 */
    R_BSP_GroupIrqWrite(BSP_GRP_IRQ_LVD1, lvd1_powerloss_cb);

    /* ---- 段1：配置阈值/检测方向，开 LVD1（此时不开中断，避免稳定期误触发） ---- */
    R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_LVD);

    /* 改阈值前必须先关 LVD1 */
    R_SYSTEM->LVCMPCR &= (uint8_t)~R_SYSTEM_LVCMPCR_LVD1E_Msk;

    /* 设掉电检测阈值 */
    R_SYSTEM->LVDLVLR = (uint8_t)((R_SYSTEM->LVDLVLR & ~R_SYSTEM_LVDLVLR_LVD1LVL_Msk)
                                  | LVD1_THRESHOLD_2V9);

    /* 检测方向：下降沿 VCC<Vdet1（拔电池掉电）。
     *   LVD1CR1 = IRQSEL=0(NMI) | IDTSEL=01(下降沿) = 0x01。
     *   ⚠️ IDTSEL: 00=上升沿 01=下降沿 10=升降都检 11=禁止，切勿设 00。 */
    R_SYSTEM->LVD1CR1 = 0x01u;

    /* 先只开比较输出(CMPE)，不开中断/复位(RIE=0)；保留位 bit3 写 1 */
    R_SYSTEM->LVD1CR0 = (uint8_t)(R_SYSTEM_LVD1CR0_CMPE_Msk | LVD1CR0_RSV_BIT3);

    /* 开 LVD1 */
    R_SYSTEM->LVCMPCR |= R_SYSTEM_LVCMPCR_LVD1E_Msk;

    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_LVD);

    /* 等 LVD1 稳定 t(d(E-A)) */
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);

    /* ---- 段2：LVD1 稳定后，清检测标志并使能中断 ---- */
    R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_LVD);
    R_SYSTEM->LVD1SR = 0x00u;                          /* 清 DET 标志（写 0） */
    R_SYSTEM->LVD1CR0 |= R_SYSTEM_LVD1CR0_RIE_Msk;    /* RIE=1 使能中断。
                                                          ⚠️ RI 必须保持 0 = 中断模式（RI=1 是复位，绝不能用） */
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_LVD);

    /* ---- 使能 NMI 源（NMIER bit2 = LVD1EN） ---- */
    R_ICU->NMIER |= (uint16_t)(1u << BSP_GRP_IRQ_LVD1);
}
