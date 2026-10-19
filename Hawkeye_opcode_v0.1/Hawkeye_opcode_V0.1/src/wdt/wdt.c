/*
 * wdt.c
 *
 *  Created on: 2026年3月3日
 *      Author: AT403
 */


/**
 * @file    wdt.c
 * @brief   WDT 保护模块实现 — 主循环直接喂狗，超时直接复位
 *          适用于 RA2E1 + FSP
 */

#include "wdt/wdt.h"
#include "r_wdt_api.h"
#include <stdio.h>
#include "debug_uart/bsp_debug_uart.h"
#include "hawkeye_config.h"
/* -----------------------------------------------------------------------
 * RA2E1 复位状态寄存器（硬件实际定义）
 *
 *   RSTSR0 (0x40000410, 8-bit):
 *     bit0  PORF     上电复位
 *     bit1  LVD0RF   电压监测0复位
 *     bit2  LVD1RF   电压监测1复位
 *     bit3  LVD2RF   电压监测2复位
 *     bit7  DPSRSTF  深度软件待机复位
 *
 *   RSTSR1 (0x400000C0, 16-bit):
 *     bit0  IWDTRF   独立看门狗复位
 *     bit1  WDTRF    WDT 复位
 *     bit2  SWRF     软件复位
 * ----------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
 * 内部：读取并打印复位原因，然后清除寄存器标志
 * ----------------------------------------------------------------------- */
static void _report_reset_reason(void)
{
    uint8_t  rstsr0 = R_SYSTEM->RSTSR0;
    uint16_t rstsr1 = R_SYSTEM->RSTSR1;

    if (rstsr1 & R_SYSTEM_RSTSR1_WDTRF_Msk)
    {
        uart9_send_blocking("+RESP:RESET=WDT\r\n");
    }
    else if (rstsr1 & R_SYSTEM_RSTSR1_IWDTRF_Msk)
    {
        uart9_send_blocking("+RESP:RESET=IWDT\r\n");
    }
    else if (rstsr1 & R_SYSTEM_RSTSR1_SWRF_Msk)
    {
        uart9_send_blocking("+RESP:RESET=SW\r\n");
    }
    else if (rstsr0 & R_SYSTEM_RSTSR0_PORF_Msk)
    {
        uart9_send_blocking("+RESP:RESET=POR\r\n");
    }
    else
    {
        char buf[48];
        snprintf(buf, sizeof(buf), "+RESP:RESET=OTHER,R0=0x%02X,R1=0x%04X\r\n",
                 rstsr0, rstsr1);
        uart9_send_blocking(buf);
    }

    /* 清除复位标志（写 0 清零），避免干扰下次上电判断 */
    R_SYSTEM->RSTSR0 = 0x00u;
    R_SYSTEM->RSTSR1 = 0x0000u;
}

/* -----------------------------------------------------------------------
 * 公共 API 实现
 * ----------------------------------------------------------------------- */

void wdt_init(void)
{
    /* 1. 打印上次复位原因 */
    _report_reset_reason();

    /* 2. 打开 WDT 外设 */
    fsp_err_t err = R_WDT_Open(g_wdt0.p_ctrl, g_wdt0.p_cfg);
    if (FSP_SUCCESS != err && FSP_ERR_ALREADY_OPEN != err)
    {
        uart9_send_blocking("+RESP:WDT_OPEN_FAIL\r\n");
        return;
    }

    /* 3. 首次 Refresh 启动计数器 */
    R_WDT_Refresh(g_wdt0.p_ctrl);

    //uart9_send_blocking("+RESP:WDT_OK\r\n");
}

void wdt_feed(void)
{
    R_WDT_Refresh(g_wdt0.p_ctrl);
}

void wdt_force_reset(const char *reason)
{
    if (reason)
    {
        uart9_send_blocking("+RESP:FATAL,");
        uart9_send_blocking(reason);
        uart9_send_blocking("\r\n");
    }
    NVIC_SystemReset();
}
