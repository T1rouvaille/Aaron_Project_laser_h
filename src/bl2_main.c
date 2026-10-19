/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2017-2020 Arm Limited.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal_data.h"
#include "comms/comms.h"
#include "menu.h"
#include "header.h"
#include "crc16.h"
#include "xmodem.h"

/* ------------------------------------------------------------------ */
/*  Simple hex print helpers (replaces snprintf for %X/%lX)            */
/* ------------------------------------------------------------------ */
static uint8_t * put_nibble(uint8_t *p, uint8_t v)
{
    v &= 0x0FU;
    *p = (uint8_t)(v < 10U ? ('0' + v) : ('A' + v - 10U));
    return p + 1U;
}

static uint8_t * put_hex8(uint8_t *p, uint8_t v)
{
    p = put_nibble(p, (uint8_t)(v >> 4U));
    p = put_nibble(p, v);
    return p;
}

static uint8_t * put_hex16(uint8_t *p, uint16_t v)
{
    p = put_hex8(p, (uint8_t)(v >> 8U));
    p = put_hex8(p, (uint8_t)(v & 0xFFU));
    return p;
}

static uint8_t * put_hex32(uint8_t *p, uint32_t v)
{
    p = put_hex16(p, (uint16_t)(v >> 16U));
    p = put_hex16(p, (uint16_t)(v & 0xFFFFU));
    return p;
}

static uint8_t * put_str(uint8_t *p, const char *s)
{
    while (*s) { *p++ = (uint8_t)*s++; }
    return p;
}

/* 电池低压阈值 (ADC 原始值, 14.5V) — 与 App hawkeye_config.h 保持一致 */
#define BOOT_BAT_LOW_ADC    (1825U)

/* 电源控制引脚 — 与 App bsp_led.h 保持一致 */
#define BOOT_Batt_OFF       R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_09, BSP_IO_LEVEL_LOW)
#define BOOT_Power_En_OFF   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_08, BSP_IO_LEVEL_LOW)
#define BOOT_EN1_OFF        R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_09_PIN_13, BSP_IO_LEVEL_LOW)

#define BOOT_Batt_ON       R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_09, BSP_IO_LEVEL_HIGH)
#define EN1_ON   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_09_PIN_13, BSP_IO_LEVEL_HIGH)
#define Power_En_ON   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_08, BSP_IO_LEVEL_HIGH)
/* 锁定开关 — 关机状态 P407=HIGH, 开机状态 P407=LOW */
#define BOOT_LOCK_SW_IS_OFF()                                           \
    ({                                                                   \
        bsp_io_level_t _lv;                                              \
        R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_07, &_lv);  \
        (_lv == BSP_IO_LEVEL_HIGH);                                      \
    })

/* ADC 扫描完成回调 — 设置标志位, 与 App bsp_adc.c 逻辑一致 */
volatile bool g_boot_scan_complete = false;
void adc_callback(adc_callback_args_t *p_args)
{
    (void)p_args;
    g_boot_scan_complete = true;
}

#if 0   //WangJin 2021.04.13
#include "bl2_util.h"
#include "target.h"
//#include "tfm_hal_device_header.h"
#include "Driver_Flash.h"
#include "mbedtls/memory_buffer_alloc.h"
#include "bootutil/bootutil_log.h"
#include "bootutil/image.h"
#include "bootutil/bootutil.h"
#include "flash_map_backend/flash_map_backend.h"
#include "boot_record.h"
#include "security_cnt.h"
#include "boot_hal.h"
#include "memory_buffer_alloc.h"
#endif
#if MCUBOOT_LOG_LEVEL > MCUBOOT_LOG_LEVEL_OFF
#include "uart_stdout.h"
#endif
#if defined(CRYPTO_HW_ACCELERATOR) || \
    defined(CRYPTO_HW_ACCELERATOR_OTP_PROVISIONING)
#include "crypto_hw.h"
#endif

/* Avoids the semihosting issue */
#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
__asm("  .global __ARM_use_no_argv\n");
#endif

#if defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_8M_BASE__)
/* Macros to pick linker symbols */
#define REGION(a, b, c) a##b##c
#define REGION_NAME(a, b, c) REGION(a, b, c)
#define REGION_DECLARE(a, b, c) extern uint32_t REGION_NAME(a, b, c)

REGION_DECLARE(Image$$, ARM_LIB_STACK, $$ZI$$Base);
#endif

//WangJin 2021.02.23
//#define   __ARM_LIB_STACK_start__   Image$$ARM_LIB_STACK$$ZI$$Base
#define   Image$$ARM_LIB_STACK$$ZI$$Base    "__ARM_LIB_STACK_start__"
//WangJin 2021.02.23

/* Flash device name must be specified by target */
//extern ARM_DRIVER_FLASH FLASH_DEV_NAME;

#define BL2_MBEDTLS_MEM_BUF_LEN 0x2000
/* Static buffer to be used by mbedtls for memory allocation */
//static uint8_t mbedtls_mem_buf[BL2_MBEDTLS_MEM_BUF_LEN];

struct arm_vector_table {
    uint32_t msp;
    uint32_t reset;
};

struct boot_rsp {
    /** A pointer to the header of the image to be executed. */
    const struct image_header *br_hdr;

    /**
     * The flash offset of the image to execute.  Indicates the position of
     * the image header within its flash device.
     */
    uint8_t br_flash_dev_id;
    uint32_t br_image_off;
};

int bl2_main(void);
void do_boot(struct boot_rsp *rsp);
void boot_jump_to_next_image(uint32_t reset_handler_addr);

/*!
 * \brief Chain-loading the next image in the boot sequence.
 *
 * This function calls the Reset_Handler of the next image in the boot sequence,
 * usually it is the secure firmware. Before passing the execution to next image
 * there is conditional rule to remove the secrets from the memory. This must be
 * done if the following conditions are satisfied:
 *  - Memory is shared between SW components at different stages of the trusted
 *    boot process.
 *  - There are secrets in the memory: KDF parameter, symmetric key,
 *    manufacturer sensitive code/data, etc.
 */
#if 0   //WangJin 2021.04.13
__attribute__((naked)) void boot_jump_to_next_image(uint32_t reset_handler_addr)
{
    __ASM volatile(
        ".syntax unified                 \n"
        "mov     r7, r0                  \n"
        "bl      boot_clear_bl2_ram_area \n" /* Clear RAM before jump */
        "movs    r0, #0                  \n" /* Clear registers: R0-R12, */
        "mov     r1, r0                  \n" /* except R7 */
        "mov     r2, r0                  \n"
        "mov     r3, r0                  \n"
        "mov     r4, r0                  \n"
        "mov     r5, r0                  \n"
        "mov     r6, r0                  \n"
        "mov     r8, r0                  \n"
        "mov     r9, r0                  \n"
        "mov     r10, r0                 \n"
        "mov     r11, r0                 \n"
        "mov     r12, r0                 \n"
        "mov     lr,  r0                 \n"
        "bx      r7                      \n" /* Jump to Reset_handler */
    );
}
#endif

/*!
 * \brief App-triggered firmware update via XMODEM protocol.
 *
 * Erases the entire App area, downloads new firmware via XMODEM,
 * writes CRC and OTA flag, then returns 1 on success, 0 on failure.
 * Called when App writes ENTER_UPDATE_MODE magic to OTA flag.
 */
int firmware_update_via_xmodem(void)
{
    volatile fsp_err_t err;
    uint8_t  buf[30];
    uint32_t CRC_rom;
    uint8_t  OTA_FLAG_END[4] = {0x55, 0x55, 0x55, 0x55};
    unsigned char xm_err;

    /* Blank check */
    comms_send((uint8_t *)"Blank checking the App area...\r\n", 33U);
    flash_result_t fresult;
    err = g_flash0.p_api->blankCheck(g_flash0.p_ctrl, APP_IMAGE_START_ADDRESS,
                                     (APP_IMAGE_END_ADDRESS - APP_IMAGE_START_ADDRESS), &fresult);
    if (FSP_SUCCESS != err)
    {
        comms_send((uint8_t *)"ERROR: Blank checking the App area\r\n", 37U);
        return 0;
    }

    if (fresult != FLASH_RESULT_BLANK)
    {
        comms_send((uint8_t *)"Erasing the App area...\r\n", 25U);
        ThreadsAndInterrupts(DISABLE);
        err = g_flash0.p_api->erase(g_flash0.p_ctrl, APP_IMAGE_START_ADDRESS, APP_IMAGE_NUM_BLOCKS);
        ThreadsAndInterrupts(RE_ENABLE);
        if (FSP_SUCCESS != err)
        {
            comms_send((uint8_t *)"ERROR: Erasing the App area\r\n", 31U);
            return 0;
        }
        comms_send((uint8_t *)"App area erased\r\n", 18U);
    }
    else
    {
        comms_send((uint8_t *)"App area blank\r\n", 17U);
    }

    /* XModem download and flash programming */
    comms_send((uint8_t *)"Start Xmodem transfer...\r\n", 28U);
    xm_err = XmodemDownloadAndProgramFlash(APP_IMAGE_START_ADDRESS);
    if (XM_OK != xm_err)
    {
        switch (xm_err)
        {
            case XM_ADDRESS_ERROR:
                comms_send((uint8_t *)"ERROR: Flash address invalid\r\n", 32U);
                break;
            case XM_TIMEOUT:
                comms_send((uint8_t *)"ERROR: Timeout during Xmodem download\r\n", 42U);
                break;
            case XM_PROG_FAIL:
                comms_send((uint8_t *)"ERROR: Flash programming error\r\n", 34U);
                break;
            default:
                /* Build "ERROR: unknown (0xXX)\r\n" */
                buf[0] = 'E'; buf[1] = 'R'; buf[2] = 'R'; buf[3] = 'O'; buf[4] = 'R';
                buf[5] = ':'; buf[6] = ' '; buf[7] = 'u'; buf[8] = 'n'; buf[9] = 'k';
                buf[10]= 'n'; buf[11]= 'o'; buf[12]= 'w'; buf[13]= 'n'; buf[14]= ' ';
                buf[15]= '('; buf[16]= '0'; buf[17]= 'x';
                buf[18]= (uint8_t)("0123456789ABCDEF"[(xm_err >> 4) & 0x0F]);
                buf[19]= (uint8_t)("0123456789ABCDEF"[xm_err & 0x0F]);
                buf[20]= ')'; buf[21]= '\r'; buf[22]= '\n';
                comms_send(buf, 23U);
                break;
        }
        return 0;
    }

    comms_send((uint8_t *)"Flash image download successful\r\n", 33U);

    /* Calculate CRC of downloaded image */
    CRC_rom = calcrc(APP_IMAGE_START_ADDRESS, (CRC_ADDRESS - APP_IMAGE_START_ADDRESS));
    /* Build "CRC: XXXX\r\n" */
    buf[0] = 'C'; buf[1] = 'R'; buf[2] = 'C'; buf[3] = ':'; buf[4] = ' ';
    buf[5] = (uint8_t)("0123456789ABCDEF"[(CRC_rom >> 12) & 0x0F]);
    buf[6] = (uint8_t)("0123456789ABCDEF"[(CRC_rom >>  8) & 0x0F]);
    buf[7] = (uint8_t)("0123456789ABCDEF"[(CRC_rom >>  4) & 0x0F]);
    buf[8] = (uint8_t)("0123456789ABCDEF"[ CRC_rom        & 0x0F]);
    buf[9] = '\r'; buf[10]= '\n';
    comms_send(buf, 11U);

    /* Write CRC to flash */
    {
        uint32_t crc_data = (uint32_t)CRC_rom;
        ThreadsAndInterrupts(DISABLE);
        err = g_flash0.p_api->write(g_flash0.p_ctrl, (uint32_t)&crc_data, CRC_ADDRESS, 4);
        ThreadsAndInterrupts(RE_ENABLE);
        if (FSP_SUCCESS != err)
        {
            comms_send((uint8_t *)"Write CRC to flash failed\r\n", 28U);
            return 0;
        }
    }

    /* Set OTA flag */
    comms_send((uint8_t *)"Setting the OTA flag\r\n", 24U);
    ThreadsAndInterrupts(DISABLE);
    err = g_flash0.p_api->write(g_flash0.p_ctrl, (uint32_t)&OTA_FLAG_END[0], OTA_FLAG_END_ADDRESS, 4);
    ThreadsAndInterrupts(RE_ENABLE);
    if (FSP_SUCCESS != err)
    {
        comms_send((uint8_t *)"Set OTA flag failed\r\n", 23U);
        return 0;
    }

    comms_send((uint8_t *)"OTA update complete, OTA flag set\r\n", 38U);
    return 1;
}

void do_boot(struct boot_rsp *rsp)
{
    /* Clang at O0, stores variables on the stack with SP relative addressing.
     * When manually set the SP then the place of reset vector is lost.
     * Static variables are stored in 'data' or 'bss' section, change of SP has
     * no effect on them.
     */
    static struct arm_vector_table vt;
    uintptr_t flash_base;

    (void)rsp;

    flash_base = APP_IMAGE_START_ADDRESS;

    /* Read app's vector table from flash into static RAM variable */
    vt.msp   = *(uint32_t *)(flash_base);
    vt.reset = *(uint32_t *)(flash_base + 4);

    /* Close UART, timer and IOPORT before jumping to App */
    g_uart0.p_api->close(g_uart0.p_ctrl);
    g_timer0.p_api->close(g_timer0.p_ctrl);
    R_IOPORT_Close(&g_ioport_ctrl);

#if 0   //WangJin 2021.04.13
    /* The beginning of the image is the ARM vector table, containing
     * the initial stack pointer address and the reset vector
     * consecutively. Manually set the stack pointer and jump into the
     * reset vector
     */
    rc = flash_device_base(rsp->br_flash_dev_id, &flash_base);
    assert(rc == 0);

    if (rsp->br_hdr->ih_flags & IMAGE_F_RAM_LOAD) {
       /* The image has been copied to SRAM, find the vector table
        * at the load address instead of image's address in flash
        */
        vt = (struct arm_vector_table *)(rsp->br_hdr->ih_load_addr +
                                         rsp->br_hdr->ih_hdr_size);
    } else {
        /* Using the flash address as not executing in SRAM */
        vt = (struct arm_vector_table *)(flash_base +
                                         rsp->br_image_off +
                                         rsp->br_hdr->ih_hdr_size);
    }

    rc = FLASH_DEV_NAME.Uninitialize();
    if(rc != ARM_DRIVER_OK) {
        BOOT_LOG_ERR("Error while uninitializing Flash Interface");
    }
#endif

#if MCUBOOT_LOG_LEVEL > MCUBOOT_LOG_LEVEL_OFF
    stdio_uninit();
#endif

#if defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_8M_BASE__)
    /* Restore the Main Stack Pointer Limit register's reset value
     * before passing execution to runtime firmware to make the
     * bootloader transparent to it.
     */
    __set_MSPLIM(0);
#endif
#if defined(__ARMCC_VERSION)
    __set_MSP(vt.msp);
    __DSB();
    __ISB();

    boot_jump_to_next_image(vt.reset);
#else
    SCB->VTOR = (flash_base & 0x1FFFFF80);
    __DSB();

    /* Disable MSP monitoring  */
#if BSP_FEATURE_BSP_HAS_SP_MON    //Added WangJin 2021.02.22
    R_MPU_SPMON->SP[0].CTL = 0;
    while(R_MPU_SPMON->SP[0].CTL != 0);
#endif

    __set_MSP(vt.msp);

    ((void (*)()) vt.reset)();
#endif
}

int bl2_main(void)
{
#if defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_8M_BASE__)
    uint32_t msp_stack_bottom =
            (uint32_t)&REGION_NAME(Image$$, ARM_LIB_STACK, $$ZI$$Base);
#endif

    struct boot_rsp rsp;

#if defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_8M_BASE__)
    __set_MSPLIM(msp_stack_bottom);
#endif

    /* ==================================================================
     *  锁定开关检测 — 关机状态直接断电，不跳转 APP
     *  必须在 Phase 0 之前执行，防止 APP 有效时跳过检测
     *  P407=HIGH → 锁定开关 OFF → 切断电源, 死循环等电容放电
     *  P407=LOW  → 锁定开关 ON  → 继续正常开机流程
     * ================================================================== */
    if (BOOT_LOCK_SW_IS_OFF())
    {
        LED1_OFF; LED2_OFF; LED3_OFF;
        BOOT_Batt_OFF;
        BOOT_Power_En_OFF;
        BOOT_EN1_OFF;
        while (1)
        {
            /* 不喂狗, CPU休眠等电容放电, 自然断电 */
            __WFI();
        }
    }
    /* IBOX Honcho PR3 早期握手检测 (9600 波特率) */
    ibox_try_handshake();
    /* ==================================================================
     *  电池低压检查 — 与 App hal_entry.c 逻辑完全一致
     *  电压 < BAT_BOOT_MIN_ADC 直接断电, 无 UART 输出
     * ================================================================== */
    {
        fsp_err_t adc_err;
        adc_err = R_ADC_Open(&g_adc0_ctrl, &g_adc0_cfg);
        if (FSP_SUCCESS == adc_err)
        {
            adc_err = R_ADC_ScanCfg(&g_adc0_ctrl, &g_adc0_channel_cfg);
            if (FSP_SUCCESS == adc_err)
            {
                R_ADC_ScanStart(&g_adc0_ctrl);

                /* 4 次采样取平均, 每次等待 ADC 回调 scan_complete */
                uint32_t bat_sum = 0;
                for (int i = 0; i < 4; i++)
                {
                    g_boot_scan_complete = false;
                    while (!g_boot_scan_complete)
                    {
                        /* 等待 ADC 扫描完成中断 */
                    }
                    g_boot_scan_complete = false;
                    uint16_t tmp = 0;
                    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_9, &tmp);
                    bat_sum += tmp;
                }
                R_ADC_ScanStop(&g_adc0_ctrl);
                uint16_t bat_adc_boot = (uint16_t)(bat_sum / 4);

                if (bat_adc_boot < BOOT_BAT_LOW_ADC)
                {
                    /* 电压不够, 关闭所有外设供电, 等电容放完电自然断电 */
                    R_ADC_Close(&g_adc0_ctrl);
                    BOOT_Batt_OFF;
                    BOOT_Power_En_OFF;
                    BOOT_EN1_OFF;
                    while (1)
                    {
                        /* 不喂狗, CPU休眠等电容放电, IWDT超时复位后电压仍低则再次进入此处 */
                        __WFI();
                    }
                }
            }
            R_ADC_Close(&g_adc0_ctrl);
        }
        /* ADC 初始化失败不阻塞, 继续正常开机流程 */
    /* ==================================================================
     *  Phase 0 前置 — WDT 复位检测: 通过 SRAM 魔数区分"主动关机"和"APP 崩溃"
     *  - 魔数匹配: APP 调用 system_power_off_with_reason() → 保持断电
     *  - 魔数不匹配: APP 死机/卡死 → 正常重启恢复
     *  RSTSR1 bit1 = WDTRF (WDT 复位标志)
     * ================================================================== */
    #define BL_SHUTDOWN_MAGIC_ADDR  ((volatile uint32_t *)0x20007FF0U)
    #define BL_SHUTDOWN_MAGIC_VALUE (0xDEADBEEFU)

    if ((R_SYSTEM->RSTSR1 & R_SYSTEM_RSTSR1_WDTRF_Msk) &&
        (*BL_SHUTDOWN_MAGIC_ADDR == BL_SHUTDOWN_MAGIC_VALUE))
    {
        *BL_SHUTDOWN_MAGIC_ADDR = 0x00000000U;  /* 清除标记 */
        R_SYSTEM->RSTSR0 = 0x00U;
        R_SYSTEM->RSTSR1 = 0x0000U;
        BOOT_Power_En_OFF;
        BOOT_Batt_OFF;
        while (1) { __WFI(); }      /* APP 主动关机, 不重新上电 */
    }


    /* ==================================================================
     *  Phase 0 — 秒跳检查: APP 向量表有效则直接跳转, 不输出任何 UART.
     *  涵盖 XMODEM (有 OTA flag) 和 IBOX LOAD_FLASH (无 OTA flag).
     *  Valid SP: RA2E1 RAM 0x20000000-0x20004000 (16KB)
     *  Valid PC: APP flash 0x00004000-0x00020000, Thumb bit set
     * ================================================================== */
    {
        uint32_t app_sp = *(uint32_t *)APP_IMAGE_START_ADDRESS;
        uint32_t app_pc = *(uint32_t *)(APP_IMAGE_START_ADDRESS + 4U);

        if (app_sp >= 0x20000000U && app_sp < 0x20004000U &&
            app_pc >= APP_IMAGE_START_ADDRESS && app_pc < APP_IMAGE_END_ADDRESS &&
            (app_pc & 1U) == 1U)
        {
            Power_En_ON;
            BOOT_Batt_ON;
            do_boot(&rsp);
        }
    }

    /* ==================================================================
     *  APP 无效 — 进入完整 bootloader 流程 (需要外设电源)
     * ================================================================== */
    Power_En_ON;
    BOOT_Batt_ON;



    /* IBOX 未检测到 → 切换到 115200 波特率, 后续所有 UBOX 功能均在此速率下运行 */
    comms_set_baud(UART_BAUD_FAST);


 }

#if MCUBOOT_LOG_LEVEL > MCUBOOT_LOG_LEVEL_OFF
    stdio_init();
#endif

    /* UART, Timer, Flash 已在 hal_entry() 中完成初始化 */

    /* Check if a valid App image exists, jump directly if OTA flag is set */
    uint32_t flag_end = *(uint32_t *)OTA_FLAG_END_ADDRESS;
    if( 0x55555555 == flag_end ){
        uint32_t CRC_rom = calcrc(APP_IMAGE_START_ADDRESS, (CRC_ADDRESS - APP_IMAGE_START_ADDRESS));
        uint16_t CRC_ccitt = (unsigned short)(*(unsigned int *)CRC_ADDRESS);
        if( CRC_rom == CRC_ccitt ){
            do_boot(&rsp);
        }
        else{
            comms_send((uint8_t *)"CRC NG, entering menu\r\n", 22U);
        }
    }

    /* Check if App requested firmware update (wrote magic to OTA flag) */
    if (ENTER_UPDATE_MODE == flag_end)
    {
        if (firmware_update_via_xmodem())
        {
            /* Success - OTA flag is set, jump to new App */
            do_boot(&rsp);
            /* Should not return */
        }
        /* Failed - fall through to menu */
    }

//    BOOT_LOG_INF("Starting bootloader");
    menu();

#if 0   //WangJin 2021.04.13
    /* Perform platform specific initialization */
    if (boot_platform_init() != 0) {
        while (1)
            ;
    }
    /* Initialise the mbedtls static memory allocator so that mbedtls allocates
     * memory from the provided static buffer instead of from the heap.
     */
    mbedtls_memory_buffer_alloc_init(mbedtls_mem_buf, BL2_MBEDTLS_MEM_BUF_LEN);

#ifdef CRYPTO_HW_ACCELERATOR
    rc = crypto_hw_accelerator_init();
    if (rc) {
        BOOT_LOG_ERR("Error while initializing cryptographic accelerator.");
        while (1);
    }
#endif /* CRYPTO_HW_ACCELERATOR */

#ifndef MCUBOOT_USE_UPSTREAM
    rc = boot_nv_security_counter_init();
    if (rc != 0) {
        BOOT_LOG_ERR("Error while initializing the security counter");
        while (1)
            ;
    }
#endif /* !MCUBOOT_USE_UPSTREAM */

    rc = boot_go(&rsp);
    if (rc != 0) {
        BOOT_LOG_ERR("Unable to find bootable image");
        while (1)
            ;
    }

#ifdef CRYPTO_HW_ACCELERATOR
    rc = crypto_hw_accelerator_finish();
    if (rc) {
        BOOT_LOG_ERR("Error while uninitializing cryptographic accelerator.");
        while (1);
    }
#endif /* CRYPTO_HW_ACCELERATOR */

/* This is a workaround to program the TF-M related cryptographic keys
 * to CC312 OTP memory. This functionality is independent from secure boot,
 * this is usually done in the factory floor during chip manufacturing.
 */
#ifdef CRYPTO_HW_ACCELERATOR_OTP_PROVISIONING
    BOOT_LOG_INF("OTP provisioning started.");
    rc = crypto_hw_accelerator_otp_provisioning();
    if (rc) {
        BOOT_LOG_ERR("OTP provisioning FAILED: 0x%X", rc);
        while (1);
    } else {
        BOOT_LOG_INF("OTP provisioning succeeded. TF-M won't be loaded.");

        /* We don't need to boot - the only aim is provisioning. */
        while (1);
    }
#endif /* CRYPTO_HW_ACCELERATOR_OTP_PROVISIONING */

    BOOT_LOG_INF("Bootloader chainload address offset: 0x%x",
                 rsp.br_image_off);
    BOOT_LOG_INF("Jumping to the first image slot");
#endif
    do_boot(&rsp);

    return 0;   //Should never be here
//    BOOT_LOG_ERR("Never should get here");
//    menu();
}
