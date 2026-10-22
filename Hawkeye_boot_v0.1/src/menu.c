/*
 * menu.c — ABS Protocol Bootloader for RA2E1
 *
 * 协议行为与 Honcho PR3 IBOX protocol 对齐:
 *   - HANDSHAKE ('B'):  回显 'B'
 *   - IDENTIFY  ('I'):  发送 'I' + 0x04 + 0x45 + ETX (4 bytes)
 *   - SET_ADDRESS ('D'): 先回显 'D', 再收 2 字节地址, 最后发 ETX
 *   - READ_FLASH ('N'):  不读任何字节, 根据已存地址响应:
 *         addr=0xFFFF → 版本信息 (88 bytes Honcho 格式)
 *         addr=0xFFFE → 写校准请求 flag 到 Data Flash, APP 下次启动执行校准
 *         其他        → 回显 'N'
 *   - READ_EEPROM ('F'): 不读任何字节, 根据已存地址响应:
 *         addr=0xFFFF → 发送 APP Data Flash datalog (192 bytes, 进 IBOX 时已缓存)
 *         其他        → 仅发 ETX
 *   - CHANGE_BAUD ('*'): 回显 '*', 切高速波特率
 *
 * Bootloader 特有命令 (Honcho 未实现, RA2E1 保留):
 *   - LOAD_FLASH   ('M'): 写入 Flash 页面, 含 auto-erase
 *
 * Flash page size : 512 bytes
 * Erase block size: 2048 bytes (RA2E1 Code Flash minimum erase unit)
 * Baud rate       : 9600 (slow), 28800 (fast)
 *
 * APP area  : 0x00004000 - 0x0001FFFF  (112 KB)
 */

#include "menu.h"
#include "header.h"
#include "comms/comms.h"
#include "crc16.h"
#include "hal_data.h"
#include <string.h>


/* ------------------------------------------------------------------ */
/*  boot_rsp — mirrors definition in bl2_main.c                        */
/* ------------------------------------------------------------------ */
struct image_header;   /* opaque, not needed in this file */

struct boot_rsp {
    const struct image_header *br_hdr;
    uint8_t  br_flash_dev_id;
    uint32_t br_image_off;
};

/* ------------------------------------------------------------------ */
/*  Constants                                                           */
/* ------------------------------------------------------------------ */
#define RX_TIMEOUT_MS       5U
#define NAK                 0x15U

/* Module ID — Honcho PR3 format (8 bytes × 2) */
static const uint8_t g_module_id[BYTES_MODULE_ID]        = MODULE_ID_FIRST;
static const uint8_t g_module_id_second[BYTES_MODULE_ID] = MODULE_ID_SECOND;

/* Honcho PR3 personalization strings */
static const char g_tool_part_number[BYTES_PART_NUMBER]    = G_TOOL_PART_NUMBER;
static const char g_personalization[BYTES_PERSONALIZATION] = G_PERSONALIZATION;

/* ------------------------------------------------------------------ */
/*  Module state                                                        */
/* ------------------------------------------------------------------ */
static uint32_t g_target_addr = APP_IMAGE_START_ADDRESS;
static uint32_t g_target_offset = 0U;
static uint8_t  g_addr_high     = 0U;
static uint8_t  g_addr_low      = 0U;
static uint8_t  g_datalog_buf[DATALOG_SIZE];
static bool     g_timer_enabled = true;   /* 对齐 abs: 握手后关定时器 */

/* ------------------------------------------------------------------ */
/*  Forward declarations                                                */
/* ------------------------------------------------------------------ */
extern void do_boot(struct boot_rsp *rsp);

static void send_nak(void);
static fsp_err_t recv_byte(uint8_t *out);
static uint16_t checksum16(const uint8_t *buf, uint16_t len);
static bool addr_in_app_range(uint32_t addr, uint32_t len);
static fsp_err_t erase_block_if_needed(uint32_t addr);

static void cmd_handshake(void);
static void cmd_identify(void);
static void cmd_read_eeprom(void);
static void cmd_read_flash(void);
static void cmd_read_version_info(void);
static void cmd_load_flash(void);
static void cmd_set_address(void);
static void cmd_change_baud(void);
static void try_boot(void);

/* ------------------------------------------------------------------ */
/*  Simple hex print helpers (replaces snprintf for %X/%lX)            */
/* ------------------------------------------------------------------ */

/* Write a single hex nibble into buf, return pointer after it */
static uint8_t * put_nibble(uint8_t *p, uint8_t v)
{
    v &= 0x0FU;
    *p = (uint8_t)(v < 10U ? ('0' + v) : ('A' + v - 10U));
    return p + 1U;
}

/* Write 2-digit hex (uint8) */
static uint8_t * put_hex8(uint8_t *p, uint8_t v)
{
    p = put_nibble(p, (uint8_t)(v >> 4U));
    p = put_nibble(p, v);
    return p;
}

/* Write 4-digit hex (uint16) */
static uint8_t * put_hex16(uint8_t *p, uint16_t v)
{
    p = put_hex8(p, (uint8_t)(v >> 8U));
    p = put_hex8(p, (uint8_t)(v & 0xFFU));
    return p;
}

/* Write 8-digit hex (uint32) */
static uint8_t * put_hex32(uint8_t *p, uint32_t v)
{
    p = put_hex16(p, (uint16_t)(v >> 16U));
    p = put_hex16(p, (uint16_t)(v & 0xFFFFU));
    return p;
}

/* Write a literal string, return pointer after it */
static uint8_t * put_str(uint8_t *p, const char *s)
{
    while (*s) { *p++ = (uint8_t)*s++; }
    return p;
}

/* ================================================================== */
/*  IBOX Honcho PR3 protocol loop (entered via 'B' from UBOX menu)      */
/* ================================================================== */
static void honcho_loop(void)
{
    uint8_t cmd;
    fsp_err_t err;

    /* 握手已由 UBOX 菜单完成, 此后所有读取无限等待, 不再超时返回. */
    g_timer_enabled = false;
    g_timer0.p_api->stop(g_timer0.p_ctrl);

    /* 进入 IBOX 时一次性从 Data Flash 读取 APP datalog (192 bytes) 到 RAM 缓存,
     * 后续 'F' READ_EEPROM 直接从缓存 memcpy, 无需重复读 Flash. */
    memcpy(g_datalog_buf, (const void *)FLASH_DF_BASE_ADDR, DATALOG_SIZE);

    while (1)
    {
        err = recv_byte(&cmd);

        if (err != FSP_SUCCESS)
        {
            /* Timeout: return to UBOX menu (UBOX handles APP jump itself) */
            return;
        }

        switch (cmd)
        {
            case COMMAND_HANDSHAKE:     cmd_handshake();     break;
            case COMMAND_IDENTIFY:      cmd_identify();      break;
            case COMMAND_READ_EEPROM:   cmd_read_eeprom();   break;
            case COMMAND_READ_FLASH:    cmd_read_flash();    break;
            case COMMAND_LOAD_FLASH:    cmd_load_flash();    break;
            case COMMAND_SET_ADDRESS:   cmd_set_address();   break;
            case COMMAND_CHANGE_BAUD:   cmd_change_baud();  break;
            case COMMAND_NACK:          try_boot();          break;  /* NACK: exit and boot to APP */

            default:
                send_nak();
                break;
        }
    }
}

/* ================================================================== */
/*  IBOX early handshake — called from bl2_main() BEFORE any delays       */
/*                                                                       */
/*  在 bl2_main() 最早阶段尝试 IBOX 协议握手, 避免电池 ADC 检查等        */
/*  40-150ms 的前置延迟导致 IBOX 工具超时.                               */
/*                                                                       */
/*  流程:                                                                 */
/*    1. 发送 "[BL v2.1] ABS ready\r\n" 提示符                            */
/*    2. 等待 100ms 接收 'B'                                              */
/*    3. 收到 'B' → 回显 → 进入 honcho_loop() (永不返回)                 */
/*    4. 超时 → 静默返回, 继续正常启动流程                                */
/* ================================================================== */
void ibox_try_handshake(void)
{
    uint8_t  cmd;
    uint32_t len = 1U;
    fsp_err_t err;

    comms_send((uint8_t *)"\r\n[BL v2.1] ABS ready\r\n", 23U);

    err = comms_read(&cmd, &len, 20U);   /* 20ms timeout */

    if (FSP_SUCCESS == err && COMMAND_HANDSHAKE == cmd)
    {
        comms_send(&cmd, 1U);   /* echo 'B' */
        honcho_loop();          /* IBOX 协议 — 永不返回 */
    }
    /* 超时或收到非 'B' 数据 → 静默返回, 继续正常启动 */
}

/* ================================================================== */
/*  UBOX 文本菜单 (人工操作路径)                                          */
/*                                                                       */
/*  注意: IBOX 握手已在 bl2_main() 最早阶段通过 ibox_try_handshake()    */
/*  完成, 此处不再重复检测. UBOX 菜单中按 'B' 可手动进入 IBOX 协议.     */
/* ================================================================== */
void menu(void)
{
    struct boot_rsp rsp;
    volatile fsp_err_t err = 0;
    uint8_t  rx_str[20] = {0};
    uint32_t len;
    uint8_t  reselect = 1;
    uint32_t flag_end = 0xFFFF;
    uint32_t CRC_rom, CRC_ccitt;

    FSP_PARAMETER_NOT_USED(err);
    while (1) {
        comms_send((uint8_t *)"\n\r1 - Update Boot + App\n\r", 26U);
        comms_send((uint8_t *)"2 - Update App ONLY (XModem)\n\r", 32U);
        comms_send((uint8_t *)"3 - Reboot\n\r", 14U);
        comms_send((uint8_t *)"B - IBOX Protocol\n\r", 19U);
        comms_send((uint8_t *)">", 1U);

        rx_str[0] = 0;
        len = 1;
        err = comms_read(rx_str, &len, MENU_TIMEOUT_MS);
        if(err != FSP_SUCCESS)
            break;

        switch (rx_str[0]){
            case '1':
                comms_send((uint8_t *)"\n\rSelect file to write into flash area\n\r", 42U);
                break;

            case '2':
                if (firmware_update_via_xmodem())
                {
                    reselect = 0;
                }
                break;

            case 'B':
            case 'b':
                /* 从 UBOX 菜单进入 IBOX: 切回 9600, 回显 'B' 完成握手, 进入协议循环. */
                comms_set_baud(UART_BAUD_SLOW);
                comms_send(rx_str, 1U);  /* echo 'B' back */
                honcho_loop();
                /* honcho_loop 超时返回 → 切回 115200 继续 UBOX 菜单 */
                comms_set_baud(UART_BAUD_FAST);
                rx_str[0] = 0;
                break;

            default:
                comms_send((uint8_t *)"\n\rSelect again.\n\r", 19U);
                reselect = 1;
                break;
        }
        if(reselect == 0)
            return;
        comms_send(rx_str, 1U);
        comms_send((uint8_t *)"\n\r", 2U);
    }

    flag_end = *(uint32_t *)OTA_FLAG_END_ADDRESS;

    if( 0x55555555 == flag_end ){
        do_boot(&rsp);
    }
    else if( 0xFFFFFFFF == flag_end ){
        CRC_rom = calcrc(APP_IMAGE_START_ADDRESS, (CRC_ADDRESS - APP_IMAGE_START_ADDRESS));
        CRC_ccitt = (unsigned short)(*(unsigned int *)CRC_ADDRESS);

        if( CRC_rom == CRC_ccitt ){
            do_boot(&rsp);
        }
        else{
            comms_send((uint8_t *)"CRC NG\r\n", 8U);
        }
    }
    else{
        comms_send((uint8_t *)"OTA flag invalid\r\n", 18U);
    }
}

/* ================================================================== */
/*  Command handlers — Honcho PR3 IBOX protocol                         */
/* ================================================================== */

/*
 * 'B' HANDSHAKE
 * Response: 'B' (echo back)
 * After handshake, EEPROM is read for subsequent READ_EEPROM commands
 */
static void cmd_handshake(void)
{
    uint8_t rsp = COMMAND_HANDSHAKE;
    comms_send(&rsp, 1U);

    /* Stop timer — subsequent operations never timeout (same as abs_bootloader) */
    g_timer_enabled = false;
    g_timer0.p_api->stop(g_timer0.p_ctrl);
}

/*
 * 'I' IDENTIFY — Honcho PR3 format
 * Response: 'I' + 0x04 + 0x45 + ETX (4 bytes)
 */
static void cmd_identify(void)
{
    uint8_t rsp[4];
    rsp[0] = COMMAND_IDENTIFY;
    rsp[1] = 0x04U;
    rsp[2] = 0x45U;
    rsp[3] = ETX;
    comms_send(rsp, sizeof(rsp));
}

/*
 * 'D' SET_ADDRESS — Honcho PR3 format
 *
 * Honcho DL_Send_Set_Addr_Msg():
 *   1) uart_tx_bit_bang('D')         — 先回显 'D'
 *   2) addr_high = uart_rx_bit_bang() — 再收高字节
 *   3) addr_low  = uart_rx_bit_bang() — 收低字节
 *   4) uart_tx_bit_bang(ETX)         — 最后发 ETX
 */
static void cmd_set_address(void)
{
    uReg32 addr;
    /* 1) 先回显 'D' */
    {
        uint8_t echo = COMMAND_SET_ADDRESS;
        comms_send(&echo, 1U);
    }

    /* 对齐 CMS32M67: 高字节/低字节清零 */



    /* 2) 接收 2 字节 (存入 Val[2]/Val[1], 对齐 CMS32M67) */
    if (recv_byte(&addr.Val[2]) != FSP_SUCCESS) { send_nak(); return; }
    if (recv_byte(&addr.Val[1]) != FSP_SUCCESS) { send_nak(); return; }


    /* 4) 发 ETX */
    uint8_t etx = ETX;
    comms_send(&etx, 1U);

    addr.Val[3] = 0U;
    addr.Val[0] = 0U;
    /* 3) 非特殊地址 (0xFFFF/0xFFFE) → 左移 1 位 */
    if (!(((addr.Val[2] == 0xFFU) && (addr.Val[1] == 0xFFU)) ||
          ((addr.Val[2] == 0xFFU) && (addr.Val[1] == 0xFEU))))
    {
        addr.Val32 <<= 1U;
    }

    g_addr_high = addr.Val[2];
    g_addr_low  = addr.Val[1];
    g_target_offset = ((uint32_t)addr.Val[2] << 8U) | (uint32_t)addr.Val[1];
    g_target_addr   = addr.Val32;
}

/*
 * 'N' READ_FLASH — Honcho PR3 format
 *
 * Honcho IBOX(): 不读取任何字节, 根据已存地址响应:
 *   addr=0xFFFF → DL_Send_Read_FLASH_Msg()       // 版本信息 (88 bytes)
 *   addr=0xFFFE → DL_Calibration_Msg()           // 校准 (5 bytes)
 *   other       → uart_tx_bit_bang('N')          // 仅回显 'N'
 */
static void cmd_read_flash(void)
{
    if (g_target_offset == VERSION_REQUEST_OFFSET)
    {

        /* addr = 0xFFFF → 版本信息 */
        cmd_read_version_info();
        g_target_offset = 0U;
        g_target_addr =  0x00000000;
    }
    else if (g_target_offset == CALIBRATION_REQUEST_OFFSET)
    {
        /* addr = 0xFFFE → 校准请求: 写 flag 到 Data Flash, APP 下次启动时执行校准 */

        /* 写校准请求 flag 到 Data Flash offset 12 */
        {
            uint32_t flag_val = CALIB_REQ_FLAG_MAGIC;
            fsp_err_t ferr;
            ThreadsAndInterrupts(DISABLE);
            ferr = g_flash0.p_api->write(g_flash0.p_ctrl,
                                         (uint32_t)&flag_val,
                                         CALIB_REQ_FLAG_ADDR, 4U);
            ThreadsAndInterrupts(RE_ENABLE);

            if (FSP_SUCCESS == ferr)
            {
                LED2_ON;   /* 写入成功: LED2 亮 */
            }
        }

        /* 发送响应: 'N' + 0x00 + 0x01 + 0x00 (success) / 0x01 (fail) + ETX */
        {
            uint8_t cal_rsp[5];
            cal_rsp[0] = COMMAND_READ_FLASH;
            cal_rsp[1] = 0x00U;
            cal_rsp[2] = 0x01U;
            cal_rsp[3] = 0x01U;
            cal_rsp[4] = ETX;
            comms_send(cal_rsp, sizeof(cal_rsp));
        }

        g_target_offset = 0U;
        g_target_addr =  0x00000000;
    }
    else
    {
        /* 其他地址 → 读取 Flash 内容 (从 g_target_addr 开始, 512 字节) */
        static uint8_t rsp[PAGE_SIZE + 4];
        uint16_t i;



        rsp[0] = COMMAND_READ_FLASH;
        rsp[1] = (uint8_t)(PAGE_SIZE >> 8U);
        rsp[2] = (uint8_t)(PAGE_SIZE & 0xFFU);

        ThreadsAndInterrupts(DISABLE);
        for (i = 0U; i < PAGE_SIZE; i++)
        {
            rsp[3U + i] = *(uint8_t *)(g_target_addr + i);
        }
        ThreadsAndInterrupts(RE_ENABLE);
        rsp[3U + PAGE_SIZE] = ETX;

        comms_send(rsp, sizeof(rsp));

        g_target_offset = 0U;
        g_target_addr =  0x00000000;
    }
}

/*
 * 版本信息响应 — Honcho DL_Send_Read_FLASH_Msg() 格式
 *
 * 完整响应 (90 bytes):
 *   'N' + 0x00 + 0x56 (86 = payload size)
 *   + g_module_id[8]             (offset  3-10)
 *   + g_module_id_second[8]      (offset 11-18)
 *   + ' '                       (offset 19)
 *   + g_tool_part_number[8]     (offset 20-27)
 *   + g_personalization[40]     (offset 28-67)
 *   + SW_VERSION_MAJOR          (offset 68)
 *   + SW_VERSION_MINOR          (offset 69)
 *   + SW_VERSION_MAJOR (重复)   (offset 70)
 *   + SW_VERSION_MINOR (重复)   (offset 71)
 *   + UCID[0..3]                (offset 72-75)  ← DEVICEID[0]
 *   + UCID[4..7]                (offset 76-79)  ← DEVICEID[1]
 *   + UCID[8..11]               (offset 80-83)  ← DEVICEADDR[0]
 *   + UCID[12..15]              (offset 84-87)  ← DEVICEADDR[1]
 *   + MODULE_REVISION           (offset 88)
 *   + ETX                       (offset 89)
 */
static void cmd_read_version_info(void)
{
    uint8_t  rsp[90];
    uint16_t i;
    uint16_t offset = 0U;

    /* Header: 'N' + len (86 bytes payload) */
    rsp[offset++] = COMMAND_READ_FLASH;
    rsp[offset++] = 0x00U;
    rsp[offset++] = VERSION_PAYLOAD_SIZE;   /* 86 */

    /* Module ID — 8 bytes */
    for (i = 0U; i < BYTES_MODULE_ID; i++)
    {
        rsp[offset++] = g_module_id[i];
    }

    /* Code Module ID — 8 bytes */
    for (i = 0U; i < BYTES_MODULE_ID; i++)
    {
        rsp[offset++] = g_module_id_second[i];
    }

    /* Space separator */
    rsp[offset++] = ' ';

    /* Part Number — 8 bytes */
    for (i = 0U; i < BYTES_PART_NUMBER; i++)
    {
        rsp[offset++] = (uint8_t)g_tool_part_number[i];
    }

    /* Personalization — 40 bytes (covers offset 26-65) */
    for (i = 0U; i < BYTES_PERSONALIZATION; i++)
    {
        rsp[offset++] = (uint8_t)g_personalization[i];
    }

    /* SW Version */
    rsp[offset++] = SW_VERSION_MAJOR;
    rsp[offset++] = SW_VERSION_MINOR;
    rsp[offset++] = SW_VERSION_MAJOR;   /* 重复 */
    rsp[offset++] = SW_VERSION_MINOR;   /* 重复 */

    /* Serialization — RA2E1 UCID 16 bytes (Honcho PR3 格式:
     *   DEVICEID[0:4] + DEVICEID[1:4] + DEVICEADDR[0:4] + DEVICEADDR[1:4]) */
    for (i = 0U; i < 4U; i++)  { rsp[offset++] = *(uint8_t *)(UCID_ADDRESS + i); }        /* UCID[0..3]  */
    for (i = 0U; i < 4U; i++)  { rsp[offset++] = *(uint8_t *)(UCID_ADDRESS + 4U + i); }    /* UCID[4..7]  */
    for (i = 0U; i < 4U; i++)  { rsp[offset++] = *(uint8_t *)(UCID_ADDRESS + 8U + i); }    /* UCID[8..11] */
    for (i = 0U; i < 4U; i++)  { rsp[offset++] = *(uint8_t *)(UCID_ADDRESS + 12U + i); }   /* UCID[12..15]*/

    /* MODULE_REVISION */
    rsp[offset++] = MODULE_REVISION;

    /* ETX */
    rsp[offset++] = ETX;

    /* 一次发送完整 90 字节 */
    comms_send(rsp, 90U);
}

/*
 * 校准响应 — Honcho DL_Calibration_Msg() 格式
 *
 * UBOX 收到校准命令后:
 *   1. 写 CALIB_REQ_FLAG_MAGIC 到 Data Flash offset 12
 *   2. LED2 亮起表示写入成功
 *   3. 发送响应 'N' + 0x00 + 0x01 + 0x00 + ETX
 *
 * APP 下次启动时检测到 flag 后自动执行校准并清除 flag.
 */
/*
 * 'F' READ_EEPROM — Honcho PR3 format
 *
 * Honcho IBOX(): 不读取任何字节, 根据已存地址响应:
 *   addr=0xFFFF → DL_Send_Read_EEPROM_Msg()   // 发送 log 数据
 *   other       → uart_tx_bit_bang(ETX)       // 仅发 ETX
 */
static void cmd_read_eeprom(void)
{
    if (g_target_offset == VERSION_REQUEST_OFFSET)
    {
        /* addr = 0xFFFF → 发送 APP Data Flash datalog (进入 IBOX 时已缓存到 RAM) */
        uint8_t rsp[DATALOG_SIZE + 4];
        uint16_t i;

        rsp[0] = COMMAND_READ_EEPROM;
        rsp[1] = (uint8_t)(DATALOG_SIZE >> 8U);
        rsp[2] = (uint8_t)(DATALOG_SIZE & 0xFFU);

        for (i = 0U; i < DATALOG_SIZE; i++)
        {
            rsp[3U + i] = g_datalog_buf[i];
        }
        rsp[3U + DATALOG_SIZE] = ETX;

        comms_send(rsp, sizeof(rsp));
    }
    else
    {
        /* 其他地址 → 仅发 ETX (与 Honcho 完全一致) */
        uint8_t etx = ETX;
        comms_send(&etx, 1U);
    }
}

/*
 * 'M' LOAD_FLASH
 *
 * Bootloader 特有命令 (Honcho 未实现).
 * Receive: length high + length low + data + checksum high + checksum low + ETX
 * Response: 'M' + 0x00 + 0x02 + checksum high + checksum low
 *
 * Auto-erase 2KB block when entering new block
 */
static void cmd_load_flash(void)
{
    uint8_t  nh, nl;
    uint8_t  buf[PAGE_SIZE];
    uint16_t n, i;
    fsp_err_t err;

    uint8_t etx = 0X4D;
    comms_send(&etx, 1U);

    if (recv_byte(&nh) != FSP_SUCCESS) { send_nak(); return; }
    if (recv_byte(&nl) != FSP_SUCCESS) { send_nak(); return; }
    n = (uint16_t)(((uint16_t)nh << 8U) | (uint16_t)nl);

    if (n == 0U || n > PAGE_SIZE) { send_nak(); return; }

    /* Receive data */
    for (i = 0U; i < n; i++)
    {
        if (recv_byte(&buf[i]) != FSP_SUCCESS) { send_nak(); return; }
    }

    /* Validate address range */
    if (!addr_in_app_range(g_target_addr, n))
    {
        //send_nak();
        uint16_t checksum_error = checksum16(buf, n);
        uint8_t rsp[3];
        //rsp[0] = COMMAND_LOAD_FLASH;
/*        rsp[1] = 0x00U;
        rsp[2] = 0x02U;*/
        rsp[0] = (uint8_t)(checksum_error >> 8U);
        rsp[1] = (uint8_t)(checksum_error & 0xFFU);
        rsp[2] = ETX;
        comms_send(rsp, 3U);
        return;
    }

    /* Erase 2KB block if entering new block */
    err = erase_block_if_needed(g_target_addr);
    if (err != FSP_SUCCESS) { send_nak(); return; }

    /* Write page to flash */
    ThreadsAndInterrupts(DISABLE);
    err = g_flash0.p_api->write(g_flash0.p_ctrl, (uint32_t)buf, g_target_addr, n);
    ThreadsAndInterrupts(RE_ENABLE);

    if (err != FSP_SUCCESS) { send_nak(); return; }

    g_target_addr += n;

    /* Send response: M + 00 + 02 + checksum + ETX */
    {
        uint16_t checksum = checksum16(buf, n);
        uint8_t rsp[3];
        //rsp[0] = COMMAND_LOAD_FLASH;
/*        rsp[1] = 0x00U;
        rsp[2] = 0x02U;*/
        rsp[0] = (uint8_t)(checksum >> 8U);
        rsp[1] = (uint8_t)(checksum & 0xFFU);
        rsp[2] = ETX;
        comms_send(rsp, 3U);
    }
}


/*
 * '*' CHANGE_BAUD — Bootloader 格式
 *
 * 回显 '*' 后切换到高速波特率, 保持高速等待后续 SET_ADDRESS + LOAD_FLASH.
 * '*' 是唯一波特率切换点, N/M 命令不再自行切回慢速.
 */
static void cmd_change_baud(void)
{
    uint8_t rsp = COMMAND_CHANGE_BAUD;
    comms_send(&rsp, 1U);

    /* Switch to fast baud rate (28800) */
    fsp_err_t err = comms_set_baud(UART_BAUD_FAST);
    if (err != FSP_SUCCESS)
    {
        send_nak();
    }
}


/* ================================================================== */
/*  Boot attempt                                                        */
/* ================================================================== */
static void try_boot(void)
{
    struct boot_rsp rsp;
    /* Jump to APP unconditionally (same as abs_bootloader) */
    do_boot(&rsp);
}

/* ================================================================== */
/*  Helpers                                                             */
/* ================================================================== */

static void send_nak(void)
{
    uint8_t b = NAK;
    comms_send(&b, 1U);
}

static fsp_err_t recv_byte(uint8_t *out)
{
    uint32_t len = 1U;
    if (g_timer_enabled)
    {
        return comms_read(out, &len, RX_TIMEOUT_MS);
    }
    else
    {
        return comms_read_blocking(out, &len);
    }
}

static uint16_t checksum16(const uint8_t *buf, uint16_t len)
{
    uint16_t sum = 0U;
    for (uint16_t i = 0U; i < len; i++)
    {
        sum += buf[i];
    }
    return sum;
}

static bool addr_in_app_range(uint32_t addr, uint32_t len)
{
    if (addr < APP_IMAGE_START_ADDRESS)              return false;
    if ((addr + len) > (0x0001FF00)) return false;
    return true;
}

static fsp_err_t erase_block_if_needed(uint32_t addr)
{
    if ((addr % FLASH_BLOCK_SIZE) != 0U)
    {
        return FSP_SUCCESS;
    }

    fsp_err_t err;
    ThreadsAndInterrupts(DISABLE);
    err = g_flash0.p_api->erase(g_flash0.p_ctrl, addr, 1U);
    ThreadsAndInterrupts(RE_ENABLE);
    return err;
}

/* ================================================================== */
/*  ThreadsAndInterrupts                                                */
/* ================================================================== */
void ThreadsAndInterrupts(enable_disable_t EnableDisable)
{
    static uint32_t control_reg_value;
    static uint32_t old_primask;

    if (DISABLE == EnableDisable)
    {
        old_primask       = __get_PRIMASK();
        control_reg_value = SysTick->CTRL;
        SysTick->CTRL     = 0;
        NVIC_DisableIRQ(SysTick_IRQn);
        NVIC_ClearPendingIRQ(SysTick_IRQn);
        __disable_irq();
    }
    else
    {
        NVIC_EnableIRQ(SysTick_IRQn);
        SysTick->CTRL = control_reg_value;
        __set_PRIMASK(old_primask);
    }
}
