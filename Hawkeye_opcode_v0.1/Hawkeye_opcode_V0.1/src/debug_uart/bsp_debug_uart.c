/*
 * bsp_debug_uart.c
 *
 *  Created on: Sep 18, 2024
 *      Author: AXQ0527A
 */

#include "hal_data.h"
#include "stdio.h"
#include <stdbool.h>
#include <iic/iic.h>
#include "debug_uart/bsp_debug_uart.h"
#include "debug_uart/power_calibration.h"
#include "stats/bsp_stats.h"
#include "key/bsp_irq_key.h"        /* gLaserOn[3] */

/* OTA update trigger — must match bootloader's header.h */
#define OTA_FLAG_ADDR            0x0001FFFC
#define ENTER_UPDATE_MODE        0x5555AAAA

char str_buf[RX_BUFFER_SIZE + 1] = {0};
volatile int reference_voltage = 0;
volatile int reference_voltage1 = 0;
volatile int reference_voltage2 = 0;
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
#define MAX_REF 3
volatile bool g_print_enabled = false;
extern uint32_t duty11;
extern volatile int32_t g_current_mA[3];  /* [0]=H [1]=V1 [2]=V2, 单位 mA */

static volatile bool uart_send_complete_flag = true;
static volatile bool new_data_ready = false;
static volatile uint32_t data_length = 0;
#define UART_TX_WAIT_TIMEOUT_LOOP   (5000UL)

typedef struct
{
    bool locked;
    bool front_ok;
    bool side_ok;
    bool horiz_ok;
} cfg_state_t;

static cfg_state_t cfg =
{
    .locked   = false,
    .front_ok = false,
    .side_ok= false,
    .horiz_ok = false
};
typedef enum
{
    CMD_NONE = 0,
    CMD_UNLOCK,
    CMD_LOCKSTATE,
    CMD_SET_V2,
    CMD_SET_H,
    CMD_SET_V1,
    CMD_GET_V2,
    CMD_GET_H,
    CMD_GET_V1,
    CMD_GET_ALL,
    CMD_SAVE,
    CMD_CALIBRATION,
    CMD_GET_CALIB,
    CMD_ANGLE_0,
    CMD_ANGLE_4,
    CMD_ANGLE_10,
    CMD_ANGLE_90,
    CMD_RESET,
    CMD_SET_PRINT,
    CMD_SET_DUTY2,
    CMD_GET_POWERLOG,       /* 查询关机日志 */
    CMD_CLEAR_POWERLOG,     /* 清除关机日志 */
    CMD_GET_STATS,       /* 查询运行统计 */
    CMD_CLEAR_STATS,     /* 清除运行统计 */
    CMD_GET_ALL_INFO,
    CMD_OVERHEAT,
    CMD_CLEAR_ALL_FLASH,
    /* ===== 光功率校准 ===== */
    CMD_POWCAL,             /* AT+POWCAL=<CH>,<power> 或 AT+POWCAL=STOP/SAVE/STATUS */
    CMD_POWTARGET,          /* AT+POWTARGET=<H|V1|V2>,<power_x100> 或 AT+POWTARGET? */
    CMD_LASER_CTRL,         /* AT+LASER=<CH>,<ON|OFF> */
    CMD_IMU_DEBUG,          /* AT+IMUDBG=<0|1> IMU角度实时打印开关 */
    CMD_FW_UPDATE,          /* 5 — 进入固件更新模式 */
} uart_cmd_t;

/* ======================================================================
 *  关机原因名称（索引与 power_off_reason_t 枚举值对应）
 * ====================================================================== */
static const char * const poweroff_reason_names[POWEROFF_REASON_COUNT] = {
    "BATTERY_LOW",
    "LOCK_SW_RUNTIME",
    "I2C_FAIL",
    "OVERHEAT",
    "IDLE_TIMEOUT",
};
#define CFG_PASSWORD   "123456"

// 识别命令类型
static uart_cmd_t get_uart_cmd(char * str)
{
    if (strncmp(str, "AT+UNLOCKCFG=", 13) == 0) return CMD_UNLOCK;
    if (strcmp(str, "AT+LOCKSTATE?") == 0)        return CMD_LOCKSTATE;

    if (strncmp(str, "AT+SET_V2=", 10) == 0)   return CMD_SET_V2;
    if (strncmp(str, "AT+SET_H=",   9) == 0)   return CMD_SET_H;
    if (strncmp(str, "AT+SET_V1=", 10) == 0)   return CMD_SET_V1;

    if (strcmp(str, "AT+GET_V2?") == 0)        return CMD_GET_V2;
    if (strcmp(str, "AT+GET_H?") == 0)         return CMD_GET_H;
    if (strcmp(str, "AT+GET_V1?") == 0)        return CMD_GET_V1;

    if (strcmp(str, "AT+GET_ALL") == 0)   return CMD_GET_ALL;
    if (strcmp(str, "AT+SAVE") == 0)    return CMD_SAVE;
    if (strcmp(str, "AT+RESET") == 0)  return CMD_RESET;
    if (strcmp(str, "AT+CALIBRATION") == 0) return CMD_CALIBRATION;
    if (strcmp(str, "AT+GETCALIBRATION") == 0) return CMD_GET_CALIB;
    if (strcmp(str, "0")  == 0) return CMD_ANGLE_0;
    if (strcmp(str, "4")  == 0) return CMD_ANGLE_4;
    if (strcmp(str, "10") == 0) return CMD_ANGLE_10;
    if (strcmp(str, "90") == 0) return CMD_ANGLE_90;
    if (strncmp(str, "AT+SET_DUTY2=", 13) == 0) return CMD_SET_DUTY2;
    if (strncmp(str, "AT+PRINT=", 9) == 0) return CMD_SET_PRINT;
    if (strcmp (str, "AT+GET_POWERLOG")   == 0)  return CMD_GET_POWERLOG;
    if (strcmp (str, "AT+CLEAR_POWERLOG") == 0)  return CMD_CLEAR_POWERLOG;
    if (strcmp(str, "AT+GET_STATS")   == 0)  return CMD_GET_STATS;
    if (strcmp(str, "AT+CLEAR_STATS") == 0)  return CMD_CLEAR_STATS;
    if (strcmp(str, "AT+GET_ALL_INFO") == 0)  return CMD_GET_ALL_INFO;
    if (strcmp(str, "AT+OVERHEAT") == 0)  return CMD_OVERHEAT;
    if (strcmp(str, "AT+CLEAR_ALL_FLASH") == 0) return CMD_CLEAR_ALL_FLASH;
    if (strncmp(str, "AT+POWCAL=", 10) == 0)  return CMD_POWCAL;
    if (strncmp(str, "AT+POWTARGET=", 13) == 0) return CMD_POWTARGET;
    if (strcmp(str, "AT+POWTARGET?") == 0)    return CMD_POWTARGET;
    if (strncmp(str, "AT+LASER=", 9) == 0)    return CMD_LASER_CTRL;
    if (strncmp(str, "AT+IMUDBG=", 10) == 0)  return CMD_IMU_DEBUG;
    if (strcmp(str, "5") == 0)                    return CMD_FW_UPDATE;
    return CMD_NONE;
}

// 串口初始�?
void Debug_UART9_Init(void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
    assert(FSP_SUCCESS == err);
    err = R_SCI_UART_Read(&g_uart9_ctrl, (uint8_t *)rx_buffer, RX_BUFFER_SIZE);
    assert(FSP_SUCCESS == err);
}

// 串口回调函数
void debug_uart9_callback(uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
/*            char c = p_args->data & 0xFF; // 只取�?8 �?

            // 回显收到的字�?
            R_SCI_UART_Write(&g_uart9_ctrl, (uint8_t*)&c, 1);*/
            new_data_ready = true;
            data_length = RX_BUFFER_SIZE;
            break;
        }

        case UART_EVENT_TX_COMPLETE:
        {
            uart_send_complete_flag = true;
            break;
        }

        default:
            break;
    }
}

void uart9_send_blocking(const char *msg)
{
    if ((NULL == msg) || ('\0' == msg[0]))
    {
        return;
    }

    uart_send_complete_flag = false;
    R_SCI_UART_Write(&g_uart9_ctrl, (uint8_t *)msg, strlen(msg));
    // waiting with timeout to avoid hang in brownout range
    uint32_t timeout = UART_TX_WAIT_TIMEOUT_LOOP;
    while ((!uart_send_complete_flag) && (timeout > 0U))
    {
        timeout--;
        __NOP();
    }
    wdt_feed();  // 每条消息发完喂一次狗
}
void Debug_UART9_ProcessReceivedData(void)
{

    // ===== 每次进入时，根据三路电压是否都非0动态决定锁定状态 =====
/*    if (reference_voltage != 0 && reference_voltage1 != 0 && reference_voltage2 != 0)
    {
        cfg.locked = true;
    }*/
    memcpy(str_buf, (const char *)rx_buffer, RX_BUFFER_SIZE);

    /* 去掉尾部回车换行及空格（兼容不同串口工具的差异） */
    for (uint16_t i = 0; i < RX_BUFFER_SIZE; i++)
    {
        if (str_buf[i] == '\r' || str_buf[i] == '\n')
        {
            str_buf[i] = '\0';
            break;
        }
    }
    /* 继续向前扫描，截断所有尾部空白字符 */
    {
        int tail = (int)strlen(str_buf) - 1;
        while (tail >= 0 && (str_buf[tail] == ' ' || str_buf[tail] == '\t'))
        {
            str_buf[tail] = '\0';
            tail--;
        }
    }

    uart_cmd_t cmd = get_uart_cmd(str_buf);

    switch (cmd)
    {
        case CMD_UNLOCK:
        {
            char * pwd = str_buf + 13;     // 跳过 "AT+UNLOCKCFG="

            if (strcmp(pwd, CFG_PASSWORD) == 0)
            {
                cfg.locked   = false;
                cfg.front_ok = false;
                cfg.side_ok  = false;
                cfg.horiz_ok = false;

                uart9_send_blocking("+RESP:UNLOCKED\r\n");
            }
            else
            {
                uart9_send_blocking("+RESP:PASSWORD ERROR, PLEASE RETRY\r\n");
            }
        }
        break;


        case CMD_LOCKSTATE:
        {
            if (cfg.locked)
                uart9_send_blocking("+RESP:LOCKED\r\n");
            else
                uart9_send_blocking("+RESP:UNLOCKED\r\n");
        }
        break;


        // ---------------- 设置类命�?----------------

        case CMD_SET_V2:
        {
            if (cfg.locked)
            {
                uart9_send_blocking("+RESP:PLEASE UNLOCK FIRST\r\n");
                break;
            }

            int val = atoi(str_buf + 10);

            if (val >= 0 && val <= 3300)
            {
                reference_voltage = val;
                cfg.front_ok = true;
                uart9_send_blocking("+RESP:OK\r\n");
            }
            else
            {
                uart9_send_blocking("+RESP:ERR\r\n");
            }
        }
        break;


        case CMD_SET_H:
        {
            if (cfg.locked)
            {
                uart9_send_blocking("+RESP:PLEASE UNLOCK FIRST\r\n");
                break;
            }

            int val = atoi(str_buf + 9);

            if (val >= 0 && val <= 3300)
            {
                reference_voltage1 = val;
                cfg.side_ok = true;
                uart9_send_blocking("+RESP:OK\r\n");
            }
            else
            {
                uart9_send_blocking("+RESP:ERR\r\n");
            }
        }
        break;


        case CMD_SET_V1:
        {
            if (cfg.locked)
            {
                uart9_send_blocking("+RESP:PLEASE UNLOCK FIRST\r\n");
                break;
            }

            int val = atoi(str_buf + 10);

            if (val >= 0 && val <= 3300)
            {
                reference_voltage2 = val;
                cfg.horiz_ok = true;
                uart9_send_blocking("+RESP:OK\r\n");
            }
            else
            {
                uart9_send_blocking("+RESP:ERR\r\n");
            }
        }
        break;


        // ---------------- 查询命令 ----------------

        case CMD_GET_V2:
        {
            char msg[64];
            sprintf(msg, "+RESP:V2=%dmV\r\n", reference_voltage);
            uart9_send_blocking(msg);
        }
        break;


        case CMD_GET_H:
        {
            char msg[64];
            sprintf(msg, "+RESP:H=%dmV\r\n", reference_voltage1);
            uart9_send_blocking(msg);
        }
        break;


        case CMD_GET_V1:
        {
            char msg[64];
            sprintf(msg, "+RESP:V1=%dmV\r\n", reference_voltage2);
            uart9_send_blocking(msg);
        }
        break;

        case CMD_GET_ALL:
        {
            char msg[64];
            sprintf(msg, "RESP:V2=%dmV\r\n", reference_voltage);
            uart9_send_blocking(msg);
            sprintf(msg, "RESP:H=%dmV\r\n", reference_voltage1);
            uart9_send_blocking(msg);
            sprintf(msg, "RESP:V1=%dmV\r\n", reference_voltage2);
            uart9_send_blocking(msg);
        }
        break;

        case CMD_SAVE:
        {
            if (!cfg.locked)
            {
                uart9_send_blocking("+RESP:PARAMETER NOT READY (NOT LOCKED)\r\n");
                break;
            }

            if (flash_save_reference(reference_voltage,
                                     reference_voltage1,
                                     reference_voltage2) == FSP_SUCCESS)
            {
                uart9_send_blocking("+RESP:SAVE OK\r\n");
            }
            else
            {
                uart9_send_blocking("+RESP:SAVE FAIL\r\n");
            }
        }
        break;

        case CMD_RESET:
        {
            if (!cfg.locked)
            {
                /* 清空 RAM 中的�?*/
                reference_voltage  = 0;
                reference_voltage1 = 0;
                reference_voltage2 = 0;

                cfg.front_ok = false;
                cfg.side_ok  = false;
                cfg.horiz_ok = false;

                /* 保存到Flash，将三路参考电压清�?*/
                if (flash_save_reference(0, 0, 0) == FSP_SUCCESS)
                {
                    uart9_send_blocking("+RESP:RESET SUCCESS, FLASH ERASED\r\n");
                }
                else
                {
                    uart9_send_blocking("+RESP:RESET FAILED\r\n");
                }
            }
            else
            {
                uart9_send_blocking("+RESP:PLEASE UNLOCK FIRST\r\n");
            }
        }
        break;


        case CMD_CALIBRATION:
        {
            g_calib_request = true;
            g_calib_cnt = 0;
            g_calib_sum[0] = 0;
            g_calib_sum[1] = 0;
            g_calib_sum[2] = 0;
            uart9_send_blocking("+RESP:CALIBRATION START\r\n");
        }
        break;

        case CMD_GET_CALIB:
        {
            int16_t ox, oy, oz;
            if (flash_load_calibration(&ox, &oy, &oz) == FSP_SUCCESS)
            {
                char msg[64];
                sprintf(msg, "+RESP:CALIB X=%d Y=%d Z=%d\r\n", ox, oy, oz);
                uart9_send_blocking(msg);
            }
            else
            {
                uart9_send_blocking("+RESP:NO CALIB DATA\r\n");
            }
        }
        break;

        case CMD_ANGLE_0:
        {
            gAngleMode = ANGLE_MODE_0_4;
            uart9_send_blocking("+RESP:ANGLE MODE SET 0~4deg\r\n");
        }
        break;

        case CMD_ANGLE_4:
        {
            gAngleMode = ANGLE_MODE_4_10;
            uart9_send_blocking("+RESP:ANGLE MODE SET 4~10deg\r\n");
        }
        break;

        case CMD_ANGLE_10:
        {
            gAngleMode = ANGLE_MODE_10_90;
            uart9_send_blocking("+RESP:ANGLE MODE SET 10~90deg\r\n");
        }
        break;

        case CMD_ANGLE_90:
        {
            gAngleMode = ANGLE_MODE_UNCAL;
            uart9_send_blocking("+RESP:ANGLE MODE SET UNCAL\r\n");
        }
        break;

        case CMD_SET_DUTY2:
        {
            int val = atoi(str_buf + 13);
            if (val >= 0 && val <= 500)
            {
                duty11 = (uint32_t)val;
                char msg[64];
                sprintf(msg, "+RESP:DUTY2=%lu\r\n", (unsigned long)duty11);
                uart9_send_blocking(msg);
            }
            else
            {
                uart9_send_blocking("+RESP:ERR, RANGE 0~100\r\n");
            }
        }
        break;

        case CMD_SET_PRINT:
        {
            int val = atoi(str_buf + 9);
            if (val == 1)
            {
                g_print_enabled = true;
                uart9_send_blocking("+RESP:PRINT ON\r\n");
            }
            else if (val == 0)
            {
                g_print_enabled = false;
                uart9_send_blocking("+RESP:PRINT OFF\r\n");
            }
            else
            {
                uart9_send_blocking("+RESP:ERR\r\n");
            }
        }
        break;

        case CMD_IMU_DEBUG:
        {
            int val = atoi(str_buf + 10);
            if (val == 1)
            {
                g_imu_debug_enabled = true;
                uart9_send_blocking("+RESP:IMUDBG ON\r\n");
            }
            else if (val == 0)
            {
                g_imu_debug_enabled = false;
                uart9_send_blocking("+RESP:IMUDBG OFF\r\n");
            }
            else
            {
                uart9_send_blocking("+RESP:ERR\r\n");
            }
        }
        break;

        case CMD_GET_POWERLOG:
               {
                   flash_poweroff_log_t log;
                   fsp_err_t err = flash_load_poweroff_log(&log);

                   if (err == FSP_ERR_NOT_INITIALIZED)
                   {
                       uart9_send_blocking("+RESP:POWERLOG NO DATA\r\n");
                       break;
                   }

                   char msg[128];
                   snprintf(msg, sizeof(msg), "+RESP:POWERLOG TOTAL=%lu\r\n", (unsigned long)log.total);
                   uart9_send_blocking(msg);

                   for (uint8_t i = 0; i < POWEROFF_REASON_COUNT; i++)
                   {
                       snprintf(msg, sizeof(msg), "+RESP:POWERLOG %s=%lu\r\n",
                                poweroff_reason_names[i],
                                (unsigned long)log.count[i]);
                       uart9_send_blocking(msg);
                   }
                   break;
               }

               /* ================================================================
                *  AT+CLEAR_POWERLOG — 清除关机日志（需先解锁）
                * ================================================================ */
       case CMD_CLEAR_POWERLOG:
       {
           if (cfg.locked) { uart9_send_blocking("+RESP:PLEASE UNLOCK FIRST\r\n"); break; }
           if (flash_clear_poweroff_log() == FSP_SUCCESS)
               uart9_send_blocking("+RESP:POWERLOG CLEARED\r\n");
           else
               uart9_send_blocking("+RESP:POWERLOG CLEAR FAIL\r\n");
           break;
       }
       case CMD_GET_STATS:
       {
           stats_print_all();
           break;
       }

       case CMD_CLEAR_STATS:
       {
           if (cfg.locked)
           {
               uart9_send_blocking("+RESP:PLEASE UNLOCK FIRST\r\n");
               break;
           }
           stats_clear();
           uart9_send_blocking("+RESP:STATS CLEARED\r\n");
           break;
       }
       case CMD_OVERHEAT:
       {
           uart9_send_blocking("+RESP:OVERHEAT TEST\r\n");
           system_overheat_request(OVERHEAT_SOURCE_DEBUG);
           break;
       }

       case CMD_CLEAR_ALL_FLASH:
       {
           if (cfg.locked)
           {
               uart9_send_blocking("+RESP:PLEASE UNLOCK FIRST\r\n");
               break;
           }
           wdt_feed();
           if (flash_clear_all() == FSP_SUCCESS)
           {
               /* Flash 擦除后同步清零 RAM 参考电压, 避免关机延时期间
                * 仍按旧参考电压做 PID 调光 */
               reference_voltage  = 0;
               reference_voltage1 = 0;
               reference_voltage2 = 0;
               cfg.front_ok = false;
               cfg.side_ok  = false;
               cfg.horiz_ok = false;
               uart9_send_blocking("+RESP:ALL FLASH CLEARED\r\n");
           }
           else
               uart9_send_blocking("+RESP:CLEAR FAIL\r\n");
           break;
       }
       case CMD_GET_ALL_INFO:
       {
           char msg[128];
           char hms[24];

           uart9_send_blocking("========== ALL FLASH INFO ==========\r\n");
           wdt_feed();
           /* ==================== 1. 设备/版本信息 ==================== */
           uart9_send_blocking("----- Device Info -----\r\n");
           {
               flash_devinfo_t info;
               fsp_err_t err = flash_load_devinfo(&info);
               if (err == FSP_ERR_NOT_INITIALIZED)
               {
                   uart9_send_blocking("  DevInfo : NO DATA\r\n");
               }
               else
               {
                   snprintf(msg, sizeof(msg), "  Project : %s\r\n",
                            info.project_name);
                   uart9_send_blocking(msg);

                   snprintf(msg, sizeof(msg), "  HW Ver  : %u.%u.%u.%u\r\n",
                            info.hw_major, info.hw_minor,
                            info.hw_patch, info.hw_build);
                   uart9_send_blocking(msg);

                   snprintf(msg, sizeof(msg), "  FW Ver  : %u.%u.%u.%u\r\n",
                            info.fw_major, info.fw_minor,
                            info.fw_patch, info.fw_build);
                   uart9_send_blocking(msg);

                   snprintf(msg, sizeof(msg), "  HW Rev    : %s\r\n",
                            info.hardware_revision);
                   uart9_send_blocking(msg);

                   snprintf(msg, sizeof(msg), "  FW Rev    : %s\r\n",
                            info.firmware_revision);
                   uart9_send_blocking(msg);
               }
           }
           wdt_feed();
           /* ==================== 2. 完整 datalog dump（按 Stepper xlsx 顺序） ==================== */
           if (flash_read_u32(FLASH_OFFSET_HDR_MAGIC) != FLASH_DATALOG_MAGIC)
           {
               uart9_send_blocking("----- Datalog -----\r\n");
               uart9_send_blocking("  Datalog : NO DATA\r\n");
           }
           else
           {
               /* 打印辅助宏：秒→时分秒 + 各类型字段 dump */
               #define FMT_HMS(sec, buf) do { \
                   uint32_t _h = (sec)/3600U; \
                   uint32_t _m = ((sec)%3600U)/60U; \
                   uint32_t _s = (sec)%60U; \
                   snprintf((buf), sizeof(buf), "%luh%02lum%02lus", \
                            (unsigned long)_h, (unsigned long)_m, (unsigned long)_s); \
               } while(0)

               #define DUMP_SECS(label, off) do { \
                   uint32_t _v = flash_read_u32(off); \
                   FMT_HMS(_v, hms); \
                   snprintf(msg, sizeof(msg), "  %-48s : %s\r\n", label, hms); \
                   uart9_send_blocking(msg); \
               } while(0)

               #define DUMP_U32(label, off) do { \
                   snprintf(msg, sizeof(msg), "  %-48s : %lu\r\n", label, (unsigned long)flash_read_u32(off)); \
                   uart9_send_blocking(msg); \
               } while(0)

               #define DUMP_U16(label, off) do { \
                   snprintf(msg, sizeof(msg), "  %-48s : %lu\r\n", label, (unsigned long)flash_read_u16(off)); \
                   uart9_send_blocking(msg); \
               } while(0)

               #define DUMP_I16(label, off) do { \
                   snprintf(msg, sizeof(msg), "  %-48s : %d\r\n", label, (int)(int16_t)flash_read_u16(off)); \
                   uart9_send_blocking(msg); \
               } while(0)

               #define DUMP_I32(label, off) do { \
                   snprintf(msg, sizeof(msg), "  %-48s : %ld\r\n", label, (long)(int32_t)flash_read_u32(off)); \
                   uart9_send_blocking(msg); \
               } while(0)

               /* ---- Protection[14] (@24..51, uint16) ---- */
               uart9_send_blocking("----- [Protection] -----\r\n");
               DUMP_U16("[Protection] NTC - Hot Laser", FLASH_PROT_NTC_HOT_LASER);
               DUMP_U16("[Protection] NTC - Cold Laser", FLASH_PROT_NTC_COLD_LASER);
               DUMP_U16("[Protection] NTC - Frozen Pack", FLASH_PROT_NTC_FROZEN_PACK);
               DUMP_U16("[Protection] NTC - Hot Pack", FLASH_PROT_NTC_HOT_PACK);
               DUMP_U16("[Protection] Battery Under Voltage", FLASH_PROT_BATTERY_UV);
               DUMP_U16("[Protection] Main Power Switch Off", FLASH_PROT_MAIN_SW_OFF);
               DUMP_U16("[Protection] MEMS error", FLASH_PROT_MEMS_ERROR);
               DUMP_U16("[Protection] IDLE Timeout", FLASH_PROT_IDLE_TIMEOUT);
               DUMP_U16("[Protection] H1 Over Current", FLASH_PROT_H1_OVERCUR);
               DUMP_U16("[Protection] H2 Over Current[x]", FLASH_PROT_H2_OVERCUR);
               DUMP_U16("[Protection] V1 Over Current", FLASH_PROT_V1_OVERCUR);
               DUMP_U16("[Protection] V2 Over Current", FLASH_PROT_V2_OVERCUR);
               DUMP_U16("[Protection] 8V Buck Convertor - Upper Limit[x]", FLASH_PROT_BUCK_UPPER);
               DUMP_U16("[Protection] 8V Buck Convertor - Lower Limit[x]", FLASH_PROT_BUCK_LOWER);
               wdt_feed();

               /* ---- Time[38] + total_run_s (@4, @52..203, uint32 秒) ---- */
               uart9_send_blocking("----- [Time] -----\r\n");
               DUMP_SECS("[Time] Total runtime of the unit", FLASH_OFFSET_HDR_TOTAL_RUN);
               DUMP_SECS("[Time] Laser H1 runtime", FLASH_TIME_H1);
               DUMP_SECS("[Time] Laser H2 runtime[x]", FLASH_TIME_H2);
               DUMP_SECS("[Time] Laser V1 runtime", FLASH_TIME_V1);
               DUMP_SECS("[Time] Laser V2 runtime", FLASH_TIME_V2);
               DUMP_SECS("[Time] Laser H1&H2 runtime[x]", FLASH_TIME_H1H2);
               DUMP_SECS("[Time] Laser H1&V1 runtime", FLASH_TIME_H1V1);
               DUMP_SECS("[Time] Laser H1&V2 runtime", FLASH_TIME_H1V2);
               DUMP_SECS("[Time] Laser H2&V1 runtime[x]", FLASH_TIME_H2V1);
               DUMP_SECS("[Time] Laser H2&V2 runtime[x]", FLASH_TIME_H2V2);
               DUMP_SECS("[Time] Laser V1&V2 runtime", FLASH_TIME_V1V2);
               DUMP_SECS("[Time] Laser H1&H2&V1 runtime[x]", FLASH_TIME_H1H2V1);
               DUMP_SECS("[Time] Laser H1&H2&V2 runtime[x]", FLASH_TIME_H1H2V2);
               DUMP_SECS("[Time] Laser H1&V1&V2 runtime", FLASH_TIME_H1V1V2);
               DUMP_SECS("[Time] Laser H2&V1&V2 runtime[x]", FLASH_TIME_H2V1V2);
               DUMP_SECS("[Time] Laser H1&H2&V1&V2 runtime[x]", FLASH_TIME_H1H2V1V2);
               DUMP_SECS("[Time] Slope Laser H1 runtime", FLASH_SLOPE_H1);
               DUMP_SECS("[Time] Slope Laser H2 runtime[x]", FLASH_SLOPE_H2);
               DUMP_SECS("[Time] Slope Laser V1 runtime", FLASH_SLOPE_V1);
               DUMP_SECS("[Time] Slope Laser V2 runtime", FLASH_SLOPE_V2);
               DUMP_SECS("[Time] Slope Laser H1&H2 runtime[x]", FLASH_SLOPE_H1H2);
               DUMP_SECS("[Time] Slope Laser H1&V1 runtime", FLASH_SLOPE_H1V1);
               DUMP_SECS("[Time] Slope Laser H1&V2 runtime", FLASH_SLOPE_H1V2);
               DUMP_SECS("[Time] Slope Laser H2&V1 runtime[x]", FLASH_SLOPE_H2V1);
               DUMP_SECS("[Time] Slope Laser H2&V2 runtime[x]", FLASH_SLOPE_H2V2);
               DUMP_SECS("[Time] Slope Laser V1&V2 runtime", FLASH_SLOPE_V1V2);
               DUMP_SECS("[Time] Slope Laser H1&H2&V1 runtime[x]", FLASH_SLOPE_H1H2V1);
               DUMP_SECS("[Time] Slope Laser H1&H2&V2 runtime[x]", FLASH_SLOPE_H1H2V2);
               DUMP_SECS("[Time] Slope Laser H1&V1&V2 runtime", FLASH_SLOPE_H1V1V2);
               DUMP_SECS("[Time] Slope Laser H2&V1&V2 runtime[x]", FLASH_SLOPE_H2V1V2);
               DUMP_SECS("[Time] Slope Laser H1&H2&V1&V2 runtime[x]", FLASH_SLOPE_H1H2V1V2);
               DUMP_SECS("[Time] tilt uncalibrated", FLASH_TIME_TILT_UNCAL);
               DUMP_SECS("[Time] tilt < 4 degrees (leveled)", FLASH_TIME_TILT_0_4);
               DUMP_SECS("[Time] 4 < tilt < 10 degrees", FLASH_TIME_TILT_4_10);
               DUMP_SECS("[Time] tilt > 10 degrees", FLASH_TIME_TILT_10_90);
               DUMP_SECS("[Time] Total runtime of yaw motor[x]", FLASH_TIME_YAW_MOTOR);
               DUMP_SECS("[Time] Total runtime of vertical motor[x]", FLASH_TIME_VERT_MOTOR);
               DUMP_SECS("[Time] Time in low brightness[x]", FLASH_TIME_LOW_BRIGHT);
               DUMP_SECS("[Time] Time in medium brightness[x]", FLASH_TIME_MED_BRIGHT);
               DUMP_SECS("[Time] Time in high brightness[x]", FLASH_TIME_HIGH_BRIGHT);
               wdt_feed();

               /* ---- Cycle[8] (@208..239, uint32) ---- */
               uart9_send_blocking("----- [Cycle] -----\r\n");
               DUMP_U32("[Cycle] Main Power Switch", FLASH_CYCLE_MAIN_SW);
               DUMP_U32("[Cycle] Horizontal1 Laser Switch", FLASH_CYCLE_H1_LASER);
               DUMP_U32("[Cycle] Horizontal2 Laser Switch[x]", FLASH_CYCLE_H2_LASER);
               DUMP_U32("[Cycle] Vertical1 Laser Switch", FLASH_CYCLE_V1_LASER);
               DUMP_U32("[Cycle] Vertical2 Laser Switch", FLASH_CYCLE_V2_LASER);
               DUMP_U32("[Cycle] Lock Switch", FLASH_CYCLE_LOCK_SW);
               DUMP_U32("[Cycle] Global Brightness Switch Cycles[x]", FLASH_CYCLE_BRIGHTNESS);
               DUMP_U32("[Cycle] Global Brightness Remote Switch Cycles[x]", FLASH_CYCLE_BRIGHTNESS_REMOTE);
               wdt_feed();

               /* ---- Optical[4] (@240..255, int32) ---- */
               uart9_send_blocking("----- [Optical power] -----\r\n");
               DUMP_I32("[Optical power] Laser H1 power", FLASH_OPT_H1);
               DUMP_I32("[Optical power] Laser H2 power", FLASH_OPT_H2);
               DUMP_I32("[Optical power] Laser V1 power", FLASH_OPT_V1);
               DUMP_I32("[Optical power] Laser V2 power", FLASH_OPT_V2);
               wdt_feed();

               /* ---- Offset / MEMS (@256..273) ---- */
               uart9_send_blocking("----- [Offset] / [Model] -----\r\n");
               DUMP_I16("[Offset] MEMS X axis Offset", FLASH_OFFSET_MEMS_X);
               DUMP_I16("[Offset] MEMS Y axis Offset", FLASH_OFFSET_MEMS_Y);
               DUMP_I16("[Offset] MEMS Z axis Offset", FLASH_OFFSET_MEMS_Z);
               DUMP_U32("[Offset Reset Count] MEMS Reset", FLASH_OFFSET_MEMS_RESET);
               {
                   uint8_t model[8];
                   if (flash_load_mems_model(model) == FSP_SUCCESS)
                   {
                       char model_str[9];
                       for (uint8_t i = 0; i < 8U; i++) model_str[i] = (char)model[i];
                       model_str[8] = '\0';
                       snprintf(msg, sizeof(msg), "  %-48s : %s\r\n", "[Model] MEMS Model", model_str);
                       uart9_send_blocking(msg);
                   }
                   else
                   {
                       snprintf(msg, sizeof(msg), "  %-48s : NO DATA\r\n", "[Model] MEMS Model");
                       uart9_send_blocking(msg);
                   }
               }
               wdt_feed();

               /* ---- Reference[4] (@274..281, uint16 mV) ---- */
               uart9_send_blocking("----- [Reference Voltage] -----\r\n");
               DUMP_U16("[Reference Voltage] Laser H1", FLASH_REF_H1);
               DUMP_U16("[Reference Voltage] Laser H2[x]", FLASH_REF_H2);
               DUMP_U16("[Reference Voltage] Laser V1", FLASH_REF_V1);
               DUMP_U16("[Reference Voltage] Laser V2", FLASH_REF_V2);
               wdt_feed();

               /* ---- drop / temp (@282..289) ---- */
               uart9_send_blocking("----- [Count] / [temp] -----\r\n");
               DUMP_U16("[Count] MEMS drop record(>500g)[x]", FLASH_OFFSET_DROP);
               DUMP_U16("[temp] Laser Works in high Temp", FLASH_TEMP_HIGH);
               DUMP_U16("[temp] Laser Works in normal Temp", FLASH_TEMP_NORMAL);
               DUMP_U16("[temp] Laser Works in low Temp", FLASH_TEMP_LOW);
               wdt_feed();

               /* ---- Bluetooth / RunTime (@290..313) ---- */
               uart9_send_blocking("----- [Count] / [Run Time] -----\r\n");
               DUMP_U16("[Count] Bluetooth connected[x]", FLASH_BLE_CONNECTED);
               DUMP_U16("[Count] Bluetooth connections dropped[x]", FLASH_BLE_DROPPED);
               DUMP_SECS("[Run Time] Remote operation distance <20m[x]", FLASH_RUNTIME_REMOTE_20);
               DUMP_SECS("[Run Time] Remote operation distance <40m[x]", FLASH_RUNTIME_REMOTE_40);
               DUMP_SECS("[Run Time] Remote operation distance <60m[x]", FLASH_RUNTIME_REMOTE_60);
               DUMP_SECS("[Run Time] Remote operation distance >60m[x]", FLASH_RUNTIME_REMOTE_60P);
               DUMP_SECS("[Run Time] Coin cell runtime[x]", FLASH_RUNTIME_COIN_CELL);
               wdt_feed();

               /* ---- 版本信息 (@314..319, 数字化 HW/FW Rev) ---- */
               uart9_send_blocking("----- [Version] -----\r\n");
               {
                   /* FW Rev = major/minor/patch 三个 uint8 (@314/315/316) */
                   uint32_t fw = flash_read_u32(FLASH_OFFSET_FWREV_BASE);
                   snprintf(msg, sizeof(msg), "  %-48s : %lu\r\n", "[Version] FW Rev Major",
                            (unsigned long)(fw & 0xFFU));
                   uart9_send_blocking(msg);
                   snprintf(msg, sizeof(msg), "  %-48s : %lu\r\n", "[Version] FW Rev Minor",
                            (unsigned long)((fw >> 8U) & 0xFFU));
                   uart9_send_blocking(msg);
                   snprintf(msg, sizeof(msg), "  %-48s : %lu\r\n", "[Version] FW Rev Patch",
                            (unsigned long)((fw >> 16U) & 0xFFU));
                   uart9_send_blocking(msg);

                   /* HW Rev = uint16 ASCII "A1" (小端: 低字节 '1', 高字节 'A') */
                   uint16_t hw = flash_read_u16(FLASH_OFFSET_HWREV);
                   char hw_str[3];
                   hw_str[0] = (char)(hw >> 8U);       /* 高字节 'A' */
                   hw_str[1] = (char)(hw & 0xFFU);     /* 低字节 '1' */
                   hw_str[2] = '\0';
                   snprintf(msg, sizeof(msg), "  %-48s : %s (0x%04X)\r\n", "[Version] HW Rev",
                            hw_str, (unsigned)hw);
                   uart9_send_blocking(msg);
               }
               wdt_feed();

               #undef FMT_HMS
               #undef DUMP_SECS
               #undef DUMP_U32
               #undef DUMP_U16
               #undef DUMP_I16
               #undef DUMP_I32
           }
           uart9_send_blocking("====================================\r\n");
           break;
       }

        /* ==================================================================
         *  AT+POWCAL — 光功率自动校准
         *
         *  格式:
         *    AT+POWCAL=<CH>,<power_x100>  上报光功率读数，MCU 自动调参考电压
         *    AT+POWCAL=STOP[,<CH>]        停止校准（可指定通道，不指定则全部）
         *    AT+POWCAL=SAVE              保存校准结果到 Flash
         *    AT+POWCAL=STATUS            查询校准状态
         *
         *  响应:
         *    +POWCAL:<CH>,ADJ,<ref_mV>,<cur_mA>       已调整，需再次测量
         *    +POWCAL:<CH>,DONE,<ref_mV>,<cur_mA>      到位，目标达成
         *    +POWCAL:ERR,<CH>,SAT,<ref_mV>,<cur_mA>   饱和：电压上限/电流到限
         *    +POWCAL:ERR,<CH>,LASER_OFF,<ref>,<cur>   保护: 激光未打开
         *    +POWCAL:ERR,<CH>,PD_LOW,<ref>,<cur>      保护: PD 初始值过低
         *    +POWCAL:ERR,<CH>,NO_RESP,<ref>,<cur>     保护: 升压无响应
         *    +POWCAL:STOP,<CH>              已停止
         *    +POWCAL:SAVE,OK                保存成功
         *    +POWCAL:SAVE,FAIL              保存失败
         * ================================================================== */
        case CMD_POWCAL:
        {
            char *arg = str_buf + 10;  /* 跳过 "AT+POWCAL=" */

            /* ----- 查询状态 ----- */
            if (strcmp(arg, "STATUS") == 0)
            {
                for (int i = 0; i < POWCAL_CH_NUM; i++)
                {
                    int ref = powcal_get_ref_mv((powcal_channel_t)i);
                    powcal_status_t st = powcal_get_status((powcal_channel_t)i);
                    const char *state_str =
                        (st == POWCAL_STAT_IDLE)      ? "IDLE" :
                        (st == POWCAL_STAT_ADJ)       ? "ADJ"  :
                        (st == POWCAL_STAT_DONE)      ? "DONE" :
                        (st == POWCAL_STAT_SAT)       ? "SAT"  :
                        (st == POWCAL_STAT_LASER_OFF) ? "LASER_OFF" :
                        (st == POWCAL_STAT_PD_LOW)    ? "PD_LOW" : "NO_RESP";
                    char msg[80];
                    wdt_feed();
                    snprintf(msg, sizeof(msg),
                             "+POWCAL:STATUS,%s,%s,%dmV,target=%d\r\n",
                             powcal_ch_names[i], state_str, ref,
                             powcal_targets[i]);
                    uart9_send_blocking(msg);
                    wdt_feed();  /* 多发消息间喂狗 */
                }
            }
            /* ----- 保存 ----- */
            else if (strcmp(arg, "SAVE") == 0)
            {
                if (powcal_save())
                {
                    wdt_feed();  /* Flash 写入后喂狗 */
                    uart9_send_blocking("+POWCAL:SAVE,OK\r\n");
                    /* 保存成功后自动锁定 */
                    cfg.locked   = true;
                    cfg.front_ok = true;
                    cfg.side_ok  = true;
                    cfg.horiz_ok = true;
                    wdt_feed();  /* 锁定前喂狗 */
                    uart9_send_blocking("+RESP:LOCKED\r\n");
                }
                else
                {
                    uart9_send_blocking("+POWCAL:SAVE,FAIL\r\n");
                }
            }
            /* ----- 停止校准 ----- */
            else if (strncmp(arg, "STOP", 4) == 0)
            {
                /* AT+POWCAL=STOP 或 AT+POWCAL=STOP,H / STOP,V1 / STOP,V2 */
                char *ch_str = NULL;
                if (arg[4] == ',')
                {
                    ch_str = arg + 5;
                }

                if (ch_str != NULL && ch_str[0] != '\0')
                {
                    /* 停止指定通道 */
                    int ch = -1;
                    if (strcmp(ch_str, "H") == 0)       ch = POWCAL_CH_H;
                    else if (strcmp(ch_str, "V1") == 0) ch = POWCAL_CH_V1;
                    else if (strcmp(ch_str, "V2") == 0) ch = POWCAL_CH_V2;

                    if (ch >= 0)
                    {
                        powcal_reset((powcal_channel_t)ch);
                        char msg[40];
                        snprintf(msg, sizeof(msg), "+POWCAL:STOP,%s\r\n", powcal_ch_names[ch]);
                        uart9_send_blocking(msg);
                    }
                    else
                    {
                        uart9_send_blocking("+POWCAL:ERR,UNKNOWN CHANNEL\r\n");
                    }
                }
                else
                {
                    /* 停止全部 */
                    for (int i = 0; i < POWCAL_CH_NUM; i++)
                    {
                        powcal_reset((powcal_channel_t)i);
                    }
                    wdt_feed();  /* 重置全部通道后喂狗 */
                    uart9_send_blocking("+POWCAL:STOP,ALL\r\n");
                }
            }
            /* ----- 光功率读数: AT+POWCAL=<CH>,<power> ----- */
            else
            {
                /* 解析通道名 */
                int ch = -1;
                if      (arg[0] == 'H'  && (arg[1] == ',' || arg[1] == '\0'))
                    ch = POWCAL_CH_H;
                else if (arg[0] == 'V' && arg[1] == '1' && (arg[2] == ',' || arg[2] == '\0'))
                    ch = POWCAL_CH_V1;
                else if (arg[0] == 'V' && arg[1] == '2' && (arg[2] == ',' || arg[2] == '\0'))
                    ch = POWCAL_CH_V2;

                if (ch < 0)
                {
                    uart9_send_blocking("+POWCAL:ERR,FORMAT: AT+POWCAL=<H|V1|V2>,<power_x100>\r\n");
                    break;
                }

                /* 跳过通道名, 找逗号后的数值 */
                const char *val_str = strchr(arg, ',');
                if (val_str == NULL || val_str[1] == '\0')
                {
                    uart9_send_blocking("+POWCAL:ERR,MISSING POWER VALUE\r\n");
                    break;
                }
                val_str++;  /* 跳过逗号 */
                int power = atoi((char *)val_str);

                /* 执行校准 */
                wdt_feed();  /* 校准计算前喂狗 */
                powcal_status_t result = powcal_process((powcal_channel_t)ch, power);
                int ref = powcal_get_ref_mv((powcal_channel_t)ch);
                wdt_feed();  /* 校准计算后喂狗 */

                char msg[60];

                /* ADJ/DONE → 正常格式；其他 → ERR 格式 */
                if (result == POWCAL_STAT_ADJ || result == POWCAL_STAT_DONE)
                {
                    const char *state_str = (result == POWCAL_STAT_ADJ) ? "ADJ" : "DONE";
                    snprintf(msg, sizeof(msg), "+POWCAL:%s,%s,%dmV,%ldmA\r\n",
                             powcal_ch_names[ch], state_str, ref,
                             (long)g_current_mA[ch]);
                }
                else
                {
                    const char *err_str =
                        (result == POWCAL_STAT_SAT)       ? "SAT"  :
                        (result == POWCAL_STAT_LASER_OFF) ? "LASER_OFF" :
                        (result == POWCAL_STAT_PD_LOW)    ? "PD_LOW" : "NO_RESP";
                    snprintf(msg, sizeof(msg), "+POWCAL:ERR,%s,%s,%dmV,%ldmA\r\n",
                             powcal_ch_names[ch], err_str, ref,
                             (long)g_current_mA[ch]);
                }
                uart9_send_blocking(msg);
            }
        }
        break;

        /* ==================================================================
         *  AT+POWTARGET — 运行时设置/查询目标光功率 (写入 Flash 持久化)
         *
         *  格式:
         *    AT+POWTARGET=<CH>,<power_x100>  设置某通道目标光功率
         *    AT+POWTARGET?                  查询三路目标
         *
         *  响应:
         *    +POWTARGET:<CH>,<power_x100>             设置成功
         *    +POWTARGET:H=<val>,V1=<val>,V2=<val>     查询
         *    +POWTARGET:ERR,FORMAT                    格式错误
         * ================================================================== */
        case CMD_POWTARGET:
        {
            /* ----- 查询 ----- */
            if (strcmp(str_buf, "AT+POWTARGET?") == 0)
            {
                char msg[64];
                snprintf(msg, sizeof(msg), "+POWTARGET:H=%d,V1=%d,V2=%d\r\n",
                         powcal_get_target(POWCAL_CH_H),
                         powcal_get_target(POWCAL_CH_V1),
                         powcal_get_target(POWCAL_CH_V2));
                uart9_send_blocking(msg);
                break;
            }

            /* ----- 设置 ----- */
            char *arg = str_buf + 13;  /* 跳过 "AT+POWTARGET=" */

            int ch = -1;
            if      (arg[0] == 'H'  && (arg[1] == ',' || arg[1] == '\0'))
                ch = POWCAL_CH_H;
            else if (arg[0] == 'V' && arg[1] == '1' && (arg[2] == ',' || arg[2] == '\0'))
                ch = POWCAL_CH_V1;
            else if (arg[0] == 'V' && arg[1] == '2' && (arg[2] == ',' || arg[2] == '\0'))
                ch = POWCAL_CH_V2;

            if (ch < 0)
            {
                uart9_send_blocking("+POWTARGET:ERR,FORMAT: AT+POWTARGET=<H|V1|V2>,<power_x100>\r\n");
                break;
            }

            const char *val_str = strchr(arg, ',');
            if (val_str == NULL || val_str[1] == '\0')
            {
                uart9_send_blocking("+POWTARGET:ERR,MISSING VALUE\r\n");
                break;
            }
            val_str++;  /* 跳过逗号 */
            int val = atoi((char *)val_str);

            powcal_set_target((powcal_channel_t)ch, val);

            char msg[48];
            snprintf(msg, sizeof(msg), "+POWTARGET:%s,%d\r\n",
                     powcal_ch_names[ch],
                     powcal_get_target((powcal_channel_t)ch));
            uart9_send_blocking(msg);
            break;
        }

        /* ==================================================================
         *  AT+LASER — 串口控制激光开关
         *
         *  格式:
         *    AT+LASER=<CH>,<ON|OFF>    控制指定通道
         *    AT+LASER=ALL,<ON|OFF>     控制全部三路
         *
         *  响应:
         *    +LASER:<CH>,ON            已开启
         *    +LASER:<CH>,OFF           已关闭
         *    +LASER:ALL,ON             全部已开启
         *    +LASER:ALL,OFF            全部已关闭
         *    +LASER:ERR,BUSY           MOS 状态机忙，稍后重试
         *    +LASER:ERR,FORMAT         格式错误
         * ================================================================== */
        case CMD_LASER_CTRL:
        {
            char *arg = str_buf + 9;  /* 跳过 "AT+LASER=" */

            /* 解析通道名 */
            const char *ch_str = arg;
            const char *val_str = strchr(arg, ',');
            if (val_str == NULL || val_str[1] == '\0')
            {
                uart9_send_blocking("+LASER:ERR,FORMAT: AT+LASER=<H|V1|V2|ALL>,<ON|OFF>\r\n");
                break;
            }

            /* 临时截断通道名以便比较 */
            char ch_name[8] = {0};
            size_t ch_len = (size_t)(val_str - ch_str);
            if (ch_len >= sizeof(ch_name)) ch_len = sizeof(ch_name) - 1;
            memcpy(ch_name, ch_str, ch_len);
            val_str++;  /* 跳过逗号 */

            bool turn_on;
            if (strcmp(val_str, "ON") == 0)
                turn_on = true;
            else if (strcmp(val_str, "OFF") == 0)
                turn_on = false;
            else
            {
                uart9_send_blocking("+LASER:ERR,FORMAT: use ON or OFF\r\n");
                break;
            }

            /* ALL — 控制全部三路（原子操作，避免 MOS 状态机冲突）*/
            if (strcmp(ch_name, "ALL") == 0)
            {
                if (laser_serial_ctrl_all(turn_on))
                {
                    char msg[32];
                    snprintf(msg, sizeof(msg), "+LASER:ALL,%s\r\n",
                             turn_on ? "ON" : "OFF");
                    uart9_send_blocking(msg);
                }
                else
                {
                    uart9_send_blocking("+LASER:ERR,BUSY\r\n");
                }
                break;
            }

            /* 单路操作 */
            int idx = -1;
            if (strcmp(ch_name, "H") == 0)       idx = 0;
            else if (strcmp(ch_name, "V1") == 0) idx = 1;
            else if (strcmp(ch_name, "V2") == 0) idx = 2;

            if (idx < 0)
            {
                uart9_send_blocking("+LASER:ERR,FORMAT: unknown channel, use H/V1/V2/ALL\r\n");
                break;
            }

            if (laser_serial_ctrl((uint8_t)idx, turn_on))
            {
                char msg[32];
                snprintf(msg, sizeof(msg), "+LASER:%s,%s\r\n",
                         ch_name, turn_on ? "ON" : "OFF");
                uart9_send_blocking(msg);
            }
            else
            {
                uart9_send_blocking("+LASER:ERR,BUSY\r\n");
            }
        }
        break;

        /* ==================================================================
         *  5 — 进入固件更新模式
         *
         *  功能: 通过 Code Flash 写入 ENTER_UPDATE_MODE 通知 Bootloader 进入 XModem 固件更新模式
         *  响应: +FWUPDATE:REBOOT  然后 MCU 复位
         * ================================================================== */
        case CMD_FW_UPDATE:
        {
            fsp_err_t    err;
            uint32_t     update_flag = ENTER_UPDATE_MODE;
            uint32_t     saved_primask;
            uint32_t     saved_systick;

            uart9_send_blocking("+FWUPDATE:ERASING OTA FLAG...\r\n");

            /* Ensure flash driver is open */
            err = g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);
            if (FSP_SUCCESS != err)
            {
                /* May already be open — try to continue */
            }

            /* --- Critical section: disable interrupts for code flash operation --- */
            saved_primask = __get_PRIMASK();
            saved_systick = SysTick->CTRL;
            SysTick->CTRL  = 0;
            NVIC_DisableIRQ(SysTick_IRQn);
            NVIC_ClearPendingIRQ(SysTick_IRQn);
            __disable_irq();

            /* Erase last 2KB block (0x1E000-0x1FFFF) containing OTA flag */
            err = g_flash0.p_api->erase(g_flash0.p_ctrl, 0x0001E000, 1);
            if (FSP_SUCCESS == err)
            {
                /* Write 0x5555AAAA to OTA flag address (0x1FFFC) */
                err = g_flash0.p_api->write(g_flash0.p_ctrl,
                                            (uint32_t)&update_flag,
                                            OTA_FLAG_ADDR, 4);
            }

            /* --- Restore interrupts --- */
            SysTick->CTRL = saved_systick;
            NVIC_EnableIRQ(SysTick_IRQn);
            __set_PRIMASK(saved_primask);

            /* Close flash driver */
            g_flash0.p_api->close(g_flash0.p_ctrl);

            wdt_feed();

            if (FSP_SUCCESS == err)
            {
                uart9_send_blocking("+FWUPDATE:REBOOTING...\r\n");
            }
            else
            {
                uart9_send_blocking("+FWUPDATE:FLASH WRITE FAILED\r\n");
            }

            /* Delay to let UART TX complete */
            for (volatile uint32_t d = 0; d < 500000UL; d++) { __NOP(); }

            /* Reboot into bootloader */
            NVIC_SystemReset();
            while (1) { }
        }
        break;

        default:
        {
        }
        break;
    }


    // �?三路设置完成后自动上锁（只执行一次）
    if (!cfg.locked && cfg.front_ok && cfg.side_ok && cfg.horiz_ok)
    {
        cfg.locked = true;
        uart9_send_blocking("+RESP:LOCKED\r\n");
    }

    memset((void *)rx_buffer, 0, RX_BUFFER_SIZE);
    R_SCI_UART_Read(&g_uart9_ctrl, (uint8_t *)rx_buffer, RX_BUFFER_SIZE);
}




