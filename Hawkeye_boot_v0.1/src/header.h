/*
 * header.h
 * ABS Bootloader configuration for RA2E1
 * Based on ABS_BOOTLOADER specifications
 */

#ifndef HEADER_H_
#define HEADER_H_


/* LED OFF (低电平有效) — 已重映射为 UBOX 引脚 (P915/P914/P913) */
#define LED1_ON    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_02_PIN_06, BSP_IO_LEVEL_LOW)
#define LED2_ON    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_02_PIN_07, BSP_IO_LEVEL_LOW)
#define LED3_ON    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_09_PIN_13, BSP_IO_LEVEL_LOW)
/* LED ON (高电平有效) */
#define LED1_OFF   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_02_PIN_06, BSP_IO_LEVEL_HIGH)
#define LED2_OFF   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_02_PIN_07, BSP_IO_LEVEL_HIGH)
#define LED3_OFF   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_09_PIN_13, BSP_IO_LEVEL_HIGH)
#define portMAX_DELAY              10        //5 seconds time out for receiving command from PC
#define MENU_TIMEOUT_MS            10       //UBOX menu input timeout (ms)

/* UBOX 专属：App 写入此魔数触发 XMODEM 固件升级 */
#define ENTER_UPDATE_MODE           0x5555AAAA

/* Flash memory configuration */
#define FLASH_BLOCK_SIZE                (2 * 1024)      // RA2E1 minimum erase unit
#define APP_IMAGE_START_ADDRESS         0x00004000
#define APP_IMAGE_END_ADDRESS           0x00020000
#define OTA_FLAG_END_ADDRESS            (APP_IMAGE_END_ADDRESS - 4)
#define CRC_ADDRESS                     (APP_IMAGE_END_ADDRESS - 8)
#define APP_IMAGE_NUM_BLOCKS            ((APP_IMAGE_END_ADDRESS - APP_IMAGE_START_ADDRESS) / FLASH_BLOCK_SIZE)

/* ABS Bootloader specific definitions */
#define FMC_APROM_BASE                  0x00004000     /* APROM Base Address */
#define PAGE_SIZE                       0x200U         /* 512 bytes per page */

/* Communication baud rates */
#define UART_BAUD_FAST                  115200U
#define UART_BAUD_SLOW                  9600U

/* ================================================================== */
/*  Command definitions — Honcho PR3 IBOX protocol                      */
/* ================================================================== */
#define COMMAND_NACK                    0x01U
#define COMMAND_HANDSHAKE               'B'
#define COMMAND_IDENTIFY                'I'
#define COMMAND_READ_EEPROM             'F'
#define COMMAND_READ_FLASH              'N'
#define COMMAND_LOAD_FLASH              'M'
#define COMMAND_SET_ADDRESS             'D'
#define COMMAND_CHANGE_BAUD             '*'
#define ETX                             0x03U

/* ================================================================== */
/*  Module identification — Honcho PR3 personalization.h format         */
/* ================================================================== */
#define MODULE_ID_FIRST         {'N','A','8','3','7','3','3','0'}
#define MODULE_ID_SECOND        {'N','A','8','3','7','3','3','0'}
#define MODULE_REVISION         0x04U

/* Honcho protocol constants */
#define BYTES_MODULE_ID         8U
#define BYTES_PART_NUMBER       8U
#define BYTES_PERSONALIZATION   40U
#define BYTES_SERIALIZATION     16U

/* Version info payload size (Honcho format: 'N' + lenH + lenL + 86 bytes payload + ETX = 90 bytes) */
#define VERSION_PAYLOAD_SIZE    86U

/* Bootloader version — matching Honcho PR3 */
#define SW_VERSION_MAJOR        0x00U
#define SW_VERSION_MINOR        0x01U

/* Tool identity strings — matching Honcho PR3 personalization.h */
#define G_TOOL_PART_NUMBER      "NA837330"
#define G_PERSONALIZATION       "MSL2 MARCHTHR Hawkeye Laser M1 20VHE OCT"

/* RA2E1 Unique ID register */
#define UCID_ADDRESS            0x01001C00U

/* Special address offsets for version/calibration requests (Honcho protocol) */
#define VERSION_REQUEST_OFFSET      0x0000FFFFU
#define CALIBRATION_REQUEST_OFFSET  0x0000FFFEU

/* ================================================================== */
/*  Calibration request flag — shared with APP via Data Flash            */
/*  UBOX 收到 Honcho 校准命令后写此 flag, APP 启动时检测并执行校准      */
/* ================================================================== */
#define FLASH_DF_BASE_ADDR          0x40100000U    /* RA2E1 Data Flash 基地址 */
#define DATALOG_SIZE                192U            /* Data Flash datalog: 3 blocks × 64 bytes */
#define CALIB_REQ_FLAG_OFFSET       12U             /* 第1块 offset 12 (校准数据 reserved 之后) */
#define CALIB_REQ_FLAG_ADDR         (FLASH_DF_BASE_ADDR + CALIB_REQ_FLAG_OFFSET)
#define CALIB_REQ_FLAG_MAGIC        0xCA11B000U     /* "CALIB" 谐音 */

typedef enum e_enable_disable
{
    DISABLE,
    RE_ENABLE
} enable_disable_t;

/* Union for 32-bit address/data handling */
typedef union tuReg32
{
    uint32_t Val32;
    struct
    {
        uint16_t LW;
        uint16_t HW;
    } Word;
    uint8_t Val[4];
} uReg32;

void ThreadsAndInterrupts(enable_disable_t EnableDisable);
void display_image_slot_info(void);

#endif /* HEADER_H_ */
