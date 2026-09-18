/*
 * menu.c
 *
 *  Created on: 2 Oct 2019
 *      Author: b3800274
 */
#include "menu.h"
#include "header.h"
#include "comms/comms.h"
#include "xmodem.h"
//#include "downloader_thread.h"    WangJin 2021.04.06
#include "header.h"
#include "crc16.h"

extern void do_boot(struct boot_rsp *rsp);

//extern struct boot_rsp;
struct boot_rsp *rsp;
//const char * gp_banner[] = {
//                            "  /\\/\\   ___ _ __  _   _",
//                            " /    \\ / _ \\ '_ \\| | | |",
//                            "/ /\\/\\ \\  __/ | | | |_| |",
//                            "\\/    \\/\\___|_| |_|\\__,_|",
//};

/*
 * Firmware update via XModem — reusable function.
 * Can be called from menu or auto-update path.
 * Returns 1 on success (OTA flag set), 0 on failure.
 */
int firmware_update_via_xmodem(void)
{
    volatile fsp_err_t err;
    uint8_t tx_str[80];
    uint32_t CRC_rom;
    uint8_t OTA_FLAG_END[4] = {0x55, 0x55, 0x55, 0x55};
    unsigned char xm_err;

    /* Blank check */
    sprintf((char *)tx_str, "Blank checking the App area...\r\n");
    comms_send(tx_str, strlen((char *)tx_str));
    flash_result_t fresult;
    err = g_flash0.p_api->blankCheck(g_flash0.p_ctrl, APP_IMAGE_START_ADDRESS,
                                     (APP_IMAGE_END_ADDRESS - APP_IMAGE_START_ADDRESS), &fresult);
    if (FSP_SUCCESS != err)
    {
        sprintf((char *)tx_str, "ERROR: Blank checking the App area\r\n");
        comms_send(tx_str, strlen((char *)tx_str));
        return 0;
    }

    if (fresult != FLASH_RESULT_BLANK)
    {
        sprintf((char *)tx_str, "Erasing the App area...\r\n");
        comms_send(tx_str, strlen((char *)tx_str));
        ThreadsAndInterrupts(DISABLE);
        err = g_flash0.p_api->erase(g_flash0.p_ctrl, APP_IMAGE_START_ADDRESS, APP_IMAGE_NUM_BLOCKS);
        ThreadsAndInterrupts(RE_ENABLE);
        if (FSP_SUCCESS != err)
        {
            sprintf((char *)tx_str, "ERROR: Erasing the App area\r\n");
            comms_send(tx_str, strlen((char *)tx_str));
            return 0;
        }
        sprintf((char *)tx_str, "App area erased\r\n");
        comms_send(tx_str, strlen((char *)tx_str));
    }
    else
    {
        sprintf((char *)tx_str, "App area blank\r\n");
        comms_send(tx_str, strlen((char *)tx_str));
    }

    /* XModem download and flash programming */
    sprintf((char *)tx_str, "Start Xmodem transfer...\r\n");
    comms_send(tx_str, strlen((char *)tx_str));
    xm_err = XmodemDownloadAndProgramFlash(APP_IMAGE_START_ADDRESS);
    if (XM_OK != xm_err)
    {
        switch (xm_err)
        {
            case XM_ADDRESS_ERROR:
                sprintf((char *)tx_str, "ERROR: Flash address invalid\r\n");
                break;
            case XM_TIMEOUT:
                sprintf((char *)tx_str, "ERROR: Timeout during Xmodem download\r\n");
                break;
            case XM_PROG_FAIL:
                sprintf((char *)tx_str, "ERROR: Flash programming error\r\n");
                break;
            default:
                sprintf((char *)tx_str, "ERROR: unknown (%d)\r\n", xm_err);
                break;
        }
        comms_send(tx_str, strlen((char *)tx_str));
        return 0;
    }

    sprintf((char *)tx_str, "Flash image download successful\r\n");
    comms_send(tx_str, strlen((char *)tx_str));

    /* Calculate CRC of downloaded image */
    CRC_rom = calcrc(APP_IMAGE_START_ADDRESS, (CRC_ADDRESS - APP_IMAGE_START_ADDRESS));
    sprintf((char *)tx_str, "CRC: %04X\r\n", CRC_rom);
    comms_send(tx_str, strlen((char *)tx_str));

    /* Write CRC to flash */
    {
        uint32_t crc_data = (uint32_t)CRC_rom;
        ThreadsAndInterrupts(DISABLE);
        err = g_flash0.p_api->write(g_flash0.p_ctrl, (uint32_t)&crc_data, CRC_ADDRESS, 4);
        ThreadsAndInterrupts(RE_ENABLE);
        if (FSP_SUCCESS != err)
        {
            sprintf((char *)tx_str, "Write CRC to flash failed\r\n");
            comms_send(tx_str, strlen((char *)tx_str));
            return 0;
        }
    }

    /* Set OTA flag */
    sprintf((char *)tx_str, "Setting the OTA flag\r\n");
    comms_send(tx_str, strlen((char *)tx_str));
    ThreadsAndInterrupts(DISABLE);
    err = g_flash0.p_api->write(g_flash0.p_ctrl, (uint32_t)&OTA_FLAG_END[0], OTA_FLAG_END_ADDRESS, 4);
    ThreadsAndInterrupts(RE_ENABLE);
    if (FSP_SUCCESS != err)
    {
        sprintf((char *)tx_str, "Set OTA flag failed\r\n");
        comms_send(tx_str, strlen((char *)tx_str));
        return 0;
    }

    sprintf((char *)tx_str, "OTA update complete, OTA flag set\r\n");
    comms_send(tx_str, strlen((char *)tx_str));
    return 1;
}

void menu(void)
{
//    struct boot_rsp rsp;
    volatile fsp_err_t err = 0;
    uint8_t tx_str[80];
    uint8_t rx_str[20] = {0};
    uint32_t len;
    uint8_t     reselect = 1;
    uint32_t    flag_end = 0xFFFF;
    uint32_t  CRC_rom, CRC_ccitt;

    FSP_PARAMETER_NOT_USED(err);

    while (1) {
        sprintf((char *)tx_str, "\n\r1 - Update Boot + App\n\r");
        comms_send(tx_str, strlen((char *)tx_str));
        sprintf((char *)tx_str, "2 - Update App ONLY (XModem)\n\r");
        comms_send(tx_str, strlen((char *)tx_str));
        sprintf((char *)tx_str, "3 - Reboot\n\r");
        comms_send(tx_str, strlen((char *)tx_str));
        comms_send((uint8_t *)">", 1);

        rx_str[0] = 0;
        len = 1;
        err = comms_read(rx_str, &len, portMAX_DELAY);
        if(err != FSP_SUCCESS)
            break;

        switch (rx_str[0]){
            case '1':   //Update Boot + App area
                sprintf((char *)tx_str, "\n\rSelect file to write into flash area\n\r");
                comms_send(tx_str, strlen((char *)tx_str));
                break;

            case '2':   //Update App ONLY
                if (firmware_update_via_xmodem())
                {
                    reselect = 0;
                }
                break;

            default:
                sprintf((char *)tx_str, "\n\rSelect again.\n\r");
                comms_send(tx_str, strlen((char *)tx_str));
                reselect = 1;
                break;
        }
        if(reselect == 0)
            return;
    comms_send(rx_str, 1);
    comms_send((uint8_t *)"\n\r", 2);
    }

    flag_end = *(uint32_t *)OTA_FLAG_END_ADDRESS;

    if( 0x55555555 == flag_end ){
        sprintf((char *)tx_str, "\n\rJumping to App\n\r");
        comms_send(tx_str, strlen((char *)tx_str));
        do_boot(&rsp);
    }
    else if( 0xFFFFFFFF == flag_end ){
        CRC_rom = calcrc(APP_IMAGE_START_ADDRESS, (CRC_ADDRESS - APP_IMAGE_START_ADDRESS));
        CRC_ccitt = (unsigned short)(*(unsigned int *)CRC_ADDRESS);

        if( CRC_rom == CRC_ccitt ){
            sprintf((char *)tx_str, "CRC OK...Jumping to App\r\n");
            comms_send(tx_str, strlen((char *)tx_str));
            do_boot(&rsp);
        }
        else{
            sprintf((char *)tx_str, "CRC_rom: %08X, CRC_ccitt: %08X, CRC NG..\r\n", CRC_rom, CRC_ccitt);
            comms_send(tx_str, strlen((char *)tx_str));
        }
//        CRC_rom = calcrc(0x4000, 4);
    }
    else{
        sprintf((char *)tx_str, "OTA flag corrupted (0x%08X), re-entering menu...\r\n", (unsigned int)flag_end);
        comms_send(tx_str, strlen((char *)tx_str));
        /* Continue the outer while(1) — re-enter menu instead of infinite reset */
    }
}

void ThreadsAndInterrupts(enable_disable_t EnableDisable)
{
    static uint32_t control_reg_value;
    static uint32_t old_primask;

    if( DISABLE == EnableDisable )
    {
        /** Store the interrupt state */
        old_primask = __get_PRIMASK();

        /* Disable other threads whilst flash erasing */
//        tx_thread_suspend(& _ux_system_host->ux_system_host_hcd_thread );

        /* Disable the SysTick timer */
        control_reg_value = SysTick->CTRL;
        SysTick->CTRL = 0;
        NVIC_DisableIRQ( SysTick_IRQn ); /* Disable the SysTick timer IRQ */
        NVIC_ClearPendingIRQ( SysTick_IRQn ); /* Clear any pending SysTick timer IRQ */

        __disable_irq(); /* Disable interrupts */
    }
    else
    {
        NVIC_EnableIRQ( SysTick_IRQn ); /* Enable the SysTick timer IRQ */
        SysTick->CTRL = control_reg_value; /* Restart the SysTick timer */

        /** Restore the interrupt state */
        __set_PRIMASK( old_primask ); /* Enable interrupts */

        /* Resume the threads */
//        tx_thread_resume(& _ux_system_host->ux_system_host_hcd_thread );
    }
}
