#include "../comms.h"
#include "hal_data.h"
#include "common_data.h"
#include "header.h"

#ifdef COMMS_UART

static volatile bool g_tx_complete = false;
static volatile bool g_rx_complete = false;
static volatile bool g_timer0_event = false;

void g_timer0_callback(timer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    g_timer0_event = true;
}

void user_uart_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
            g_tx_complete = true;
            break;

        case UART_EVENT_RX_COMPLETE:
            g_rx_complete = true;
            break;

        default:
            break;
    }
}

/*
 * comms_send — 阻塞式发送.
 *
 * 使用 g_uart0.p_api->write() 实例 API (而非直接调用 R_SCI_UART_Write),
 * 确保回调机制正确触发.
 */
void comms_send(uint8_t *p_src, uint32_t len)
{
    if ((NULL == p_src) || (0U == len)) return;

    g_tx_complete = false;
    fsp_err_t err = g_uart0.p_api->write(g_uart0.p_ctrl, p_src, len);
    if (FSP_SUCCESS != err) return;

    while (!g_tx_complete)
    {
        __NOP();
    }
}

/*
 * comms_read — 带超时的阻塞式接收.
 */
fsp_err_t comms_read(uint8_t *p_dest, uint32_t *len, uint32_t timeout_milliseconds)
{
    fsp_err_t err;

    /* 配置并启动超时定时器 */
    err = g_timer0.p_api->stop(g_timer0.p_ctrl);
    if (FSP_SUCCESS != err) return err;

    err = g_timer0.p_api->periodSet(g_timer0.p_ctrl, (RAW_COUNT_MS * timeout_milliseconds));
    if (FSP_SUCCESS != err) return err;

    g_timer0_event = false;
    err = g_timer0.p_api->start(g_timer0.p_ctrl);
    if (FSP_SUCCESS != err) return err;

    /* 启动接收 */
    g_rx_complete = false;
    err = g_uart0.p_api->read(g_uart0.p_ctrl, p_dest, *len);
    if (FSP_SUCCESS != err)
    {
        g_timer0.p_api->stop(g_timer0.p_ctrl);
        return err;
    }

    /* 等待接收完成或超时 */
    while (!g_rx_complete && !g_timer0_event)
    {
        __NOP();
    }

    g_timer0.p_api->stop(g_timer0.p_ctrl);

    if (g_rx_complete)
    {
        return FSP_SUCCESS;
    }
    else
    {
        /* 超时: 取消未完成的接收 */
        g_uart0.p_api->communicationAbort(g_uart0.p_ctrl, UART_DIR_RX);
        return FSP_ERR_TIMEOUT;
    }
}

/*
 * comms_read_blocking — 永久阻塞式接收 (无超时, 对齐 abs_bootloader 行为).
 */
fsp_err_t comms_read_blocking(uint8_t *p_dest, uint32_t *len)
{
    fsp_err_t err;

    g_rx_complete = false;
    err = g_uart0.p_api->read(g_uart0.p_ctrl, p_dest, *len);
    if (FSP_SUCCESS != err) return err;

    while (!g_rx_complete)
    {
        __NOP();
    }
    return FSP_SUCCESS;
}

/*
 * comms_set_baud — 运行时修改波特率.
 */
fsp_err_t comms_set_baud(uint32_t rate)
{
    fsp_err_t err;
    baud_setting_t baud_setting;

    err = R_SCI_UART_BaudCalculate(rate, false, 500, &baud_setting);
    if (FSP_SUCCESS != err) return err;

    err = g_uart0.p_api->baudSet(g_uart0.p_ctrl, &baud_setting);
    return err;
}

#endif
