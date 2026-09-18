/***********************************************************************************************************************
 * File Name    : timer_pwm.c
 * Description  : Contains timer functions definition.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2020 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/

#include "common_utils.h"
#include "timer_pwm.h"
#include <SysTick/bsp_SysTick.h>

/*******************************************************************************************************************//**
 * @addtogroup r_sci_uart_ep
 * @{
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * @brief       Initialize GPT in PWM mode.
 * @param[in]   None
 * @retval      FSP_SUCCESS         Upon successful open of timer
 * @retval      Any Other Error code apart from FSP_SUCCESS  Unsuccessful open
 **********************************************************************************************************************/
fsp_err_t gpt_initialize(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Open GPT module */
    err = R_GPT_Open (&g_timer7_ctrl, &g_timer7_cfg);
    if (FSP_SUCCESS != err)
    {

        APP_ERR_PRINT ("\r\n** R_GPT_TimerOpen API failed **\r\n");
    }
    err = R_GPT_Open (&g_timer4_ctrl, &g_timer4_cfg);//P100 P101
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n** R_GPT_TimerOpen API failed **\r\n");
    }
    err = R_GPT_Open (&g_timer5_ctrl, &g_timer5_cfg);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n** R_GPT_TimerOpen API failed **\r\n");
    }

    err = R_GPT_Open (&g_timer6_ctrl, &g_timer6_cfg);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n** R_GPT_TimerOpen API failed **\r\n");
    }
    return err;
}

/*******************************************************************************************************************//**
 * @brief       Start GPT in PWM mode.
 * @param[in]   None
 * @retval      FSP_SUCCESS         Upon successful start of timer
 * @retval      Any Other Error code apart from FSP_SUCCESS  Unsuccessful start
 **********************************************************************************************************************/
fsp_err_t gpt_start(void)
{

    fsp_err_t err = FSP_SUCCESS;

    /* Start GPT module */
    err=  R_GPT_Start (&g_timer7_ctrl);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n ** R_GPT_Start API failed **\r\n");
    }
    err=  R_GPT_Start (&g_timer4_ctrl);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n ** R_GPT_Start API failed **\r\n");

    }
    err=  R_GPT_Start (&g_timer5_ctrl);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n ** R_GPT_Start API failed **\r\n");

    }

    err=  R_GPT_Start (&g_timer6_ctrl);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n ** R_GPT_Start API failed **\r\n");

    }
    return err;

}

fsp_err_t set_laser1_200k_intensity(uint32_t raw_count, uint8_t pin)
{
    fsp_err_t err = FSP_SUCCESS;
#if defined(BOARD_RA4W1_EK) || defined (BOARD_RA6T1_RSSK)
    raw_count = (MAX_DUTY_CYCLE - raw_count);
#endif
    /* Set GPT timer's DutyCycle as per user input */
    err = R_GPT_DutyCycleSet (&g_timer4_ctrl, raw_count, pin);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n ** R_GPT_DutyCycleSet API failed **\r\n");
    }
    return err;
}
fsp_err_t set_laser2_200k_intensity(uint32_t raw_count, uint8_t pin)
{
    fsp_err_t err = FSP_SUCCESS;
#if defined(BOARD_RA4W1_EK) || defined (BOARD_RA6T1_RSSK)
    raw_count = (MAX_DUTY_CYCLE - raw_count);
#endif
    /* Set GPT timer's DutyCycle as per user input */
    err = R_GPT_DutyCycleSet (&g_timer7_ctrl, raw_count, pin);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n ** R_GPT_DutyCycleSet API failed **\r\n");
    }
    return err;
}

fsp_err_t set_laser3_200k_intensity(uint32_t raw_count, uint8_t pin)
{
    fsp_err_t err = FSP_SUCCESS;
#if defined(BOARD_RA4W1_EK) || defined (BOARD_RA6T1_RSSK)
    raw_count = (MAX_DUTY_CYCLE - raw_count);
#endif
    /* Set GPT timer's DutyCycle as per user input */
    err = R_GPT_DutyCycleSet (&g_timer7_ctrl, raw_count, pin);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n ** R_GPT_DutyCycleSet API failed **\r\n");
    }
    return err;
}
fsp_err_t set_laser1_8470_intensity(uint32_t raw_count, uint8_t pin)
{
    fsp_err_t err = FSP_SUCCESS;
#if defined(BOARD_RA4W1_EK) || defined (BOARD_RA6T1_RSSK)
    raw_count = (MAX_DUTY_CYCLE - raw_count);
#endif
    /* Set GPT timer's DutyCycle as per user input */
    err = R_GPT_DutyCycleSet (&g_timer6_ctrl, raw_count, pin);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n ** R_GPT_DutyCycleSet API failed **\r\n");
    }
    return err;
}
fsp_err_t set_laser2_8470_intensity(uint32_t raw_count, uint8_t pin)
{
    fsp_err_t err = FSP_SUCCESS;
#if defined(BOARD_RA4W1_EK) || defined (BOARD_RA6T1_RSSK)
    raw_count = (MAX_DUTY_CYCLE - raw_count);
#endif
    /* Set GPT timer's DutyCycle as per user input */
    err = R_GPT_DutyCycleSet (&g_timer6_ctrl, raw_count, pin);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n ** R_GPT_DutyCycleSet API failed **\r\n");
    }
    return err;
}

fsp_err_t set_laser3_8470_intensity(uint32_t raw_count, uint8_t pin)
{
    fsp_err_t err = FSP_SUCCESS;
#if defined(BOARD_RA4W1_EK) || defined (BOARD_RA6T1_RSSK)
    raw_count = (MAX_DUTY_CYCLE - raw_count);
#endif
    /* Set GPT timer's DutyCycle as per user input */
    err = R_GPT_DutyCycleSet (&g_timer5_ctrl, raw_count, pin);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n ** R_GPT_DutyCycleSet API failed **\r\n");
    }
    return err;
}

/*******************************************************************************************************************//**
 * @brief       Close the GPT HAL driver before the project ends up in an Error Trap.
 * @param[in]   None
 * @retval      None
 **********************************************************************************************************************/
void timer_gpt_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Close the GPT module */
    //err = R_GPT_Close (&g_timer5_ctrl);
    if (FSP_SUCCESS != err)
    {
        /* GPT Close failure message */
        APP_ERR_PRINT ("\r\n ** R_GPT_Close API failed **\r\n");
    }

}
void timer5_stop(void)
{
    fsp_err_t err = FSP_SUCCESS;
    //err = R_GPT_Stop(&g_timer5_ctrl);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n** GPT failed while changing intensity ** \r\n");

    }

}
void timer2_stop(void)
{
    fsp_err_t err = FSP_SUCCESS;
    //err = R_GPT_Stop(&g_timer2_ctrl);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n** GPT failed while changing intensity ** \r\n");

    }

}


/*******************************************************************************************************************//**
 * @} (end addtogroup r_sci_uart_ep)
 **********************************************************************************************************************/
