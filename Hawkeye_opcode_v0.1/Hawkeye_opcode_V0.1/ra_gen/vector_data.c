/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = sci_uart_rxi_isr, /* SCI0 RXI (Receive data full) */
            [1] = sci_uart_txi_isr, /* SCI0 TXI (Transmit data empty) */
            [2] = gpt_counter_overflow_isr, /* GPT9 COUNTER OVERFLOW (Overflow) */
            [3] = sci_uart_eri_isr, /* SCI0 ERI (Receive error) */
            [4] = adc_scan_end_isr, /* ADC0 SCAN END (End of A/D scanning operation) */
            [5] = iic_master_txi_isr, /* IIC0 TXI (Transmit data empty) */
            [6] = sci_uart_tei_isr, /* SCI0 TEI (Transmit end) */
            [7] = iic_master_eri_isr, /* IIC0 ERI (Transfer error) */
            [8] = iic_master_rxi_isr, /* IIC0 RXI (Receive data full) */
            [10] = iic_master_tei_isr, /* IIC0 TEI (Transmit end) */
            [12] = agt_int_isr, /* AGT1 INT (AGT interrupt) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_SCI0_RXI,GROUP0), /* SCI0 RXI (Receive data full) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_SCI0_TXI,GROUP1), /* SCI0 TXI (Transmit data empty) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_GPT9_COUNTER_OVERFLOW,GROUP2), /* GPT9 COUNTER OVERFLOW (Overflow) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_SCI0_ERI,GROUP3), /* SCI0 ERI (Receive error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_ADC0_SCAN_END,GROUP4), /* ADC0 SCAN END (End of A/D scanning operation) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_IIC0_TXI,GROUP5), /* IIC0 TXI (Transmit data empty) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI0_TEI,GROUP6), /* SCI0 TEI (Transmit end) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_IIC0_ERI,GROUP7), /* IIC0 ERI (Transfer error) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_IIC0_RXI,GROUP0), /* IIC0 RXI (Receive data full) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_IIC0_TEI,GROUP2), /* IIC0 TEI (Transmit end) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_AGT1_INT,GROUP4), /* AGT1 INT (AGT interrupt) */
        };
        #endif
        #endif
