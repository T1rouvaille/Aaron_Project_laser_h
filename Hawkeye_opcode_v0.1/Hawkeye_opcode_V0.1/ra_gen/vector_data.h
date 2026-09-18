/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
        extern "C" {
        #endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (11)
#endif
/* ISR prototypes */
void sci_uart_rxi_isr(void);
void sci_uart_txi_isr(void);
void gpt_counter_overflow_isr(void);
void sci_uart_eri_isr(void);
void adc_scan_end_isr(void);
void iic_master_txi_isr(void);
void sci_uart_tei_isr(void);
void iic_master_eri_isr(void);
void iic_master_rxi_isr(void);
void iic_master_tei_isr(void);
void agt_int_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_SCI0_RXI ((IRQn_Type) 0) /* SCI0 RXI (Receive data full) */
#define SCI0_RXI_IRQn          ((IRQn_Type) 0) /* SCI0 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI0_TXI ((IRQn_Type) 1) /* SCI0 TXI (Transmit data empty) */
#define SCI0_TXI_IRQn          ((IRQn_Type) 1) /* SCI0 TXI (Transmit data empty) */
#define VECTOR_NUMBER_GPT9_COUNTER_OVERFLOW ((IRQn_Type) 2) /* GPT9 COUNTER OVERFLOW (Overflow) */
#define GPT9_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 2) /* GPT9 COUNTER OVERFLOW (Overflow) */
#define VECTOR_NUMBER_SCI0_ERI ((IRQn_Type) 3) /* SCI0 ERI (Receive error) */
#define SCI0_ERI_IRQn          ((IRQn_Type) 3) /* SCI0 ERI (Receive error) */
#define VECTOR_NUMBER_ADC0_SCAN_END ((IRQn_Type) 4) /* ADC0 SCAN END (End of A/D scanning operation) */
#define ADC0_SCAN_END_IRQn          ((IRQn_Type) 4) /* ADC0 SCAN END (End of A/D scanning operation) */
#define VECTOR_NUMBER_IIC0_TXI ((IRQn_Type) 5) /* IIC0 TXI (Transmit data empty) */
#define IIC0_TXI_IRQn          ((IRQn_Type) 5) /* IIC0 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI0_TEI ((IRQn_Type) 6) /* SCI0 TEI (Transmit end) */
#define SCI0_TEI_IRQn          ((IRQn_Type) 6) /* SCI0 TEI (Transmit end) */
#define VECTOR_NUMBER_IIC0_ERI ((IRQn_Type) 7) /* IIC0 ERI (Transfer error) */
#define IIC0_ERI_IRQn          ((IRQn_Type) 7) /* IIC0 ERI (Transfer error) */
#define VECTOR_NUMBER_IIC0_RXI ((IRQn_Type) 8) /* IIC0 RXI (Receive data full) */
#define IIC0_RXI_IRQn          ((IRQn_Type) 8) /* IIC0 RXI (Receive data full) */
#define VECTOR_NUMBER_IIC0_TEI ((IRQn_Type) 10) /* IIC0 TEI (Transmit end) */
#define IIC0_TEI_IRQn          ((IRQn_Type) 10) /* IIC0 TEI (Transmit end) */
#define VECTOR_NUMBER_AGT1_INT ((IRQn_Type) 12) /* AGT1 INT (AGT interrupt) */
#define AGT1_INT_IRQn          ((IRQn_Type) 12) /* AGT1 INT (AGT interrupt) */
#ifdef __cplusplus
        }
        #endif
#endif /* VECTOR_DATA_H */
