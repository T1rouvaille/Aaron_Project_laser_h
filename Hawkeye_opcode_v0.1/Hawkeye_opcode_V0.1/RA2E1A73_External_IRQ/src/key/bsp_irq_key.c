/*
 * bsp_key.c
 *
 *  Created on: Sep 18, 2024
 *      Author: AXQ0527A
 */
#include <key/bsp_irq_key.h>

volatile bool key1_sw2_press = false;

void Key_Init(void)
{
    R_IOPORT_Open (&g_ioport_ctrl, g_ioport.p_cfg);
}

uint32_t Key_Scan(bsp_io_port_pin_t key)
{
   bsp_io_level_t state;

   // 读取按键引脚电平
   R_IOPORT_PinRead(&g_ioport_ctrl, key, &state);
   if (BSP_IO_LEVEL_HIGH == state)
   {
      return KEY_OFF; //按键没有被按下
   }
   else
   {
      do  //等待按键释放
      {
            R_IOPORT_PinRead(&g_ioport_ctrl, key, &state);
      } while (BSP_IO_LEVEL_LOW == state);
   }

   return KEY_ON; //按键被按下了
}

void Key_IRQ_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /* Open ICU module */
    err = R_ICU_ExternalIrqOpen(&g_external_irq7_ctrl, &g_external_irq7_cfg);

    err = R_ICU_ExternalIrqEnable(&g_external_irq7_ctrl);

}

void key_external_irq_callback(external_irq_callback_args_t *p_args)
{
    if (7 == p_args->channel)
    {
    key1_sw2_press = true;
    }


}
