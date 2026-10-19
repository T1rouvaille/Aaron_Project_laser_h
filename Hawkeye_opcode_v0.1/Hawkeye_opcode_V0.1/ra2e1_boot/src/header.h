/*
 * header.h
 *
 *  Created on: 2 Oct 2019
 *      Author: b3800274
 */

#ifndef HEADER_H_
#define HEADER_H_

//#define portMAX_DELAY              0xFFFFFFFF
#define portMAX_DELAY              5000       //5 seconds time out for receiving command from PC

//#define PRIMARY_IMAGE_START_ADDRESS     0x00004000      //0x00010000 --> 0x00004000
//#define SECONDARY_IMAGE_START_ADDRESS   0x00020000      // 0x00100000 --> 0x00020000 WangJin 2021.03.05
//#define SECONDARY_IMAGE_END_ADDRESS     0x0003C000      // 0x001F0000 --> 0x000F0000 WangJin 2021.03.05
#define FLASH_BLOCK_SIZE                (2 * 1024)      // (32 * 1024) --> 2 * 1024
#define APP_IMAGE_START_ADDRESS         0x00004000
#define APP_IMAGE_END_ADDRESS           0x00020000
//#define OTA_FLAG_START_ADDRESS          (APP_IMAGE_END_ADDRESS - 8)
#define OTA_FLAG_END_ADDRESS          (APP_IMAGE_END_ADDRESS - 4)
#define CRC_ADDRESS                   (APP_IMAGE_END_ADDRESS - 8)
#define APP_IMAGE_NUM_BLOCKS            ((APP_IMAGE_END_ADDRESS - APP_IMAGE_START_ADDRESS) / FLASH_BLOCK_SIZE)

/* Magic value written to OTA_FLAG_END_ADDRESS by App to trigger firmware update */
#define ENTER_UPDATE_MODE           0x5555AAAA

//#define SECONDARY_IMAGE_NUM_BLOCKS      ((SECONDARY_IMAGE_END_ADDRESS - SECONDARY_IMAGE_START_ADDRESS) / FLASH_BLOCK_SIZE)

typedef enum e_enable_disable
{
    DISABLE,
    RE_ENABLE
}enable_disable_t;

void ThreadsAndInterrupts(enable_disable_t EnableDisable);
void display_image_slot_info(void);

#endif /* HEADER_H_ */
