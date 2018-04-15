/*
	Copyright 2015 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "crc.h"
#include "buffer.h"

// 0 means generic, no LEDs
#define HW_VER					0

/*
 * Defines
 */
#define FLASH_SECTORS			12
#define BOOTLOADER_BASE			11
#define APP_BASE				0
#define APP_SECTORS				7
#define NEW_APP_BASE			8
#define NEW_APP_SECTORS			3
#define NEW_APP_MAX_SIZE		(3 * (1 << 17))

// Base address of the Flash sectors
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) // Base @ of Sector 0, 16 Kbytes
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) // Base @ of Sector 1, 16 Kbytes
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) // Base @ of Sector 2, 16 Kbytes
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) // Base @ of Sector 3, 16 Kbytes
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) // Base @ of Sector 4, 64 Kbytes
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) // Base @ of Sector 5, 128 Kbytes
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) // Base @ of Sector 6, 128 Kbytes
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) // Base @ of Sector 7, 128 Kbytes
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) // Base @ of Sector 8, 128 Kbytes
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) // Base @ of Sector 9, 128 Kbytes
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) // Base @ of Sector 10, 128 Kbytes
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) // Base @ of Sector 11, 128 Kbytes

// LEDs
#if HW_VER == 60
#define LED_GREEN_GPIO			GPIOB
#define LED_GREEN_PIN			0
#define LED_RED_GPIO			GPIOB
#define LED_RED_PIN				1
#else
#define LED_GREEN_GPIO			GPIOC
#define LED_GREEN_PIN			4
#define LED_RED_GPIO			GPIOA
#define LED_RED_PIN				7
#endif

#define LED_GREEN				0
#define LED_RED					1

#if HW_VER == 0
#define LED_GREEN_ON()
#define LED_GREEN_OFF()
#define LED_RED_ON()
#define LED_RED_OFF()
#else
#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)
#endif

// Private constants
static const uint32_t flash_addr[FLASH_SECTORS] = {
		ADDR_FLASH_SECTOR_0,
		ADDR_FLASH_SECTOR_1,
		ADDR_FLASH_SECTOR_2,
		ADDR_FLASH_SECTOR_3,
		ADDR_FLASH_SECTOR_4,
		ADDR_FLASH_SECTOR_5,
		ADDR_FLASH_SECTOR_6,
		ADDR_FLASH_SECTOR_7,
		ADDR_FLASH_SECTOR_8,
		ADDR_FLASH_SECTOR_9,
		ADDR_FLASH_SECTOR_10,
		ADDR_FLASH_SECTOR_11
};
static const uint16_t flash_sector[12] = {
		FLASH_Sector_0,
		FLASH_Sector_1,
		FLASH_Sector_2,
		FLASH_Sector_3,
		FLASH_Sector_4,
		FLASH_Sector_5,
		FLASH_Sector_6,
		FLASH_Sector_7,
		FLASH_Sector_8,
		FLASH_Sector_9,
		FLASH_Sector_10,
		FLASH_Sector_11
};

// Private functions
static void exit_bootloader(void);
static int16_t erase_app(uint32_t new_app_size);
static int16_t write_app_data(uint32_t offset, uint8_t *data, uint32_t len);
void blink_led(int led, int blinks);
void sleep(int time);

// Private variables
static uint8_t *new_app_addr;

static void exit_bootloader(void) {
	// Use the WWDG to reset the MCU
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	WWDG_SetPrescaler(WWDG_Prescaler_1);
	WWDG_SetWindowValue(255);
	WWDG_Enable(100);

	__disable_irq();
	for(;;){};
}

static int16_t erase_app(uint32_t new_app_size) {
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	new_app_size += flash_addr[APP_BASE];

	for (int i = 0;i < APP_SECTORS;i++) {
		if (new_app_size > flash_addr[APP_BASE + i]) {
			int16_t res = FLASH_EraseSector(flash_sector[APP_BASE + i], VoltageRange_3);
			if (res != FLASH_COMPLETE) {
				return res;
			}
		} else {
			break;
		}
	}

	return FLASH_COMPLETE;
}

static int16_t write_app_data(uint32_t offset, uint8_t *data, uint32_t len) {
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	for (uint32_t i = 0;i < len;i++) {
		int16_t res = FLASH_ProgramByte(flash_addr[APP_BASE] + offset + i, data[i]);
		if (res != FLASH_COMPLETE) {
			return res;
		}
	}

	return FLASH_COMPLETE;
}

static int16_t write_new_app(void) {
	int32_t ind = 0;
	uint32_t size = buffer_get_uint32(new_app_addr, &ind);
	uint16_t crc_app = buffer_get_uint16(new_app_addr, &ind);

	if (size > NEW_APP_MAX_SIZE) {
		return -1;
	}

	if (size == 0) {
		return -2;
	}

	uint16_t crc_calc = crc16(new_app_addr + ind, size);
	if (crc_calc != crc_app) {
		return -3;
	}

	int16_t res = erase_app(size);
	if (res != FLASH_COMPLETE) {
		return res;
	}

	return write_app_data(0, new_app_addr + ind, size);
}

void blink_led(int led, int blinks) {
#if HW_VER == 0
	(void)led;
	(void)blinks;
#else
	if (led == LED_GREEN) {
		for (int i = 0;i < blinks;i++) {
			LED_GREEN_ON();
			sleep(200);
			LED_GREEN_OFF();
			sleep(200);
		}
	} else if (led == LED_RED) {
		for (int i = 0;i < blinks;i++) {
			LED_RED_ON();
			sleep(200);
			LED_RED_OFF();
			sleep(200);
		}
	}
#endif
}

void sleep(int time) {
	for (volatile int i = 0;i < time * 10000;i++) {
		__NOP();
	}

//	chThdSleepMilliseconds(time);
}

int main(void) {
//	halInit();
//	chSysInit();

	// Change the vector table offset
	SCB_VTOR = flash_addr[BOOTLOADER_BASE];

	// Set app base address
	new_app_addr = (uint8_t *)flash_addr[NEW_APP_BASE];

	// LEDs
#if HW_VER != 0
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
#endif

	LED_GREEN_OFF();
	LED_RED_OFF();

	// Up to 5 retries
	for (int i = 0;i < 5;i++) {
		int16_t res = write_new_app();

		if (res == -1) {
			blink_led(LED_RED, 3);
			exit_bootloader();
		} else if (res == -2) {
			blink_led(LED_RED, 4);
			exit_bootloader();
		} else if (res == -3) {
			blink_led(LED_RED, 5);
			exit_bootloader();
		} else if (res == FLASH_COMPLETE) {
			blink_led(LED_GREEN, 3);
			exit_bootloader();
		}

		blink_led(LED_RED, 1);
	}

	// Erasing or writing to flash failed. A programmer is needed
	// to upload the firmware now.
	sleep(2000);
	blink_led(LED_RED, 8);
	exit_bootloader();

	// Should not happen
	for(;;) {
		sleep(1000);
	}
}
