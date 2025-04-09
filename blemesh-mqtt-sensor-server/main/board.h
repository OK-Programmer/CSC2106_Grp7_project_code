/* board.h - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BOARD_H_    /* Include guard to prevent multiple inclusions */
#define _BOARD_H_

#ifdef __cplusplus
extern "C" {         /* Make header compatible with C++ compilers */
#endif /**< __cplusplus */

#include "driver/gpio.h"  /* Include ESP32 GPIO driver definitions */

/* LED pin definitions based on the board configuration */
#if defined(CONFIG_BLE_MESH_ESP_WROOM_32)
#define LED_R GPIO_NUM_25  /* Red LED connected to GPIO 25 for ESP-WROOM-32 */
#define LED_G GPIO_NUM_26  /* Green LED connected to GPIO 26 for ESP-WROOM-32 */
#define LED_B GPIO_NUM_27  /* Blue LED connected to GPIO 27 for ESP-WROOM-32 */
#elif defined(CONFIG_BLE_MESH_ESP_WROVER)
#define LED_R GPIO_NUM_0   /* Red LED connected to GPIO 0 for ESP-WROVER */
#define LED_G GPIO_NUM_2   /* Green LED connected to GPIO 2 for ESP-WROVER */
#define LED_B GPIO_NUM_4   /* Blue LED connected to GPIO 4 for ESP-WROVER */
#elif defined(CONFIG_BLE_MESH_ESP32C3_DEV)
#define LED_R GPIO_NUM_8   /* All LEDs connected to GPIO 8 for ESP32-C3 */
#define LED_G GPIO_NUM_8
#define LED_B GPIO_NUM_8
#elif defined(CONFIG_BLE_MESH_ESP32S3_DEV)
#define LED_R GPIO_NUM_47  /* All LEDs connected to GPIO 47 for ESP32-S3 */
#define LED_G GPIO_NUM_47
#define LED_B GPIO_NUM_47
#elif defined(CONFIG_BLE_MESH_ESP32C6_DEV)
#define LED_R GPIO_NUM_8   /* All LEDs connected to GPIO 8 for ESP32-C6 */
#define LED_G GPIO_NUM_8
#define LED_B GPIO_NUM_8
#elif defined(CONFIG_BLE_MESH_ESP32C61_DEV)
#define LED_R GPIO_NUM_8   /* All LEDs connected to GPIO 8 for ESP32-C61 */
#define LED_G GPIO_NUM_8
#define LED_B GPIO_NUM_8
#elif defined(CONFIG_BLE_MESH_ESP32H2_DEV)
#define LED_R GPIO_NUM_8   /* All LEDs connected to GPIO 8 for ESP32-H2 */
#define LED_G GPIO_NUM_8
#define LED_B GPIO_NUM_8
#elif defined(CONFIG_BLE_MESH_ESP32C5_DEV)
#define LED_R GPIO_NUM_8   /* All LEDs connected to GPIO 8 for ESP32-C5 */
#define LED_G GPIO_NUM_8
#define LED_B GPIO_NUM_8
#endif

/* LED state definitions */
#define LED_ON  1    /* Value to turn on an LED */
#define LED_OFF 0    /* Value to turn off an LED */

/* Structure to track LED state */
struct _led_state {
    uint8_t current;   /* Current state of the LED (on/off) */
    uint8_t previous;  /* Previous state of the LED */
    uint8_t pin;       /* GPIO pin number to which the LED is connected */
    char *name;        /* Name or identifier for the LED */
};

/* Function to control the LED state */
void board_led_operation(uint8_t pin, uint8_t onoff);

/* Function to initialize the board components */
void board_init(void);

#ifdef __cplusplus
}           /* End of C compatibility code block */
#endif /**< __cplusplus */

#endif /* _BOARD_H_ */
