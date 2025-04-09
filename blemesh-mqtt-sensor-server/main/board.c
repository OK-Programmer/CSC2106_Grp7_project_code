/* board.c - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"

#define TAG "BOARD"  // Tag for ESP log messages

/**
 * LED state structure array to manage the three LEDs (R,G,B)
 * Each entry contains:
 * - current: Current state of the LED (on/off)
 * - previous: Previous state of the LED (to detect changes)
 * - pin: GPIO pin number for the LED
 * - name: String name of the LED color for logging
 */
struct _led_state led_state[3] = {
    { LED_OFF, LED_OFF, LED_R, "red"   },
    { LED_OFF, LED_OFF, LED_G, "green" },
    { LED_OFF, LED_OFF, LED_B, "blue"  },
};

/**
 * Sets the state of a specific LED
 *
 * @param pin   GPIO pin number of the LED to control
 * @param onoff Desired state (LED_ON or LED_OFF)
 */
void board_led_operation(uint8_t pin, uint8_t onoff)
{
    // Search for the LED in our state array
    for (int i = 0; i < 3; i++) {
        if (led_state[i].pin != pin) {
            continue;
        }
        // Check if LED is already in the requested state
        if (onoff == led_state[i].previous) {
            ESP_LOGW(TAG, "led %s is already %s",
                     led_state[i].name, (onoff ? "on" : "off"));
            return;
        }
        // Set the LED to the new state
        gpio_set_level(pin, onoff);
        led_state[i].previous = onoff;
        return;
    }

    // If we reach here, the pin doesn't match any known LED
    ESP_LOGE(TAG, "LED is not found!");
}

/**
 * Initialize all LEDs by configuring GPIO pins as outputs
 * and setting initial state to OFF
 */
static void board_led_init(void)
{
    for (int i = 0; i < 3; i++) {
        // Reset GPIO pin to default state
        gpio_reset_pin(led_state[i].pin);
        // Set pin as output
        gpio_set_direction(led_state[i].pin, GPIO_MODE_OUTPUT);
        // Turn LED off initially
        gpio_set_level(led_state[i].pin, LED_OFF);
        // Update state to match hardware
        led_state[i].previous = LED_OFF;
    }
}

/**
 * Main board initialization function
 * Currently only initializes LEDs but could be expanded
 * to initialize other board components
 */
void board_init(void)
{
    board_led_init();
}
