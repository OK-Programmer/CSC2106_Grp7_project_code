/**
 * @file temperature.cpp
 * @brief Temperature and humidity monitoring using DHT22 sensor
 */

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"  // Include the GPIO driver for gpio_config()
#include <DHT.h>          // DHT sensor library for reading temperature and humidity

// Define hardware configuration
#define DHTPIN 4          // GPIO pin where the DHT22 sensor is connected
#define DHTTYPE DHT22     // Sensor type (DHT22/AM2302/AM2321)

// Instantiate DHT sensor with specified pin and type
DHT dht(DHTPIN, DHTTYPE);

/**
 * @brief Initialize the DHT temperature sensor
 * 
 * Sets up the DHT sensor and prints initialization message
 */
void tempsetup() {
    //static bool initialized = false;  // Prevent multiple initializations

    //if (!initialized) {
        printf("\nDHT22 Temperature Sensor Initialized.\n");

        // Initialize the DHT sensor
        dht.begin();

        //initialized = true;
    //}
}

/**
 * @brief Main loop for temperature monitoring
 * 
 * Continuously reads temperature and humidity values from the DHT sensor
 * and calculates heat index. Runs every 2 seconds.
 */
void temploop() {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));  // Wait 2 seconds between readings
        
        // Read sensor values
        float humidity = dht.readHumidity();                // Relative humidity (%)
        float temperature = dht.readTemperature();          // Temperature in Celsius
        float fahrenheit = dht.readTemperature(true);       // Temperature in Fahrenheit

        // Error handling - verify sensor readings
        if (isnan(humidity) || isnan(temperature) || isnan(fahrenheit)) {
            printf("Failed to read from DHT sensor! (humidity: %.2f, temp: %.2f, fahrenheit: %.2f)\n", 
                   humidity, temperature, fahrenheit);
            continue;  // Skip to the next iteration if readings failed
        }

        // Calculate heat index (perceived temperature)
        float heatIndexF = dht.computeHeatIndex(fahrenheit, humidity);  // Heat index in Fahrenheit
        float heatIndexC = dht.computeHeatIndex(temperature, humidity, false);  // Heat index in Celsius

        // Print all measured and calculated values to console
        printf("Humidity: %.2f%%  Temperature: %.2f°C %.2f°F  Heat index: %.2f°C %.2f°F\n",
               humidity, temperature, fahrenheit, heatIndexC, heatIndexF);
    }
}
