#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"  // Include the GPIO driver for gpio_config()
#include <DHT.h>  // Ensure this is included

#define DHTPIN 4   // ✅ Use `gpio_num_t` type instead of `int`
#define DHTTYPE DHT22       // DHT 22 (AM2302), AM2321

DHT dht(DHTPIN, DHTTYPE);

void tempsetup() {
    //static bool initialized = false;  // Prevent multiple initializations

 //   if (!initialized) {
        printf("\nDHT22 Temperature Sensor Initialized.\n");


        // Initialize the DHT sensor
        dht.begin();

      //  initialized = true;
  //  }
}


void temploop() {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000)); 
        // Read the humidity and temperature
        float humidity = dht.readHumidity();
        float temperature = dht.readTemperature();
        float fahrenheit = dht.readTemperature(true);  // Fahrenheit temperature

        // Check if any reads failed and retry
        if (isnan(humidity) || isnan(temperature) || isnan(fahrenheit)) {
            printf("Failed to read from DHT sensor! (humidity: %.2f, temp: %.2f, fahrenheit: %.2f)\n", 
                   humidity, temperature, fahrenheit);
            //vTaskDelay(pdMS_TO_TICKS(2000));  // Retry after delay
            continue;
        }

        // Compute heat index
        float heatIndexF = dht.computeHeatIndex(fahrenheit, humidity);  // Heat index in Fahrenheit
        float heatIndexC = dht.computeHeatIndex(temperature, humidity, false);  // Heat index in Celsius

        // Print values
        printf("Humidity: %.2f%%  Temperature: %.2f°C %.2f°F  Heat index: %.2f°C %.2f°F\n",
               humidity, temperature, fahrenheit, heatIndexC, heatIndexF);

     //   vTaskDelay(pdMS_TO_TICKS(2000));  // Wait for the next reading (2 seconds)
    }
}
    



