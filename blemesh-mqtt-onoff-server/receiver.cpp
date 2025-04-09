/**
 * IR Receiver Utility
 * 
 * This code implements an IR signal receiver for ESP32-C3 using the IRremote library.
 * It captures incoming IR signals and outputs raw timing data to be used for future
 * IR signal transmission.
 */

#include <Arduino.h>
#include <IRremote.h>  // Library for IR signal processing

// Define the GPIO pin connected to the IR receiver module
// Make sure to choose a pin compatible with your ESP32-C3 board
#define IR_RECEIVE_PIN 4

// Forward declaration of app_main which is called by the ESP-IDF framework
extern "C" void app_main(void); // We'll call our IR test from inside app_main or setup()

/**
 * Initialize the IR receiver
 * Sets up serial communication and configures the IR receiver module
 */
void setupIRTest() {
    // Initialize serial communication at 115200 baud rate
    Serial.begin(115200);
    Serial.println("Starting IR Receiver...");
    
    // Configure IR pin with pull-up resistor to ensure stable readings
    pinMode(IR_RECEIVE_PIN, INPUT_PULLUP);
    // Initialize the IR receiver (false = no LED feedback)
    IrReceiver.begin(IR_RECEIVE_PIN, false);  
    Serial.println("IR Receiver Ready.");
}

/**
 * Main IR reception processing loop
 * Checks for new IR signals and outputs the raw timing data
 * This function should be called repeatedly in the main program loop
 */
void loopIRTest() {
    // Check if IR data has been received
    if (IrReceiver.decode()) {
        Serial.println("IR Data Received:");
        
        // Print raw timing data in a format that can be used for transmission
        Serial.print("rawData[] = {");
        for (uint16_t i = 1; i < IrReceiver.decodedIRData.rawDataPtr->rawlen; i++) {
            // Convert ticks to microseconds for readable output
            Serial.print(IrReceiver.decodedIRData.rawDataPtr->rawbuf[i] * MICROS_PER_TICK);
            // Add commas between values for array format
            if (i < IrReceiver.decodedIRData.rawDataPtr->rawlen - 1) Serial.print(", ");
        }
        Serial.println("};");

        // User instruction for data extraction
        Serial.println("---- COPY THE ABOVE DATA ----");

        // Reset the receiver for the next IR signal
        IrReceiver.resume();
    }
}
