#include <Arduino.h>
#include <IRremote.h>

// Choose a GPIO for IR receive (make sure it is valid for your ESP32-C3 board)
#define IR_RECEIVE_PIN 4


extern "C" void app_main(void); // We'll call our IR test from inside app_main or setup()

void setupIRTest() {
    Serial.begin(115200);
    Serial.println("Starting IR Receiver...");
    
    pinMode(IR_RECEIVE_PIN, INPUT_PULLUP);
    IrReceiver.begin(IR_RECEIVE_PIN, false);  
    Serial.println("IR Receiver Ready.");
}

void loopIRTest() {
    if (IrReceiver.decode()) {
        Serial.println("IR Data Received:");
        
        // Print raw timing data
        Serial.print("rawData[] = {");
        for (uint16_t i = 1; i < IrReceiver.decodedIRData.rawDataPtr->rawlen; i++) {
            Serial.print(IrReceiver.decodedIRData.rawDataPtr->rawbuf[i] * MICROS_PER_TICK);
            if (i < IrReceiver.decodedIRData.rawDataPtr->rawlen - 1) Serial.print(", ");
        }
        Serial.println("};");

        // Save formatted data for future transmission
        Serial.println("---- COPY THE ABOVE DATA ----");

        IrReceiver.resume(); // Prepare for the next reception
    }
}
