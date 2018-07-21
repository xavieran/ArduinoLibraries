/*
 * Author: Emmanuel Jacyna
 * Simple example to demonstrate how to use SimpleRFLink to receive data
 */

#include "SimpleRFLink.h"
#include "Arduino.h"

#define BYTES_PER_FRAME 2
#define RF_BAUD 2500
#define RX_PIN A0

#define LED_PIN A1

//Declare a new RFReceiver object, passing in the pin to receive on
//and the number of bytes to receive per frame
RFReceiver RX(RX_PIN, BYTES_PER_FRAME);

void setup()
{
    //Sets the baud rate of the rf link. 2500 is usually pretty good
    RX.setBaudRate(RF_BAUD);
    Serial.begin(9600); // Debugging only
}

//Declare a buffer to store the bytes in
uint8_t buffer[BYTES_PER_FRAME];

void loop(){
    if (RX.dataReady()) // Non-blocking
    {
        //Read the data into your buffer
        RX.readData(buffer);

        //Do what you want with the data
        Serial.print("Received Bytes:");
        for (int i = 0; i < BYTES_PER_FRAME; i++){
            Serial.print(buffer[i], DEC);
            Serial.print("-");
        }

        Serial.print("\n");
        //Flash the LED to provide visual indication of receiving data
        digitalWrite(LED_PIN, HIGH);
        delay(1);
        digitalWrite(LED_PIN, LOW);
        delay(1);
    }
}
 
