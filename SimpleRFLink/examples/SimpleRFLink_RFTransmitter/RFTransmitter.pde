/*
 * Author: Emmanuel Jacyna
 * Simple example of how to use SimpleRFLink to transmit data
 */

#include "SimpleRFLink.h"
#include "Arduino.h"


#define SERIAL_BAUD 9600
#define RF_BAUD 2500
#define BYTES_PER_FRAME 2
#define TX_PIN A0 

//Declare a new RFTransmitter, passing in the transmit pin, 
//the baud rate and the bytes per frame
RFTransmitter TX(TX_PIN, (unsigned int) RF_BAUD, BYTES_PER_FRAME);

 void setup(){
     Serial.begin(SERIAL_BAUD);
 }


 /*Declare a buffer to send data through
  *Note that it MUST be at least as large as the number of 
  *bytes per frame or it will crash
  */
 byte buffer[BYTES_PER_FRAME] = {0, 100};
 void loop(){

    //Increment so each transmission is different
    buffer[0]++;
    buffer[1]++;
    //Send the buffer
    TX.sendFrame(buffer);
    
    /*It is important to add a slight delay, or the data seems
     *to get jumbled up. Experiment with this to find a value
     *appropriate for your setup.
     */
    delay(20);
 }
