// RF Link Library

/*
 *Author: Emmanuel Jacyna
 * Simple library to implement RF transmission with ASK modules
 */

#ifndef _SimpleRFLink_h_
#define _SimpleRFLink_h_
#endif

#include "Arduino.h"

#include <inttypes.h>
#if defined(__AVR__)
    #include <avr/io.h>
    #include <avr/interrupt.h>
#endif

class RFReceiver
{
  public:
    RFReceiver(uint8_t RXPin, uint8_t bytesPerFrame);
    void setBaudRate(uint16_t baudRate);
    boolean dataReady(void);
    void readData(uint8_t * buffer);
};

class RFTransmitter
{
 public:
  RFTransmitter(uint8_t TXPin, uint16_t baudRate, uint8_t bytesPerFrame);
  void sendFrame(uint8_t * buffer);
  
 private:
  uint8_t _TXPin;
  uint8_t _bitTime;
  uint8_t _bytesPerFrame;
  void sendByte(uint8_t byte);
  void sendBit(boolean bit);
};
