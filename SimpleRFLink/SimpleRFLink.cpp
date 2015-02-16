// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!


#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #if defined(__AVR__)
    #include <avr/io.h>
    #include <avr/interrupt.h>
  #endif
  #include "WProgram.h"
#endif

#include "SimpleRFLink.h"


#define SYNCH_BITS 10
#define LOWER_TIME_LIMIT 5
#define UPPER_TIME_LIMIT 10

#define S_RESET 0
#define S_CNT_ONE_LOW 1
#define S_CNT_ONE_HIGH 2
#define S_CNT_ZERO_LOW 3
#define S_CNT_ZERO_HIGH 4


volatile uint8_t RX_PIN;
volatile uint8_t BYTES_PER_FRAME;

RFReceiver::RFReceiver(uint8_t RXPin, uint8_t bytesPerFrame){
    BYTES_PER_FRAME = bytesPerFrame;
    RX_PIN = RXPin;
    pinMode(RX_PIN, INPUT);
}

/* Timer2 reload value, globally available */
volatile unsigned int _tcnt2;

void RFReceiver::setBaudRate(uint16_t baudRate){
    noInterrupts();
    TIMSK2 &= ~(1<<TOIE2);
    TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
    TCCR2B &= ~(1<<WGM22);
    ASSR &= ~(1<<AS2);
    TIMSK2 &= ~(1<<OCIE2A);
    TCCR2B = 0;
    TCCR2B |= (1<<CS21);

    /* We need to calculate a proper value to load the timer counter.
    * The following loads the value 131 into the Timer 2 counter register
    * The math behind this is:
    * (CPU frequency) / (prescaler value) = 2e6 Hz = .5 us.
    * 25 (desired period) / .5us = 50.
    * MAX(uint8) + 1 - 25 = 231;
    *
    * NOTE:
    * bittime|tcnt2
    * 400|156
    * 200|206
    * 100|231
    */
    /* Save value globally for later reload in ISR */
    switch (baudRate){
    case 5000:_tcnt2 = 231;break;
    case 2500:_tcnt2 = 206;break;
    case 1250:_tcnt2 = 156;break;
    }
    TCNT2 = _tcnt2;
    TIMSK2 |= (1<<TOIE2);
    interrupts();
}
  

volatile uint8_t _countState = 0;
volatile uint8_t _zeroCount = 0;
volatile uint8_t _oneCount = 0;
volatile boolean _priorOne = false;
volatile boolean _priorZero = false;
volatile uint8_t _synchCount = 0;
volatile boolean _synched = false;
volatile boolean _byteStarted = false;

/* The limit of bytes per frame is thus 8, although this is arbitrary */
volatile uint8_t _dataBuffer[8] = {0,0,0,0,0,0,0,0};
volatile uint8_t _detectedBits = 0;
volatile boolean _dataReady = false;

volatile boolean _sample = 0;

void insertBit(boolean bit){
    if (bit){
        /* Shift the byte to the right 1 and or in the left most 1, i.e. 2^7 = 128 */
        _dataBuffer[_detectedBits / 8] = _dataBuffer[_detectedBits / 8] >> 1 | 128;
    } else {
        _dataBuffer[_detectedBits / 8] = _dataBuffer[_detectedBits / 8] >> 1;
    }
    _detectedBits++;
}

//Set up the ISR
ISR(TIMER2_OVF_vect) {
    /* Reload the timer */
    TCNT2 = _tcnt2;

    //i.e. it is processing what we gave it
    //We wait until it is false to start detecting things.
    if (!_dataReady){
        //Sample the bit
        _sample = digitalRead(RX_PIN);

        switch (_countState){
        /*************** STATE RESET ******************/
        case S_RESET:
            _zeroCount = 0;
            _oneCount = 0;
            _priorOne = false;
            _priorZero = false;
            _synchCount = 0;
            _synched = false;
            _byteStarted = false;
            _detectedBits = 0;
            for (int i = 0; i < BYTES_PER_FRAME; i++){
                _dataBuffer[i] = 0;
            }

            if (_sample == 1){
                _countState = S_CNT_ONE_LOW;
                _oneCount++;
            } else {
                _countState = S_CNT_ZERO_LOW;
                _zeroCount++;
            }
            break;

        /*************** STATE COUNT ONES LOW ******************/
        case S_CNT_ONE_LOW:
            if (_sample){
                _oneCount++;
                if (_oneCount >= LOWER_TIME_LIMIT){
                    _countState = S_CNT_ONE_HIGH;
                }
            } else {
                _oneCount = 0;
                _priorZero = false;
                _zeroCount++;
                _countState = S_CNT_ZERO_LOW;
            }
            break;

        /*************** STATE COUNT ZEROS LOW ******************/
        case S_CNT_ZERO_LOW:
            if (!_sample){
                _zeroCount++;
                if (_zeroCount >= LOWER_TIME_LIMIT){
                    _countState = S_CNT_ZERO_HIGH;
                }
            } else {
                _zeroCount = 0;
                _priorOne = false;
                _oneCount += 1;
                _countState = S_CNT_ONE_LOW;
            }
            break;

        /*************** STATE COUNT ONES HIGH ******************/
        case S_CNT_ONE_HIGH:
            if (_sample){
                _oneCount += 1;
                if (_oneCount >= UPPER_TIME_LIMIT){
                    if (_priorZero){
                        //Got a zero!
                        if (!_byteStarted && _synched){
                            _byteStarted = true;
                        } else if (_byteStarted){
                            //We have detected a zero instead of a
                            //one at the end of byte so reset and ignore
                            //this data
                            if (_detectedBits == 8*BYTES_PER_FRAME){
                                _countState = S_RESET;
                            //Otherwise save the data into the byte
                            } else {
                                insertBit(false);
                            }
                        }
                        _oneCount = 1;
                        _priorZero = false;
                        _countState = S_CNT_ONE_LOW;
                    } else {
                        _countState = S_RESET;
                    }
                }
            } else {
                if (_priorZero){
                    //Got a zero!!!
                    if (not _byteStarted && _synched){
                        _byteStarted = true;
                    } else if (_byteStarted){
                        if (_detectedBits == 8*BYTES_PER_FRAME){
                            _countState = S_RESET;//stopped incorrectly
                        } else {
                            //Shift in a zero to the data byte
                            insertBit(false);

                        }
                    }
                    _oneCount = 1;
                    _zeroCount++;
                    _priorZero = false;
                    _countState = S_CNT_ZERO_LOW;
                } else {
                    _priorOne = true;
                    _oneCount = 1;
                    _zeroCount++;
                    _countState = S_CNT_ZERO_LOW;
                }
            }
            break;

        /*************** STATE COUNT ZEROES HIGH ******************/
        case S_CNT_ZERO_HIGH:
            if (!_sample){
                _zeroCount++;
                if (_zeroCount >= UPPER_TIME_LIMIT){
                    if (_priorOne){
                        //Got a one!
                        if (_byteStarted){
                            if (_detectedBits == 8*BYTES_PER_FRAME){
                                _countState = S_RESET;
                                _dataReady = true;
                            } else {
                                //Shift in a 1 to the data byte
                                insertBit(true);
                            }
                        }
                        if (!_synched) _synchCount++;
                        if (_synchCount > SYNCH_BITS) _synched = true;

                        _zeroCount = 1; //???
                        _priorOne = false;
                        _countState = S_CNT_ZERO_LOW;
                    } else {
                        _countState = S_RESET;
                    }
                }
            } else {
                if (_priorOne){
                    //Got a one!!!!
                    if (_byteStarted){
                        if (_detectedBits == 8*BYTES_PER_FRAME){
                            _countState = S_RESET;
                            _dataReady = true;
                        } else {
                            insertBit(true);
                        }
                    }
                    if (!_synched) _synchCount++;
                    if (_synchCount > SYNCH_BITS) _synched = true;
                    _zeroCount = 1; //???
                    _oneCount++;
                    _priorOne = false;
                    _countState = S_CNT_ONE_LOW;
                } else {
                    _priorZero = true;
                    _zeroCount = 0;
                    _oneCount += 1;
                    _countState = S_CNT_ONE_LOW;
                }
            }
            break;
        }
    }
}


boolean RFReceiver::dataReady(void){
    return _dataReady;
}

void RFReceiver::readData(uint8_t * buffer){
    if (_dataReady){
        for (int i = 0; i < BYTES_PER_FRAME; i++){
            buffer[i] = _dataBuffer[i];
        }
    }
    _dataReady = false;
}


/*
 * RF Transmitter Class
 */

RFTransmitter::RFTransmitter(uint8_t TXPin, uint16_t baudRate, uint8_t bytesPerFrame){
    /* Stop the ISR from trying to detect data */
    _dataReady = true;
    _TXPin = TXPin;
    _bytesPerFrame = bytesPerFrame;
    switch (baudRate){
    case 5000: _bitTime = 100;break;
    case 2500: _bitTime = 200;break;
    case 1250: _bitTime = 400;break;
    }
    pinMode(_TXPin, OUTPUT);
}

void RFTransmitter::sendBit(boolean bit){
    if (bit){
        //--\__ = 1
        digitalWrite(_TXPin, HIGH);
        delayMicroseconds(_bitTime);
        digitalWrite(_TXPin, LOW);
        delayMicroseconds(_bitTime);
    } else {
        //__/-- = 0
        digitalWrite(_TXPin, LOW);
        delayMicroseconds(_bitTime);
        digitalWrite(_TXPin, HIGH);
        delayMicroseconds(_bitTime);
    }
}


void RFTransmitter::sendByte(uint8_t byte){
    //Send the byte
    for (int n = 0; n < 8; n++){
        sendBit((boolean) byte & (1 << n));
    }
    //And parity bit
    //sendBit(byte % 2);
}

void RFTransmitter::sendFrame(uint8_t * buffer){
    //SYNCH 1s
    for (int i = 0; i < 16; i++){
        sendBit(1);
    }
    //START 0
    sendBit(0);
    //DATA
    for (int i = 0; i < _bytesPerFrame; i++){
        sendByte(buffer[i]);
    }
    //STOP 1
    sendBit(1);
}
