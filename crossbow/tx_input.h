
#ifndef TX_INPUT_H
#define TX_INPUT_H

#include "Arduino.h"

#define TX_INPUT_CHANNEL_COUNT 16

class TxInput
{
  public:
  	virtual ~TxInput() {}
    int get(uint8_t channel) { return channels[channel]; };
    virtual void start(void) {};
    virtual void stop(void) {};
    virtual bool isReceiving(void) { return false; };
    virtual void loop(void) {};
    volatile static int16_t channels[TX_INPUT_CHANNEL_COUNT];
};

#endif
