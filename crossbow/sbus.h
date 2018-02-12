
#ifndef SBUS_INPUT
#define SBUS_INPUT

#include "Arduino.h"
#include "tx_input.h"


class SbusInput : public TxInput
{
  public:
  	SbusInput(HardwareSerial &serial);
    void start(void);
    void loop(void);
    bool isReceiving(void);
  private:
  	HardwareSerial &_serial;
    uint32_t _lastChannelReceivedAt = 0;
	void sbusRead(void);
};

void sbusPreparePacket(uint8_t packet[], int16_t channels[], bool isSignalLoss, bool isFailsafe);

#endif

