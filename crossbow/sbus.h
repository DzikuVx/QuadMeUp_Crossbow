
#ifndef SBUS_INPUT
#define SBUS_INPUT

#include "Arduino.h"
#include "tx_input.h"

enum sbusProtocolStates {
    SBUS_DECODING_STATE_IDLE,
    SBUS_DECODING_STATE_IN_PROGRESS
};

class SbusInput : public TxInput
{
  public:
  	SbusInput(HardwareSerial &serial);
    void start(void);
    void restart(void);
    void loop(void);
    bool isReceiving(void);
    void recoverStuckFrames(void);
  private:
  	HardwareSerial &_serial;
    uint32_t _frameDecodingStartedAt = 0;
    uint32_t _frameDecodingEndedAt = 0 ;
    uint8_t _protocolState = SBUS_DECODING_STATE_IDLE;
	void sbusRead(void);
};

void sbusPreparePacket(uint8_t packet[], int16_t channels[], bool isSignalLoss, bool isFailsafe);

#endif

