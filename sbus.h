#include "Arduino.h"

void sbusPreparePacket(uint8_t packet[], int16_t channels[], bool isSignalLoss, bool isFailsafe);
void sbusRead(HardwareSerial &_serial, SbusInput_t *sbusInput);
bool isReceivingSbus(SbusInput_t *sbusInput);