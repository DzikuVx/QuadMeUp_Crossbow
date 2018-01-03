#include "Arduino.h"

void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe);
void sbusRead(HardwareSerial &_serial, SbusInput_t *sbusInput);
bool isReceivingSbus(SbusInput_t *sbusInput);