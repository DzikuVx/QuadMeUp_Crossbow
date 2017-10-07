#include "Arduino.h"
#include <PPMReader.h>

void qspDecodeRcDataFrame(uint8_t payload[], int output[]);
uint8_t get10bitHighShift(uint8_t channel);
uint8_t get10bitLowShift(uint8_t channel);
void qspComputeCrc(uint8_t *crc, uint8_t dataByte);
void encodeRcDataPayload(PPMReader *ppmSource, uint8_t noOfChannels, uint8_t payload[], uint8_t *payloadLength);