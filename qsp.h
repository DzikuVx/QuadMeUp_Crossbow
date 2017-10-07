#include "Arduino.h"
#include <PPMReader.h>

void qspDecodeRcDataFrame(QspConfiguration_t *qsp, int output[]);
uint8_t get10bitHighShift(uint8_t channel);
uint8_t get10bitLowShift(uint8_t channel);
void qspComputeCrc(QspConfiguration_t *qsp, uint8_t dataByte);
void encodeRcDataPayload(QspConfiguration_t *qsp, PPMReader *ppmSource, uint8_t noOfChannels);
uint8_t qspGetPacketId(void);
void qspDecodeIncomingFrame(QspConfiguration_t *qsp, uint8_t incomingByte, int ppm[]);
void qspClearPayload(QspConfiguration_t *qsp);
void qspEncodeFrame(QspConfiguration_t *qsp);