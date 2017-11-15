#include "Arduino.h"
#include <PPMReader.h>

void qspDecodeRcDataFrame(QspConfiguration_t *qsp, int output[]);
void decodeRxHealthPayload(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceState);

uint8_t get10bitHighShift(uint8_t channel);
uint8_t get10bitLowShift(uint8_t channel);
void qspComputeCrc(QspConfiguration_t *qsp, uint8_t dataByte);
void encodeRxHealthPayload(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceState, RadioState_t *radioState);
void encodeRcDataPayload(QspConfiguration_t *qsp, PPMReader *ppmSource, uint8_t noOfChannels);
void qspDecodeIncomingFrame(
    QspConfiguration_t *qsp, 
    uint8_t incomingByte, 
    int ppm[], 
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState
);
void qspClearPayload(QspConfiguration_t *qsp);
void qspEncodeFrame(QspConfiguration_t *qsp);

void encodePingPayload(QspConfiguration_t *qsp, uint32_t currentMicros);