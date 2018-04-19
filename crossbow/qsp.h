#include "Arduino.h"

void qspDecodeRcDataFrame(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceSate);
void decodeRxHealthPayload(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceState);

uint8_t get10bitHighShift(uint8_t channel);
uint8_t get10bitLowShift(uint8_t channel);
void qspComputeCrc(QspConfiguration_t *qsp, uint8_t dataByte);
void encodeRxHealthPayload(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceState, volatile RadioState_t *radioState);
void encodeRcDataPayload(QspConfiguration_t *qsp, volatile int16_t channels[], uint8_t noOfChannels);
void qspDecodeIncomingFrame(
    QspConfiguration_t *qsp, 
    uint8_t incomingByte, 
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState,
    volatile RadioState_t *radioState
);
void qspClearPayload(QspConfiguration_t *qsp);
void qspEncodeFrame(QspConfiguration_t *qsp, volatile RadioState_t *radioState,  uint8_t buffer[], uint8_t *size);

void encodePingPayload(QspConfiguration_t *qsp, uint32_t currentMicros);