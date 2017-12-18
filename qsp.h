#include "Arduino.h"
#include "crsfReceiver.h"

void qspDecodeRcDataFrame(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceSate);
void decodeRxHealthPayload(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceState);

uint8_t get10bitHighShift(uint8_t channel);
uint8_t get10bitLowShift(uint8_t channel);
void qspComputeCrc(QspConfiguration_t *qsp, uint8_t dataByte);
void encodeRxHealthPayload(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceState, RadioState_t *radioState);
void encodeRcDataPayload(QspConfiguration_t *qsp, CrsfState_t *crsfState, uint8_t noOfChannels);
void qspDecodeIncomingFrame(
    QspConfiguration_t *qsp, 
    uint8_t incomingByte, 
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState,
    RadioState_t *radioState
);
void qspClearPayload(QspConfiguration_t *qsp);
void qspEncodeFrame(QspConfiguration_t *qsp, uint8_t buffer[], uint8_t *size);

void encodePingPayload(QspConfiguration_t *qsp, uint32_t currentMicros);