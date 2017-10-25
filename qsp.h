#include "Arduino.h"
#include "crsfReceiver.h"

void qspDecodeRcDataFrame(QspConfiguration_t *qsp, int output[]);
void decodeRxHealthPayload(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceState);

uint8_t get10bitHighShift(uint8_t channel);
uint8_t get10bitLowShift(uint8_t channel);
void qspComputeCrc(QspConfiguration_t *qsp, uint8_t dataByte);
void encodeRxHealthPayload(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceState);
void encodeRcDataPayload(QspConfiguration_t *qsp, CrsfState_t *crsfState, uint8_t noOfChannels);
uint8_t qspGetPacketId(void);
void qspDecodeIncomingFrame(QspConfiguration_t *qsp, uint8_t incomingByte, int ppm[], RxDeviceState_t *rxDeviceState);
void qspClearPayload(QspConfiguration_t *qsp);
void qspEncodeFrame(QspConfiguration_t *qsp);

void encodePingPayload(QspConfiguration_t *qsp, uint32_t currentMicros);