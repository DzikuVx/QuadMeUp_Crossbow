#pragma once

#include "Arduino.h"
#include "qsp.h"

#define RADIO_FREQUENCY_MIN 868000000
#define RADIO_FREQUENCY_MAX 870000000
#define RADIO_FREQUENCY_RANGE (RADIO_FREQUENCY_MAX-RADIO_FREQUENCY_MIN)
#define RADIO_CHANNEL_WIDTH 250000
#define RADIO_CHANNEL_COUNT 9 // 9 channels in 2MHz range (RADIO_FREQUENCY_RANGE/RADIO_CHANNEL_WIDTH) + 1
#define RADIO_HOP_OFFSET 5

#ifndef RADIO_NODE_H
#define RADIO_NODE_H

#include "variables.h"

class RadioNode {
    public:
        RadioNode(void);
        void init(uint8_t ss, uint8_t rst, uint8_t di0, void(*callback)(int));
        void readRssi(void);
        void readSnr(void);
        void hopFrequency(bool forward, uint8_t fromChannel, uint32_t timestamp);
        void readAndDecode(
            QspConfiguration_t *qsp,
            RxDeviceState_t *rxDeviceState,
            TxDeviceState_t *txDeviceState
        );
        uint8_t getChannel(void);
        uint32_t getChannelEntryMillis(void);
        void handleChannelDwell(void);
        void handleTxDoneState(bool hop);
        void handleTx(QspConfiguration_t *qsp);
        volatile int8_t bytesToRead = -1;
        volatile uint8_t radioState = RADIO_STATE_RX;
        uint8_t rssi = 0;
        uint8_t snr = 0;
        uint8_t lastReceivedChannel = 0;
        uint8_t failedDwellsCount = 0;
        uint32_t loraBandwidth = 250000;
        uint8_t loraSpreadingFactor = 7;
        uint8_t loraCodingRate = 6;
        uint8_t loraTxPower = 17; // Defines output power of TX, defined in dBm range from 2-17
        bool canTransmit = false;
    private:
        uint8_t _channel = 0;
        uint32_t _channelEntryMillis = 0;
        uint32_t nextTxCheckMillis = 0;
};

#endif