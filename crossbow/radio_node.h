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
        static uint8_t getRadioRssi(void);
        static uint8_t getRadioSnr(void);
        static uint32_t getFrequencyForChannel(uint8_t channel);
        static uint8_t getNextChannel(uint8_t channel);
        static uint8_t getPrevChannel(uint8_t channel);
        void hopFrequency(bool forward, uint8_t fromChannel, uint32_t timestamp);
        void readAndDecode(
            volatile RadioState_t *radioState,
            QspConfiguration_t *qsp,
            RxDeviceState_t *rxDeviceState,
            TxDeviceState_t *txDeviceState
        );
        uint8_t getChannel(void);
        uint32_t getChannelEntryMillis(void);
        int8_t bytesToRead = -1;
        uint8_t deviceState = RADIO_STATE_RX;
    private:
        uint8_t _channel = 0;
        uint32_t _channelEntryMillis = 0;
};

#endif