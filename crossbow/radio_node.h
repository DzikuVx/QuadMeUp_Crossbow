#pragma once

#include "Arduino.h"

#ifndef RADIO_NODE_H
#define RADIO_NODE_H

#define RADIO_STATE_TX 1
#define RADIO_STATE_RX 2

#define TX_TRANSMIT_SLOT_RATE 67 //ms
#define RX_CHANNEL_DWELL_TIME (TX_TRANSMIT_SLOT_RATE + 10) //Dwell on a channel slightly longer 
#define RX_FAILSAFE_DELAY (TX_TRANSMIT_SLOT_RATE * 8)
#define TX_FAILSAFE_DELAY (RX_FAILSAFE_DELAY * 4)

#define RADIO_FREQUENCY_MIN 868000000
#define RADIO_FREQUENCY_MAX 870000000
#define RADIO_FREQUENCY_RANGE (RADIO_FREQUENCY_MAX-RADIO_FREQUENCY_MIN)
#define RADIO_CHANNEL_WIDTH 250000
#define RADIO_CHANNEL_COUNT 9 // 9 channels in 2MHz range (RADIO_FREQUENCY_RANGE/RADIO_CHANNEL_WIDTH) + 1
#define RADIO_HOP_OFFSET 5

struct RadioState_t {
    uint32_t loraBandwidth = 250000;
    uint8_t loraSpreadingFactor = 7;
    uint8_t loraCodingRate = 6;
    uint8_t loraTxPower = 17; // Defines output power of TX, defined in dBm range from 2-17
    int8_t bytesToRead = -1;
    uint8_t rssi = 0;
    uint8_t snr = 0;
    uint8_t deviceState = RADIO_STATE_RX;
    uint32_t nextTxCheckMillis = 0;

    const uint32_t dwellTime = TX_TRANSMIT_SLOT_RATE * 2; 
    uint8_t lastReceivedChannel = 0;
    uint8_t failedDwellsCount = 0;
};

class RadioNode {
    public:
        RadioNode(void);
        static uint8_t getRadioRssi(void);
        static uint8_t getRadioSnr(void);
        static uint32_t getFrequencyForChannel(uint8_t channel);
        static uint8_t getNextChannel(uint8_t channel);
        static uint8_t getPrevChannel(uint8_t channel);
        void hopFrequency(bool forward, uint8_t fromChannel, uint32_t timestamp);
        uint8_t getChannel(void);
        uint32_t getChannelEntryMillis(void);
    private:
        uint8_t _channel = 0;
        uint32_t _channelEntryMillis = 0;
};

#endif