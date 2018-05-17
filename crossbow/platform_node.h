#pragma once

#include "Arduino.h"

#ifndef PLATFORM_NODE_H
#define PLATFORM_NODE_H

#define PLATFORM_TOTAL_CHANNEL_COUNT 11 //Including RSSI channel and other
#define PLATFORM_CHANNEL_COUNT 10
#define PLATFORM_DEFAULT_CHANNEL_VALUE 1000

class PlatformNode {

    public:
        PlatformNode(void);
        int getRcChannel(uint8_t channel);
        void setRcChannel(uint8_t channel, int value, int offset);
    private:
        volatile int _channels[PLATFORM_TOTAL_CHANNEL_COUNT];
};

#endif