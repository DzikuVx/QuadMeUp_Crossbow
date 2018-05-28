#pragma once

#include "Arduino.h"
#include "radio_node.h"
#include <EEPROM.h>

#ifndef PLATFORM_NODE_H
#define PLATFORM_NODE_H

#define PLATFORM_TOTAL_CHANNEL_COUNT 11 //Including RSSI channel and other
#define PLATFORM_CHANNEL_COUNT 10
#define PLATFORM_DEFAULT_CHANNEL_VALUE 1000

extern RadioNode radioNode;

enum deviceStates {
    DEVICE_STATE_OK,
    DEVICE_STATE_FAILSAFE,
    DEVICE_STATE_UNDETERMINED
};
 
enum platformConfigMemoryLayout { 
    EEPROM_ADDRESS_BIND_KEY_SEEDED = 0x00, 
    EEPROM_ADDRESS_BIND_0, 
    EEPROM_ADDRESS_BIND_1, 
    EEPROM_ADDRESS_BIND_2, 
    EEPROM_ADDRESS_BIND_3,
    PLATFORM_CONFIG_LAST_BYTE 
}; 

class PlatformNode {

    public:
        PlatformNode(void);
        int getRcChannel(uint8_t channel);
        void setRcChannel(uint8_t channel, int value, int offset);
        void enterBindMode(void);
        void leaveBindMode(void);
        void seed(void);
        void loadBindKey();
        uint8_t bindKey[4];
        uint32_t nextLedUpdate = 0;
        uint8_t platformState = DEVICE_STATE_UNDETERMINED;
        bool isBindMode = false;
        uint32_t bindModeExitMillis;
    private:
        volatile int _channels[PLATFORM_TOTAL_CHANNEL_COUNT];
};

#endif