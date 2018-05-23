#pragma once

#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <EEPROM.h>
#include "platform_node.h"

extern PlatformNode platformNode;

enum platformConfigMemoryLayout {
    EEPROM_ADDRESS_BIND_KEY_SEEDED = 0x00,
    EEPROM_ADDRESS_BIND_0,
    EEPROM_ADDRESS_BIND_1,
    EEPROM_ADDRESS_BIND_2,
    EEPROM_ADDRESS_BIND_3,
    PLATFORM_CONFIG_LAST_BYTE
};

class PlatformConfig {
    public:
        PlatformConfig(void);
        void seed(void);
        void loadBindKey();
};

#endif