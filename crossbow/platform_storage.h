#pragma once

#include "Arduino.h"

#if defined(ARDUINO_AVR_FEATHER32U4)
#include <EEPROM.h>
#elif defined(ARDUINO_ESP32_DEV)
#include "EEPROM.h"
#elif defined(ARDUINO_SAMD_FEATHER_M0)
// Include EEPROM-like API for FlashStorage
#include <FlashAsEEPROM.h>
#endif

#ifndef STORAGE_H
#define STORAGE_H

class PlatformStorage {

    public:
        PlatformStorage(void);
        void commit(void);
        void write(int address, uint8_t val);
        uint8_t read(int address);
};

#endif