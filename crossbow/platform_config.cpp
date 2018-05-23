#include "platform_config.h"
#include "Arduino.h"

PlatformConfig::PlatformConfig(void) {
    randomSeed(analogRead(0));
};

void PlatformConfig::seed(void) {
    uint8_t val;

    val = EEPROM.read(EEPROM_ADDRESS_BIND_KEY_SEEDED);

    if (val != 0xf1) {
        EEPROM.write(EEPROM_ADDRESS_BIND_0, random(1, 255)); //Yes, from 1 to 254
        EEPROM.write(EEPROM_ADDRESS_BIND_1, random(1, 255)); //Yes, from 1 to 254
        EEPROM.write(EEPROM_ADDRESS_BIND_2, random(1, 255)); //Yes, from 1 to 254
        EEPROM.write(EEPROM_ADDRESS_BIND_3, random(1, 255)); //Yes, from 1 to 254
        EEPROM.write(EEPROM_ADDRESS_BIND_KEY_SEEDED, 0xf1);
    }
}

void PlatformConfig::loadBindKey(void) {
    platformNode.bindKey[0] = EEPROM.read(EEPROM_ADDRESS_BIND_0);
    platformNode.bindKey[1] = EEPROM.read(EEPROM_ADDRESS_BIND_1);
    platformNode.bindKey[2] = EEPROM.read(EEPROM_ADDRESS_BIND_2);
    platformNode.bindKey[3] = EEPROM.read(EEPROM_ADDRESS_BIND_3);
}