#include "platform_node.h"

PlatformNode::PlatformNode(void) {
    for (uint8_t i = 0; i < PLATFORM_TOTAL_CHANNEL_COUNT; i++) {
        _channels[i] = PLATFORM_DEFAULT_CHANNEL_VALUE;
    }

    randomSeed(analogRead(0));
}

void PlatformNode::seed(void) {
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

void PlatformNode::loadBindKey(void) {
    bindKey[0] = EEPROM.read(EEPROM_ADDRESS_BIND_0);
    bindKey[1] = EEPROM.read(EEPROM_ADDRESS_BIND_1);
    bindKey[2] = EEPROM.read(EEPROM_ADDRESS_BIND_2);
    bindKey[3] = EEPROM.read(EEPROM_ADDRESS_BIND_3);
}

int PlatformNode::getRcChannel(uint8_t channel) {
    if (channel < PLATFORM_TOTAL_CHANNEL_COUNT) {
        return _channels[channel];
    } else {
        return PLATFORM_DEFAULT_CHANNEL_VALUE;
    }
}

void PlatformNode::setRcChannel(uint8_t channel, int value, int offset) {
    if (channel < PLATFORM_TOTAL_CHANNEL_COUNT) {
        _channels[channel] = value + offset;
    }
}

void PlatformNode::enterBindMode(void) {
    isBindMode = true;

    // Set temporary bind mode
    bindKey[0] = 0xf1;
    bindKey[0] = 0x1e;
    bindKey[0] = 0x07;
    bindKey[0] = 0x42;

    radioNode.set(
        0, // Minimum power
        125000, // 125kHz bandwidth
        6, // low spreading factor, we do not need high RX sensitivity
        5, // same for coding rate
        868000000 //Fixed frequency while binding
    );
}

void PlatformNode::leaveBindMode(void) {
    isBindMode = false;
    loadBindKey();
    radioNode.reset();
}