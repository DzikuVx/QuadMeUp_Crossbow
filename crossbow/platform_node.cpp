#include "platform_node.h"

PlatformNode::PlatformNode(void) {
    for (uint8_t i = 0; i < PLATFORM_TOTAL_CHANNEL_COUNT; i++) {
        _channels[i] = PLATFORM_DEFAULT_CHANNEL_VALUE;
    }
}

/**
 * Return true if new bind key was generated
 */
void PlatformNode::seed(void) {
    uint8_t val = storage.read(EEPROM_ADDRESS_BIND_KEY_SEEDED);

    if (val != 0xf1) {
        storage.write(EEPROM_ADDRESS_BIND_0, random(1, 255)); //Yes, from 1 to 254
        storage.write(EEPROM_ADDRESS_BIND_1, random(1, 255)); //Yes, from 1 to 254
        storage.write(EEPROM_ADDRESS_BIND_2, random(1, 255)); //Yes, from 1 to 254
        storage.write(EEPROM_ADDRESS_BIND_3, random(1, 255)); //Yes, from 1 to 254
        storage.write(EEPROM_ADDRESS_BIND_KEY_SEEDED, 0xf1);
        storage.commit();
    } 
}

void PlatformNode::loadBindKey(uint8_t key[]) {
    key[0] = storage.read(EEPROM_ADDRESS_BIND_0);
    key[1] = storage.read(EEPROM_ADDRESS_BIND_1);
    key[2] = storage.read(EEPROM_ADDRESS_BIND_2);
    key[3] = storage.read(EEPROM_ADDRESS_BIND_3);
}

void PlatformNode::saveBindKey(uint8_t key[]) {
    storage.write(EEPROM_ADDRESS_BIND_0, key[0]);
    storage.write(EEPROM_ADDRESS_BIND_1, key[1]);
    storage.write(EEPROM_ADDRESS_BIND_2, key[2]);
    storage.write(EEPROM_ADDRESS_BIND_3, key[3]);
    storage.write(EEPROM_ADDRESS_BIND_KEY_SEEDED, 0xf1);
    storage.commit();
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
    bindKey[1] = 0x1e;
    bindKey[2] = 0x07;
    bindKey[3] = 0x42;

    radioNode.set(
        0, // Minimum power
        125000, // 125kHz bandwidth
        6, // low spreading factor, we do not need high RX sensitivity
        5, // same for coding rate
        868000000 //Fixed frequency while binding
    );
    bindModeExitMillis = millis() + 1000; //This happens only on RX
}

void PlatformNode::leaveBindMode(void) {
    isBindMode = false;
    loadBindKey(bindKey);
    radioNode.reset();
}
