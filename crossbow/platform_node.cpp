#include "platform_node.h"

PlatformNode::PlatformNode(void) {
    for (uint8_t i = 0; i < PLATFORM_TOTAL_CHANNEL_COUNT; i++) {
        _channels[i] = PLATFORM_DEFAULT_CHANNEL_VALUE;
    }
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
    radioNode.reset();
}