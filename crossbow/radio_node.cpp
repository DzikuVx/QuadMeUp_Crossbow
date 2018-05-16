#include "radio_node.h"
#include "lora.h"

RadioNode::RadioNode(void) {

}

static uint8_t RadioNode::getRadioRssi(void)
{
    return 164 - constrain(LoRa.packetRssi() * -1, 0, 164);
}

static uint8_t RadioNode::getRadioSnr(void)
{
    return (uint8_t) constrain(LoRa.packetSnr(), 0, 255);
}

uint8_t RadioNode::getChannel(void) {
    return _channel;
}

uint32_t RadioNode::getChannelEntryMillis(void) {
    return _channelEntryMillis;
}

static uint32_t RadioNode::getFrequencyForChannel(uint8_t channel) {
    return RADIO_FREQUENCY_MIN + (RADIO_CHANNEL_WIDTH * channel);
}

static uint8_t RadioNode::getNextChannel(uint8_t channel) {
    return (channel + RADIO_HOP_OFFSET) % RADIO_CHANNEL_COUNT;
}

static uint8_t RadioNode::getPrevChannel(uint8_t channel) {
    return (RADIO_CHANNEL_COUNT + channel - RADIO_HOP_OFFSET) % RADIO_CHANNEL_COUNT;
}

void RadioNode::hopFrequency(bool forward, uint8_t fromChannel, uint32_t timestamp) {
    _channelEntryMillis = timestamp;

    if (forward) {
        _channel = RadioNode::getNextChannel(fromChannel);
    } else {
        _channel = RadioNode::getPrevChannel(fromChannel);
    }

    // And set hardware
    LoRa.sleep();
    LoRa.setFrequency(
        RadioNode::getFrequencyForChannel(_channel)
    );
    LoRa.idle();
}