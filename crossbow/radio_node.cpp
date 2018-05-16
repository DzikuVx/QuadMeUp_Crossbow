#include "radio_node.h"
#include "lora.h"

RadioNode::RadioNode(void) {

}

void RadioNode::init(uint8_t ss, uint8_t rst, uint8_t di0, void(*callback)(int)) {
    /*
     * Setup hardware
     */
    LoRa.setPins(
        ss,
        rst,
        di0
    );

    if (!LoRa.begin(getFrequencyForChannel(getChannel()))) {
        while (true);
    }

    //Configure LoRa module
    LoRa.setSignalBandwidth(loraBandwidth);
    LoRa.setSpreadingFactor(loraSpreadingFactor);
    LoRa.setCodingRate4(loraCodingRate);
    LoRa.setTxPower(loraTxPower);
    LoRa.enableCrc();

    //Setup ISR callback and start receiving
    LoRa.onReceive(callback);
    LoRa.receive();
    deviceState = RADIO_STATE_RX;
}

void RadioNode::readRssi(void)
{
    rssi = 164 - constrain(LoRa.packetRssi() * -1, 0, 164);
}

void RadioNode::readSnr(void)
{
    snr = (uint8_t) constrain(LoRa.packetSnr(), 0, 255);
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

void RadioNode::readAndDecode(
    QspConfiguration_t *qsp,
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState
) {
    uint8_t tmpBuffer[MAX_PACKET_SIZE];
    /*
     * There is data to be read from radio!
     */
    if (bytesToRead != NO_DATA_TO_READ) {
        LoRa.read(tmpBuffer, bytesToRead);

        for (int i = 0; i < bytesToRead; i++) {
            qspDecodeIncomingFrame(qsp, tmpBuffer[i], rxDeviceState, txDeviceState);
        }

        //After reading, flush radio buffer, we have no need for whatever might be over there
        LoRa.sleep();
        LoRa.receive();

        deviceState = RADIO_STATE_RX;
        bytesToRead = NO_DATA_TO_READ;
    }
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

void RadioNode::handleChannelDwell(void) {
    //In the beginning just keep jumping forward and try to resync over lost single frames
    if (failedDwellsCount < 6 && getChannelEntryMillis() + RX_CHANNEL_DWELL_TIME < millis()) {
        failedDwellsCount++;
        hopFrequency(true, getChannel(), getChannelEntryMillis() + RX_CHANNEL_DWELL_TIME);
        LoRa.receive();
    }

    // If we are loosing more frames, start jumping in the opposite direction since probably we are completely out of sync now
    if (failedDwellsCount >= 6 && getChannelEntryMillis() + (RX_CHANNEL_DWELL_TIME * 5) < millis()) {
        hopFrequency(false, getChannel(), getChannelEntryMillis() + RX_CHANNEL_DWELL_TIME); //Start jumping in opposite direction to resync
        LoRa.receive();
    }
}

void RadioNode::handleTxDoneState(void) {
    uint32_t currentMillis = millis();
    if (
        currentMillis > nextTxCheckMillis &&
        deviceState == RADIO_STATE_TX &&
        !LoRa.isTransmitting()
    ) {

        /*
         * In case of TX module, hop right now
         */
#ifdef DEVICE_MODE_TX
        hopFrequency(true, getChannel(), currentMillis);
#endif

        LoRa.receive();
        deviceState = RADIO_STATE_RX;
        nextTxCheckMillis = currentMillis + 1; //We check of TX done every 1ms
    }
}

void RadioNode::handleTx(QspConfiguration_t *qsp) {
    uint8_t size;
    uint8_t tmpBuffer[MAX_PACKET_SIZE];

    LoRa.beginPacket();
    //Prepare packet
    qspEncodeFrame(qsp, tmpBuffer, &size, getChannel());
    //Sent it to radio in one SPI transaction
    LoRa.write(tmpBuffer, size);
    LoRa.endPacketAsync();

    //Set state to be able to detect the moment when TX is done
    deviceState = RADIO_STATE_TX;
}