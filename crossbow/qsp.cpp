#include "Arduino.h"
#include "variables.h"

void qspDecodeRcDataFrame(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceSate) {
    int temporaryPpmOutput[PPM_OUTPUT_CHANNEL_COUNT] = {0};
    //TODO fix it, baby :)

    temporaryPpmOutput[0] = (uint16_t) (((uint16_t) qsp->payload[0] << 2) & 0x3fc) | ((qsp->payload[1] >> 6) & 0x03);
    temporaryPpmOutput[1] = (uint16_t) (((uint16_t) qsp->payload[1] << 4) & 0x3f0) | ((qsp->payload[2] >> 4) & 0x0F);
    temporaryPpmOutput[2] = (uint16_t) (((uint16_t) qsp->payload[2] << 6) & 0x3c0) | ((qsp->payload[3] >> 2) & 0x3F);
    temporaryPpmOutput[3] = (uint16_t) (((uint16_t) qsp->payload[3] << 8) & 0x300) | ((qsp->payload[4]) & 0xFF);
    temporaryPpmOutput[4] = qsp->payload[5];
    temporaryPpmOutput[5] = qsp->payload[6];
    temporaryPpmOutput[6] = (qsp->payload[7] >> 4) & 0b00001111;
    temporaryPpmOutput[7] = qsp->payload[7] & 0b00001111;
    temporaryPpmOutput[8] = (qsp->payload[8] >> 4) & 0b00001111;
    temporaryPpmOutput[9] = qsp->payload[8] & 0b00001111;

    //10bit channels are passed as is

    //8bit channels needs to be shifted left 2 places
    temporaryPpmOutput[4] = temporaryPpmOutput[4] << 2;
    temporaryPpmOutput[5] = temporaryPpmOutput[5] << 2;

    //4bit channels needs to be shifted left 6 places
    temporaryPpmOutput[6] = temporaryPpmOutput[6] << 6;
    temporaryPpmOutput[7] = temporaryPpmOutput[7] << 6;
    temporaryPpmOutput[8] = temporaryPpmOutput[8] << 6;
    temporaryPpmOutput[9] = temporaryPpmOutput[9] << 6;

    /*
     * Copy tremporary to real output and add missing 1000
     */
    for (uint8_t i = 0; i < PPM_OUTPUT_CHANNEL_COUNT; i++) {
        rxDeviceSate->channels[i] = temporaryPpmOutput[i] + 1000;
    }
}

uint8_t get10bitHighShift(uint8_t channel) {
    return ((channel % 4) * 2) + 2;
}

uint8_t get10bitLowShift(uint8_t channel) {
    return 8 - get10bitHighShift(channel);
}

uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

void qspComputeCrc(QspConfiguration_t *qsp, uint8_t dataByte)
{
    qsp->crc = crc8_dvb_s2(qsp->crc, dataByte);
}

void encodeRxHealthPayload(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceState, volatile RadioState_t *radioState) {
    qsp->payload[0] = radioState->rssi;
    qsp->payload[1] = radioState->snr;
    qsp->payload[2] = rxDeviceState->rxVoltage;
    qsp->payload[3] = rxDeviceState->a1Voltage;
    qsp->payload[4] = rxDeviceState->a2Voltage;

    uint8_t flags = 0;

    if (qsp->deviceState == DEVICE_STATE_FAILSAFE) {
        flags |= 0x01 << 0;
    }

    qsp->payload[5] = flags;

    qsp->payloadLength = qspFrameLengths[QSP_FRAME_RX_HEALTH];
}

void decodeRxHealthPayload(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceState) {
    rxDeviceState->rssi = qsp->payload[0];
    rxDeviceState->snr = qsp->payload[1];
    rxDeviceState->rxVoltage = qsp->payload[2];
    rxDeviceState->a1Voltage = qsp->payload[3];
    rxDeviceState->a2Voltage = qsp->payload[4];
    rxDeviceState->flags = qsp->payload[5];
}

/**
 * Encode 10 RC channels 
 */
void encodeRcDataPayload(QspConfiguration_t *qsp, volatile int16_t channels[], uint8_t noOfChannels)
{
    for (uint8_t i = 0; i < noOfChannels; i++)
    {
        int cV = constrain(channels[i], 1000, 2000) - 1000;

        uint16_t channelValue10 = cV & 0x03ff;
        uint8_t channelValue8   = (cV >> 2) & 0xff;
        uint8_t channelValue4   = (cV >> 6) & 0x0f;

        if (i < 4)
        {
            /*
             * First 4 channels encoded with 10 bits
             */
            uint8_t bitIndex = i + (i / 4);
            qsp->payload[bitIndex] |= (channelValue10 >> get10bitHighShift(i)) & (0x3ff >> get10bitHighShift(i));
            qsp->payload[bitIndex + 1] |= (channelValue10 << get10bitLowShift(i)) & 0xff << (8 - get10bitHighShift(i));
        }
        else if (i == 4 || i == 5)
        {
            /*
             * Next 2 with 8 bits
             */
            qsp->payload[i + 1] |= channelValue8;
        }
        else if (i == 6)
        {
            /*
             * And last 4 with 4 bits per channel
             */
            qsp->payload[7] |= (channelValue4 << 4) & B11110000;
        }
        else if (i == 7)
        {
            qsp->payload[7] |= channelValue4 & B00001111;
        }
        else if (i == 8)
        {
            qsp->payload[8] |= (channelValue4 << 4) & B11110000;
        }
        else if (i == 9)
        {
            qsp->payload[8] |= channelValue4 & B00001111;
        }
    }

    qsp->payloadLength = qspFrameLengths[QSP_FRAME_RC_DATA];
}

void qspClearPayload(QspConfiguration_t *qsp)
{
    for (uint8_t i = 0; i < QSP_PAYLOAD_LENGTH; i++)
    {
        qsp->payload[i] = 0;
    }
    qsp->payloadLength = 0;
}

void qspDecodeIncomingFrame(
    QspConfiguration_t *qsp, 
    uint8_t incomingByte, 
    RxDeviceState_t *rxDeviceState, 
    TxDeviceState_t *txDeviceState,
    volatile RadioState_t *radioState
) {
    static uint8_t frameId;
    static uint8_t payloadLength;
    static uint8_t receivedPayload;
    static uint8_t receivedChannel;

    if (qsp->protocolState == QSP_STATE_IDLE)
    {
        // Check if incomming channel ID is the same as receiver
        if (incomingByte == CHANNEL_ID)
        {
            qsp->frameDecodingStartedAt = millis();
            qsp->protocolState = QSP_STATE_CHANNEL_RECEIVED;
            qsp->crc = 0;
            qspComputeCrc(qsp, incomingByte);
            qspClearPayload(qsp);

            receivedPayload = 0;
        }
        else
        {
            qsp->protocolState = QSP_STATE_IDLE;
        }
    }
    else if (qsp->protocolState == QSP_STATE_CHANNEL_RECEIVED)
    {
        //Frame ID and payload length
        qspComputeCrc(qsp, incomingByte);

        qsp->frameId = (incomingByte >> 4) & 0x0f;
        payloadLength = qspFrameLengths[qsp->frameId];
        receivedChannel = incomingByte & 0x0f;
        qsp->protocolState = QSP_STATE_FRAME_TYPE_RECEIVED;
    }
    else if (qsp->protocolState == QSP_STATE_FRAME_TYPE_RECEIVED)
    {
        if (receivedPayload >= QSP_PAYLOAD_LENGTH) {
            qsp->protocolState = QSP_STATE_IDLE;
        }

        //Now it's time for payload
        qspComputeCrc(qsp, incomingByte);
        qsp->payload[receivedPayload] = incomingByte;

        receivedPayload++;

        if (receivedPayload == payloadLength)
        {
            qsp->protocolState = QSP_STATE_PAYLOAD_RECEIVED;
            qsp->payloadLength = payloadLength;
        }
    }
    else if (qsp->protocolState == QSP_STATE_PAYLOAD_RECEIVED)
    {
        if (qsp->crc == incomingByte) {
            //CRC is correct
            radioState->lastReceivedChannel = receivedChannel;
            qsp->onSuccessCallback(qsp, txDeviceState, rxDeviceState, radioState);
        } else {
            qsp->onFailureCallback(qsp, txDeviceState, rxDeviceState, radioState);
        }

        // In both cases switch to listening for next preamble
        qsp->protocolState = QSP_STATE_IDLE;
    }
}

/**
 * Encode frame is corrent format and write to hardware
 */
void qspEncodeFrame(QspConfiguration_t *qsp, volatile RadioState_t *radioState, uint8_t buffer[], uint8_t *size) {
    //Zero CRC
    qsp->crc = 0;

    qspComputeCrc(qsp, CHANNEL_ID);
    buffer[0] = CHANNEL_ID;

    //Write frame type and length
    // We are no longer sending payload length, so 4 bits are now free for other usages
    // uint8_t data = qsp->payloadLength & 0x0f;
    uint8_t data = radioState->channel;
    data |= (qsp->frameToSend << 4) & 0xf0;
    qspComputeCrc(qsp, data);
    buffer[1] = data;

    for (uint8_t i = 0; i < qsp->payloadLength; i++)
    {
        qspComputeCrc(qsp, qsp->payload[i]);
        buffer[i + 2] = qsp->payload[i];
    }

    buffer[qsp->payloadLength + 2] = qsp->crc;
    *size = qsp->payloadLength + 3; //Total length of QSP frame
}

void encodePingPayload(QspConfiguration_t *qsp, uint32_t currentMicros) {
    qsp->payload[0] = currentMicros & 255;
    qsp->payload[1] = (currentMicros >> 8) & 255;
    qsp->payload[2] = (currentMicros >> 16) & 255;
    qsp->payload[3] = (currentMicros >> 24) & 255;

    qsp->payloadLength = qspFrameLengths[QSP_FRAME_PING];
}