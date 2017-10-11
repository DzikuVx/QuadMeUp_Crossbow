#include "Arduino.h"
#include "variables.h"
#include <PPMReader.h>

void qspDecodeRcDataFrame(QspConfiguration_t *qsp, int output[]) {
    int temporaryPpmOutput[PPM_CHANNEL_COUNT] = {0};
    //TODO fix it, baby :)

    temporaryPpmOutput[0] = (uint16_t) (((uint16_t) qsp->payload[0] << 2) & 0x3fc) | ((qsp->payload[1] >> 6) & 0x03);
    temporaryPpmOutput[1] = (uint16_t) (((uint16_t) qsp->payload[1] << 4) & 0x3f0) | ((qsp->payload[2] >> 4) & 0x0F);
    temporaryPpmOutput[2] = (uint16_t) (((uint16_t) qsp->payload[2] << 6) & 0x3c0) | ((qsp->payload[3] >> 2) & 0x3F);
    temporaryPpmOutput[3] = (uint16_t) (((uint16_t) qsp->payload[3] << 8) & 0x300) | ((qsp->payload[4] >> 2) & 0xFF);
    temporaryPpmOutput[4] = qsp->payload[5];
    temporaryPpmOutput[5] = qsp->payload[6];
    temporaryPpmOutput[6] = (qsp->payload[7] >> 4) & 0b00001111;
    temporaryPpmOutput[7] = qsp->payload[7] & 0b00001111;
    temporaryPpmOutput[8] = (qsp->payload[8] >> 4) & 0b00001111;
    temporaryPpmOutput[9] = qsp->payload[8] & 0b00001111;

    //10bit channels
    temporaryPpmOutput[0] = map(temporaryPpmOutput[0], 0, 1000, 1000, 2000);
    temporaryPpmOutput[1] = map(temporaryPpmOutput[1], 0, 1000, 1000, 2000);
    temporaryPpmOutput[2] = map(temporaryPpmOutput[2], 0, 1000, 1000, 2000);
    temporaryPpmOutput[3] = map(temporaryPpmOutput[3], 0, 1000, 1000, 2000);

    //8bit channels
    temporaryPpmOutput[4] = map(temporaryPpmOutput[4], 0, 0xff, 1000, 2000);
    temporaryPpmOutput[5] = map(temporaryPpmOutput[5], 0, 0xff, 1000, 2000);

    //4bit channels
    temporaryPpmOutput[6] = map(temporaryPpmOutput[6], 0, 0x0f, 1000, 2000);
    temporaryPpmOutput[7] = map(temporaryPpmOutput[7], 0, 0x0f, 1000, 2000);
    temporaryPpmOutput[8] = map(temporaryPpmOutput[8], 0, 0x0f, 1000, 2000);
    temporaryPpmOutput[9] = map(temporaryPpmOutput[9], 0, 0x0f, 1000, 2000);

    /*
     * Copy tremporary to real output
     */
    for (uint8_t i = 0; i < PPM_CHANNEL_COUNT; i++) {
        output[i] = temporaryPpmOutput[i];
    }
}

uint8_t get10bitHighShift(uint8_t channel) {
    return ((channel % 4) * 2) + 2;
}

uint8_t get10bitLowShift(uint8_t channel) {
    return 8 - get10bitHighShift(channel);
}

void qspComputeCrc(QspConfiguration_t *qsp, uint8_t dataByte)
{
    qsp->crc ^= dataByte;
}

/**
 * Encode 10 RC channels 
 */
void encodeRcDataPayload(QspConfiguration_t *qsp, PPMReader *ppmSource, uint8_t noOfChannels)
{
    for (uint8_t i = 0; i < noOfChannels; i++)
    {
        uint16_t channelValue10 = map(ppmSource->get(i), 1000, 2000, 0, 1000) & 0x03ff;
        uint8_t channelValue8 = map(ppmSource->get(i), 1000, 2000, 0, 255) & 0xff;
        uint8_t channelValue4 = map(ppmSource->get(i), 1000, 2000, 0, 15) & 0x0f;

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

    qsp->payloadLength = 9;
}

uint8_t qspGetPacketId()
{
    static uint8_t packetId = 0;

    return packetId++;
}

void qspClearPayload(QspConfiguration_t *qsp)
{
    for (uint8_t i = 0; i < QSP_PAYLOAD_LENGTH; i++)
    {
        qsp->payload[i] = 0;
    }
    qsp->payloadLength = 0;
}

void qspDecodeIncomingFrame(QspConfiguration_t *qsp, uint8_t incomingByte, int ppm[])
{
    static uint8_t frameId;
    static uint8_t payloadLength;
    static uint8_t receivedPayload;
    static uint8_t packetId; //TODO move this to global scope maybe?

    if (qsp->protocolState == QSP_STATE_IDLE && incomingByte == QSP_PREAMBLE)
    {
        //If in IDLE and correct preamble comes, start to decode frame
        qsp->protocolState = QSP_STATE_PREAMBLE_RECEIVED;
        qsp->crc = 0 ^ incomingByte;
    }
    else if (qsp->protocolState == QSP_STATE_PREAMBLE_RECEIVED)
    {
        // Check if incomming channel ID is the same as receiver
        if (incomingByte == CHANNEL_ID)
        {
            qsp->protocolState = QSP_STATE_CHANNEL_RECEIVED;
            qsp->crc ^= incomingByte;

            qspClearPayload(qsp);

            receivedPayload = 0;
            packetId = 0;
        }
        else
        {
            qsp->protocolState = QSP_STATE_IDLE;
        }
    }
    else if (qsp->protocolState == QSP_STATE_CHANNEL_RECEIVED)
    {
        //Frame ID and payload length
        qsp->crc ^= incomingByte;

        frameId = (incomingByte >> 4) & 0x0f;
        payloadLength = incomingByte & 0x0f;

        qsp->protocolState = QSP_STATE_FRAME_TYPE_RECEIVED;
    }
    else if (qsp->protocolState == QSP_STATE_FRAME_TYPE_RECEIVED)
    {
        qsp->crc ^= incomingByte;
        packetId = incomingByte;
        qsp->protocolState = QSP_STATE_PACKET_ID_RECEIVED;
    }
    else if (qsp->protocolState == QSP_STATE_PACKET_ID_RECEIVED)
    {

        //Now it's time for payload
        qsp->crc ^= incomingByte;
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

#ifdef DEVICE_MODE_RX
            //If devide received a valid frame, that means it can start to talk
            canTransmit = true;
#endif

            switch (frameId)
            {
            case QSP_FRAME_RC_DATA:
                qsp->lastRcFrameReceived = millis();
                qspDecodeRcDataFrame(qsp, ppm);
                break;

            default:
                //Unknown frame
                //TODO do something in this case
                break;
            }
        }
        else
        {
            //CRC failed, frame has to be rejected
            //TODO do something in this case or something
        }

        // In both cases switch to listening for next preamble
        qsp->protocolState = QSP_STATE_IDLE;
    }
}

/**
 * Encode frame is corrent format and write to hardware
 */
void qspEncodeFrame(QspConfiguration_t *qsp) {
    //Zero CRC
    qsp->crc = 0;

    //Write preamble
    qsp->hardwareWriteFunction(QSP_PREAMBLE, qsp);
    //Write CHANNEL_ID
    qsp->hardwareWriteFunction(CHANNEL_ID, qsp);

    //Write frame type and length
    uint8_t data = qsp->payloadLength & 0x0f;
    data |= (qsp->frameToSend << 4) & 0xf0;
    qsp->hardwareWriteFunction(data, qsp);

    //Write packet ID
    qsp->hardwareWriteFunction(qspGetPacketId(), qsp);

    //Write payload
    for (uint8_t i = 0; i < qsp->payloadLength; i++)
    {
        qsp->hardwareWriteFunction(qsp->payload[i], qsp);
    }

    //Finally write CRC
    qsp->hardwareWriteFunction(qsp->crc, qsp);
}